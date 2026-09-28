// litegrip_shm.cpp — implementation of the shared memory contract
// (seqlock + POSIX shm).
//
// Memory ordering notes
// ---------------------
// A kernel-style seqlock is used:
//
//   writer: seq++ (→odd); release fence; write data; release fence; seq++ (→even)
//   reader: s0 = seq (acquire); retry if s0 is odd; read data; acquire fence;
//           retry if seq != s0 (the data may have been torn)
//
// On weakly ordered architectures (aarch64 and the like) the acquire/release
// fences generate real barrier instructions; on x86-64 they degrade to compiler
// barriers — the semantics are correct either way.
//
// The reader never blocks and never takes a lock, so it is safe to put on a
// ros2_control real-time thread; a reader retry never spins waiting for the
// writer (the writer is the non-real-time daemon), it only re-reads now and
// then inside a tiny torn window, and the caller controls the retry limit.

#include "litegrip_ros2_control/litegrip_shm.h"

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

// ─────────────────────────── segment internal layout ───────────────────────────
//
// Corresponds one to one with the public structs in litegrip_shm.h; the public
// API exposes only LitegripState / LitegripCommand / LitegripHeader, and the
// segment's internal layout is not visible to callers.

struct SharedSegment {
  uint32_t magic;
  uint32_t layout_version;
  uint64_t state_seq;             // seqlock write sequence (even = stable)
  uint64_t command_seq;           // seqlock write sequence
  uint64_t state_publish_count;   // diagnostics: state publish count
  uint64_t command_publish_count; // diagnostics: command publish count
  uint64_t state_torn_reads;      // diagnostics: state torn-read backoffs
  uint64_t command_torn_reads;    // diagnostics: command torn-read backoffs
  LitegripState state;
  LitegripCommand command;
};

// Layout pinned down: every field is a double/uint64, so natural alignment
// should leave no implicit padding at all.
static_assert(sizeof(double) == 8, "double must be 8 bytes");
static_assert(alignof(LitegripState) == 8, "LitegripState alignment is wrong");
static_assert(alignof(LitegripCommand) == 8, "LitegripCommand alignment is wrong");
// LitegripState: 9 one-element joint arrays + 12 scalars = 21 doubles.
static_assert(sizeof(LitegripState) == 8 * 21, "LitegripState has implicit padding");
// LitegripCommand: 1 one-element joint array + 4 scalars = 5 doubles.
static_assert(sizeof(LitegripCommand) == 8 * 5, "LitegripCommand has implicit padding");
// The 2 uint32s (magic + version) exactly fill 8 bytes, followed by 6 uint64s.
static_assert(sizeof(LitegripHeader) == 8 * 7, "LitegripHeader has implicit padding");

static_assert(offsetof(SharedSegment, magic) == 0, "layout drift");
static_assert(offsetof(SharedSegment, layout_version) == 4, "layout drift");
static_assert(offsetof(SharedSegment, state_seq) == 8, "layout drift");
static_assert(offsetof(SharedSegment, command_seq) == 16, "layout drift");
static_assert(offsetof(SharedSegment, state_publish_count) == 24, "layout drift");
static_assert(offsetof(SharedSegment, command_publish_count) == 32, "layout drift");
static_assert(offsetof(SharedSegment, state_torn_reads) == 40, "layout drift");
static_assert(offsetof(SharedSegment, command_torn_reads) == 48, "layout drift");
static_assert(offsetof(SharedSegment, state) == 56, "layout drift");
static_assert(offsetof(SharedSegment, command) == 56 + 168, "layout drift");
static_assert(sizeof(SharedSegment) == 264, "layout drift");

// The seqlock sequence numbers must be accessible atomically without a lock.
static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "uint64 atomics are not lock-free on this platform, "
              "so the seqlock cannot hold");

inline std::atomic<uint64_t> &atom(uint64_t *slot) {
  return *reinterpret_cast<std::atomic<uint64_t> *>(slot);
}

// ─────────────────────────── seqlock primitives ───────────────────────────

inline void seq_write_begin(std::atomic<uint64_t> &seq) {
  seq.fetch_add(1, std::memory_order_relaxed);          // enter write critical section (→odd)
  std::atomic_thread_fence(std::memory_order_release);  // data writes must not float above
}

inline void seq_write_end(std::atomic<uint64_t> &seq) {
  std::atomic_thread_fence(std::memory_order_release);  // data writes must be visible before
  seq.fetch_add(1, std::memory_order_relaxed);          // leave write critical section (→even)
}

inline bool seq_read_begin(std::atomic<uint64_t> &seq, uint64_t *stamp) {
  *stamp = seq.load(std::memory_order_acquire);
  return (*stamp & 1u) == 0u;  // odd means the writer is writing
}

inline bool seq_read_retry(std::atomic<uint64_t> &seq, uint64_t stamp) {
  std::atomic_thread_fence(std::memory_order_acquire);  // data reads must not sink below
  return seq.load(std::memory_order_relaxed) != stamp;
}

// A generalized "read one block of seqlock-protected data". max_retries < 0
// means retry without limit. During a torn-read retry it issues relax to yield
// the pipeline and avoid contending with the writer for the cache line.
template <typename T>
int seq_read_block(std::atomic<uint64_t> &seq, std::atomic<uint64_t> &torn_counter,
                   const T *src, T *out, int max_retries) {
  for (int attempt = 0;; ++attempt) {
    uint64_t stamp = 0;
    if (!seq_read_begin(seq, &stamp)) {
      // The writer is writing: do not count it as torn, just retry (no
      // spin-wait, yield the scheduler).
      if (max_retries >= 0 && attempt >= max_retries) {
        torn_counter.fetch_add(1, std::memory_order_relaxed);
        return LITEGRIP_SHM_TORN;
      }
      continue;
    }
    std::memcpy(out, src, sizeof(T));
    if (!seq_read_retry(seq, stamp)) {
      return LITEGRIP_SHM_OK;
    }
    torn_counter.fetch_add(1, std::memory_order_relaxed);
    if (max_retries >= 0 && attempt >= max_retries) {
      return LITEGRIP_SHM_TORN;
    }
  }
}

template <typename T>
int seq_write_block(std::atomic<uint64_t> &seq, T *dst, const T *src) {
  seq_write_begin(seq);
  std::memcpy(dst, src, sizeof(T));
  seq_write_end(seq);
  return LITEGRIP_SHM_OK;
}

// ─────────────────────────── handle and initialization ───────────────────────────

struct Handle {
  int fd = -1;
  SharedSegment *segment = nullptr;
  size_t size = 0;
};

inline Handle *as_handle(litegrip_shm_handle_t h) { return static_cast<Handle *>(h); }

// Fill the segment with initial values readable by the current process identity
// (no hardware assumptions are made).
void initialize_segment(SharedSegment *segment) {
  std::memset(segment, 0, sizeof(SharedSegment));
  segment->magic = LITEGRIP_SHM_MAGIC;
  segment->layout_version = LITEGRIP_SHM_LAYOUT_VERSION;
  // The seqlock starts at an even value (the stable state).
  atom(&segment->state_seq).store(0, std::memory_order_relaxed);
  atom(&segment->command_seq).store(0, std::memory_order_relaxed);
}

bool layout_matches(const SharedSegment *segment) {
  return segment->magic == LITEGRIP_SHM_MAGIC &&
         segment->layout_version == LITEGRIP_SHM_LAYOUT_VERSION;
}

}  // namespace

extern "C" {

size_t litegrip_shm_state_size(void) { return sizeof(LitegripState); }
size_t litegrip_shm_command_size(void) { return sizeof(LitegripCommand); }
size_t litegrip_shm_header_size(void) { return sizeof(LitegripHeader); }
size_t litegrip_shm_segment_size(void) { return sizeof(SharedSegment); }

int litegrip_shm_open(const char *name, int create, litegrip_shm_handle_t *out) {
  if (name == nullptr || out == nullptr) {
    return LITEGRIP_SHM_ERR_INVALID_ARG;
  }
  *out = nullptr;

  // An existing segment can simply be opened O_RDWR; only add O_CREAT when it
  // is absent (create). Note that O_CREAT|O_EXCL must not be used to test for
  // existence: it would make concurrent creators fail against each other.
  int fd = ::shm_open(name, create ? (O_CREAT | O_RDWR) : O_RDWR, 0666);
  if (fd < 0) {
    return (errno == ENOENT && !create) ? LITEGRIP_SHM_ERR_LAYOUT
                                        : LITEGRIP_SHM_ERR_OPEN;
  }

  struct stat st {};
  if (::fstat(fd, &st) != 0) {
    ::close(fd);
    return LITEGRIP_SHM_ERR_OPEN;
  }

  const bool needs_grow = static_cast<size_t>(st.st_size) != sizeof(SharedSegment);
  if (needs_grow) {
    if (!create) {
      ::close(fd);
      return LITEGRIP_SHM_ERR_LAYOUT;
    }
    if (::ftruncate(fd, static_cast<off_t>(sizeof(SharedSegment))) != 0) {
      ::close(fd);
      return LITEGRIP_SHM_ERR_TRUNCATE;
    }
  }

  void *base = ::mmap(nullptr, sizeof(SharedSegment), PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
  if (base == MAP_FAILED) {
    ::close(fd);
    return LITEGRIP_SHM_ERR_MAP;
  }

  auto *segment = static_cast<SharedSegment *>(base);

  const bool fresh = needs_grow || st.st_size == 0 || !layout_matches(segment);
  if (fresh) {
    if (!create) {
      ::munmap(base, sizeof(SharedSegment));
      ::close(fd);
      return layout_matches(segment) ? LITEGRIP_SHM_ERR_GENERIC
                                     : LITEGRIP_SHM_ERR_VERSION;
    }
    // A stale segment (the daemon exited abnormally last time) or a version
    // mismatch: rebuild it from scratch. This resets the seqlock to an even
    // value so that readers cannot get stuck on a writer's odd sequence number.
    initialize_segment(segment);
  } else if (create) {
    // The segment already exists and its layout is correct, but the creating
    // side (the daemon) is taking it over: still force the seqlock to reset,
    // because the previous owner's write critical section may not have closed.
    initialize_segment(segment);
  }

  auto *handle = new (std::nothrow) Handle();
  if (handle == nullptr) {
    ::munmap(base, sizeof(SharedSegment));
    ::close(fd);
    return LITEGRIP_SHM_ERR_GENERIC;
  }
  handle->fd = fd;
  handle->segment = segment;
  handle->size = sizeof(SharedSegment);
  *out = handle;
  return LITEGRIP_SHM_OK;
}

void litegrip_shm_close(litegrip_shm_handle_t handle) {
  Handle *h = as_handle(handle);
  if (h == nullptr) {
    return;
  }
  if (h->segment != nullptr) {
    ::munmap(h->segment, h->size);
  }
  if (h->fd >= 0) {
    ::close(h->fd);
  }
  delete h;
}

int litegrip_shm_unlink(const char *name) {
  if (name == nullptr) {
    return LITEGRIP_SHM_ERR_INVALID_ARG;
  }
  if (::shm_unlink(name) == 0 || errno == ENOENT) {
    return LITEGRIP_SHM_OK;
  }
  return LITEGRIP_SHM_ERR_GENERIC;
}

int litegrip_shm_publish_state(litegrip_shm_handle_t handle,
                               const LitegripState *state) {
  Handle *h = as_handle(handle);
  if (h == nullptr || state == nullptr) {
    return LITEGRIP_SHM_ERR_INVALID_ARG;
  }
  seq_write_block(atom(&h->segment->state_seq), &h->segment->state, state);
  atom(&h->segment->state_publish_count).fetch_add(1, std::memory_order_relaxed);
  return LITEGRIP_SHM_OK;
}

int litegrip_shm_read_state(litegrip_shm_handle_t handle, LitegripState *out,
                            int max_retries) {
  Handle *h = as_handle(handle);
  if (h == nullptr || out == nullptr) {
    return LITEGRIP_SHM_ERR_INVALID_ARG;
  }
  return seq_read_block(atom(&h->segment->state_seq),
                        atom(&h->segment->state_torn_reads), &h->segment->state,
                        out, max_retries);
}

int litegrip_shm_publish_command(litegrip_shm_handle_t handle,
                                 const LitegripCommand *command) {
  Handle *h = as_handle(handle);
  if (h == nullptr || command == nullptr) {
    return LITEGRIP_SHM_ERR_INVALID_ARG;
  }
  seq_write_block(atom(&h->segment->command_seq), &h->segment->command, command);
  atom(&h->segment->command_publish_count)
      .fetch_add(1, std::memory_order_relaxed);
  return LITEGRIP_SHM_OK;
}

int litegrip_shm_read_command(litegrip_shm_handle_t handle,
                              LitegripCommand *out, int max_retries) {
  Handle *h = as_handle(handle);
  if (h == nullptr || out == nullptr) {
    return LITEGRIP_SHM_ERR_INVALID_ARG;
  }
  return seq_read_block(atom(&h->segment->command_seq),
                        atom(&h->segment->command_torn_reads),
                        &h->segment->command, out, max_retries);
}

int litegrip_shm_read_header(litegrip_shm_handle_t handle, LitegripHeader *out) {
  Handle *h = as_handle(handle);
  if (h == nullptr || out == nullptr) {
    return LITEGRIP_SHM_ERR_INVALID_ARG;
  }
  SharedSegment *segment = h->segment;
  out->magic = segment->magic;
  out->layout_version = segment->layout_version;
  out->state_seq = atom(&segment->state_seq).load(std::memory_order_relaxed);
  out->command_seq = atom(&segment->command_seq).load(std::memory_order_relaxed);
  out->state_publish_count =
      atom(&segment->state_publish_count).load(std::memory_order_relaxed);
  out->command_publish_count =
      atom(&segment->command_publish_count).load(std::memory_order_relaxed);
  out->state_torn_reads =
      atom(&segment->state_torn_reads).load(std::memory_order_relaxed);
  out->command_torn_reads =
      atom(&segment->command_torn_reads).load(std::memory_order_relaxed);
  return LITEGRIP_SHM_OK;
}

}  // extern "C"
