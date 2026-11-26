// Linux/glibc test-only allocator interposition, including calls from shared
// native solver libraries. Never linked into a production controller.
#include <cstddef>
#include <cstdint>
extern "C" {
void* __libc_malloc(size_t);
void* __libc_calloc(size_t, size_t);
void* __libc_realloc(void*, size_t);
void __libc_free(void*);
void* __libc_memalign(size_t, size_t);
}
namespace {
thread_local bool enabled = false;
thread_local uint64_t allocations = 0;
}  // namespace
extern "C" void cbf_allocation_begin() {
  allocations = 0;
  enabled = true;
}
extern "C" uint64_t cbf_allocation_end() {
  enabled = false;
  return allocations;
}
extern "C" void* malloc(size_t n) {
  if (enabled)
    ++allocations;
  return __libc_malloc(n);
}
extern "C" void* calloc(size_t n, size_t s) {
  if (enabled)
    ++allocations;
  return __libc_calloc(n, s);
}
extern "C" void* realloc(void* p, size_t n) {
  if (enabled)
    ++allocations;
  return __libc_realloc(p, n);
}
extern "C" void free(void* p) { __libc_free(p); }
extern "C" void* aligned_alloc(size_t a, size_t n) {
  if (enabled)
    ++allocations;
  return __libc_memalign(a, n);
}
extern "C" int posix_memalign(void** p, size_t a, size_t n) {
  if (enabled)
    ++allocations;
  *p = __libc_memalign(a, n);
  return *p ? 0 : 12;
}
