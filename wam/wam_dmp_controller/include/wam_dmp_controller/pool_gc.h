#ifndef ROSRT_DETAIL_POOL_GC_H
#define ROSRT_DETAIL_POOL_GC_H

#include <lockfree/object_pool.h>

namespace rosrt
{
namespace detail
{

template<typename M>
bool poolIsDeletable(void* pool)
{
  return ((lockfree::ObjectPool<M>*)pool)->hasOutstandingAllocations();
}

template<typename M>
void deletePool(void* pool)
{
  delete ((lockfree::ObjectPool<M>*)pool);
}

typedef void(*PoolDeleteFunc)(void* pool);
typedef bool(*PoolDeletableFunc)(void* pool);
void addPoolToGC(void* pool, PoolDeleteFunc deleter, PoolDeletableFunc deletable);

} // namespace detail
} // namespace rosrt

#endif // ROSRT_DETAIL_POOL_GC_H
