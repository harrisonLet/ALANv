#include "system.h"

#include "objRing.h"

void objRing_init(Ring_o *r, void *storage, uint16_t elemSize, uint16_t capacity)
{
    r->buf = storage; 
    r->elemSize = elemSize; 
    r->capacity = capacity;
    
    r->head = 0; 
    r->count = 0;
}

// Returns true and copies the replaced element into evicted (if non-NULL) once the ring is full.
bool objRing_push(Ring_o *r, const void *in, void *evicted)
{
    uint16_t next = r->count ? (r->head + 1) % r->capacity : 0;
    uint8_t *slot = r->buf + (size_t)next * r->elemSize;
    bool full = (r->count == r->capacity);

    if (full && evicted) memcpy(evicted, slot, r->elemSize);   // slot holds the oldest
    memcpy(slot, in, r->elemSize);
    r->head = next;
    if (!full) r->count++;
    return full;
}

// age 0 = newest, 1 = the one before it, and so on.
bool objRing_get(const Ring_o *r, uint16_t age, void *out)
{
    if (age >= r->count) return false;
    uint16_t idx = (r->head + r->capacity - age) % r->capacity;
    memcpy(out, r->buf + (size_t)idx * r->elemSize, r->elemSize);
    return true;
}