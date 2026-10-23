#ifndef OBJ_RING_H
#define OBJ_RING_H

typedef struct {
    uint8_t *buf;
    uint16_t elemSize;
    uint16_t capacity;
    uint16_t head;    // slot of the newest element
    uint16_t count;   // grows to capacity, then stays
} Ring_o;

void objRing_init(Ring_o *r, void *storage, uint16_t elemSize, uint16_t capacity);
bool objRing_push(Ring_o *r, const void *in, void *evicted);
bool objRing_get(const Ring_o *r, uint16_t age, void *out);

#endif // OBJ_RING_H