
/**
 * Ring Buffer Implementation for WLAN Packet Descriptors
 * Used in: TX/RX descriptor rings, DMA buffers
 * This is template only for reference.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define RING_SIZE 256  // Ring size Power of 2 for efficient modulo

typedef struct {
    uint8_t *data;
    uint32_t paddr;      // Physical address for DMA
    uint16_t len;
    uint16_t flags;
    uint64_t timestamp;
} packet_desc_t;

typedef struct {
    packet_desc_t *buffer;
    volatile uint32_t head;  // Producer index
    volatile uint32_t tail;  // Consumer index
    uint32_t mask;           // Size - 1 for fast modulo
    uint32_t size;
} ring_buffer_t;

/**
 * Initialize ring buffer
 */
void ring_init(ring_buffer_t *ring, uint32_t size) {
    ring->buffer = (packet_desc_t *)malloc(size * sizeof(packet_desc_t));
    ring->head = 0;
    ring->tail = 0;
    ring->size = size;
    ring->mask = size - 1;  // Works only if size is power of 2
}

/**
 * Check if ring is full
 */
static inline bool ring_full(ring_buffer_t *ring) {
    return ((ring->head - ring->tail) & ring->mask) == ring->mask;
}

/**
 * Check if ring is empty
 */
static inline bool ring_empty(ring_buffer_t *ring) {
    return ring->head == ring->tail;
}

/**
 * Get available space
 */
static inline uint32_t ring_space(ring_buffer_t *ring) {
    return ring->size - ((ring->head - ring->tail) & ring->mask) - 1;
}

/**
 * Enqueue packet descriptor (Producer)
 * Returns: true on success, false if full
 */
bool ring_enqueue(ring_buffer_t *ring, packet_desc_t *desc) {
    if (ring_full(ring)) {
        return false;
    }
    
    uint32_t head = ring->head;
    memcpy(&ring->buffer[head], desc, sizeof(packet_desc_t));
    
    // Memory barrier to ensure write completes before updating head
    __sync_synchronize();
    
    ring->head = (head + 1) & ring->mask;
    return true;
}

/**
 * Dequeue packet descriptor (Consumer)
 * Returns: true on success, false if empty
 */
bool ring_dequeue(ring_buffer_t *ring, packet_desc_t *desc) {
    if (ring_empty(ring)) {
        return false;
    }
    
    uint32_t tail = ring->tail;
    memcpy(desc, &ring->buffer[tail], sizeof(packet_desc_t));
    
    __sync_synchronize();
    
    ring->tail = (tail + 1) & ring->mask;
    return true;
}

/**
 * Peek at next descriptor without removing
 */
packet_desc_t* ring_peek(ring_buffer_t *ring) {
    if (ring_empty(ring)) {
        return NULL;
    }
    return &ring->buffer[ring->tail];
}

/**
 * Bulk enqueue - efficient for batch packet processing
 */
uint32_t ring_enqueue_bulk(ring_buffer_t *ring, packet_desc_t *descs, uint32_t count) {
    uint32_t space = ring_space(ring);
    uint32_t n = (count < space) ? count : space;
    
    if (n == 0) return 0;
    
    uint32_t head = ring->head;
    for (uint32_t i = 0; i < n; i++) {
        memcpy(&ring->buffer[(head + i) & ring->mask], &descs[i], sizeof(packet_desc_t));
    }
    
    __sync_synchronize();
    ring->head = (head + n) & ring->mask;
    
    return n;
}

/**
 * Example usage in WLAN driver
 */
void example_tx_processing() {
    ring_buffer_t tx_ring;
    ring_init(&tx_ring, RING_SIZE);
    
    // Prepare packet descriptor
    packet_desc_t desc = {
        .data = NULL,  // Pointer to packet buffer
        .paddr = 0x12345000,
        .len = 1500,
        .flags = 0x01,  // TX_FLAG_ACK_REQUIRED
        .timestamp = 0
    };
    
    // Enqueue for transmission
    if (ring_enqueue(&tx_ring, &desc)) {
        // Notify hardware/firmware
        // write_register(TX_DOORBELL_REG, 1);
    }
}

