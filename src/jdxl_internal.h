/* Helpers shared by jdxl_ph1.c and jdxl_ph2.c. Not installed. */
#ifndef JDXL_INTERNAL_H
#define JDXL_INTERNAL_H

#include "jdxl/jdxl.h"

/* Index of the first pending slot (from idx on) that expects `id`, or -1. Replies arrive in
 * slot order, so a reply for a later slot means the ones before it were missed. */
static inline int jdxl_find_slot(const jdxl_slot_t *slot, uint8_t idx, uint8_t count, uint8_t id)
{
        uint8_t k;

        for (k = idx; k < count; k++) {
                if (slot[k].id == JDXL_ANY_ID || slot[k].id == id) return (int)k;
        }
        return -1;
}

static inline jdxl_opts_t jdxl_opts_or_default(const jdxl_opts_t *opts)
{
        jdxl_opts_t o;

        if (opts != NULL) return *opts;
        o.status_return_level = JDXL_SRL_ALL;
        o.tx_echo             = false;
        return o;
}

/* Duplicate detection over all 256 IDs: returns 1 if `id` was already seen. */
static inline int jdxl_seen(uint32_t seen[8], uint8_t id)
{
        const uint32_t bit = 1u << (id & 31u);

        if (seen[id >> 5] & bit) return 1;
        seen[id >> 5] |= bit;
        return 0;
}

/* Result codes the feed scanner uses internally. */
enum {
        JDXL_RX_MORE,   /* candidate is a valid prefix, wait for more bytes */
        JDXL_RX_SKIP,   /* not a packet start (garbage): slide silently */
        JDXL_RX_DROP,   /* rejected and counted: slide */
        JDXL_RX_DONE    /* complete, valid, expected */
};

#endif /* JDXL_INTERNAL_H */