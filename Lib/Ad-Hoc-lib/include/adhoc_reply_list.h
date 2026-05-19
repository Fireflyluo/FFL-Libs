#ifndef ADHOC_REPLY_LIST_H
#define ADHOC_REPLY_LIST_H

#include "adhoc_frame.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_REPLY_LIST_CAPACITY 24u
#define ADHOC_REPLY_MAX_PER_FRAME 5u
#define ADHOC_REPLY_ITEM_ENCODED_LEN 4u
#define ADHOC_A_CONTENT_LMT_A_LEN 4u
#define ADHOC_A_CONFIRM_PAYLOAD_OFFSET ADHOC_A_CONTENT_LMT_A_LEN
#define ADHOC_A_CONFIRM_PAYLOAD_BYTES (ADHOC_REPLY_MAX_PER_FRAME * ADHOC_REPLY_ITEM_ENCODED_LEN)
#define ADHOC_A_JOIN_HEAD_BYTES ADHOC_REPLY_ITEM_ENCODED_LEN

#if (ADHOC_A_CONFIRM_PAYLOAD_OFFSET + ADHOC_A_CONFIRM_PAYLOAD_BYTES) > ADHOC_FRAME_CONTENT_LEN
#error "Confirm payload exceeds content length (24)."
#endif

#if (ADHOC_A_CONFIRM_PAYLOAD_OFFSET + ADHOC_A_JOIN_HEAD_BYTES + ((ADHOC_REPLY_MAX_PER_FRAME - 1u) * ADHOC_REPLY_ITEM_ENCODED_LEN)) > ADHOC_FRAME_CONTENT_LEN
#error "Join response exceeds content length (24)."
#endif

typedef struct
{
    adhoc_payload_id_t ids[ADHOC_REPLY_LIST_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} adhoc_reply_list_t;

void adhoc_reply_list_reset(adhoc_reply_list_t *list);
int adhoc_reply_list_push_unique(adhoc_reply_list_t *list, adhoc_payload_id_t id);
int adhoc_reply_list_push_confirm_unique(adhoc_reply_list_t *list, uint32_t node_id);
int adhoc_reply_list_pop(adhoc_reply_list_t *list, adhoc_payload_id_t *out_id);
uint8_t adhoc_reply_list_size(const adhoc_reply_list_t *list);
int adhoc_reply_list_build_confirm_payload(adhoc_reply_list_t *list, uint8_t content[ADHOC_FRAME_CONTENT_LEN],
                                           uint8_t content_offset, uint8_t max_ids, uint8_t *out_used_ids);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_REPLY_LIST_H */
