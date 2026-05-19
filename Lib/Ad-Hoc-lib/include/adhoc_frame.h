#ifndef ADHOC_FRAME_H
#define ADHOC_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_FRAME_SIZE          32u
#define ADHOC_FRAME_IDX_HEAD      0u
#define ADHOC_FRAME_IDX_LEVEL     1u
#define ADHOC_FRAME_IDX_SENDER    2u
#define ADHOC_FRAME_SENDER_LEN    5u
#define ADHOC_FRAME_IDX_CONTENT   7u
#define ADHOC_FRAME_CONTENT_LEN   24u
#define ADHOC_FRAME_IDX_CRC       31u

#define ADHOC_SENDER_DOMAIN_MAX   0x07FFu
#define ADHOC_SENDER_NODE_MAX     0x1FFFFFFFu
#define ADHOC_PAYLOAD_ID_FLAG_MAX 0x07u
#define ADHOC_PAYLOAD_ID_FLAG_BIND_MIN 0u
#define ADHOC_PAYLOAD_ID_FLAG_BIND_MAX 5u
#define ADHOC_PAYLOAD_ID_FLAG_UPSTREAM_CONFIRMED 6u
#define ADHOC_PAYLOAD_ID_FLAG_JOIN_CONFIRM 7u

typedef enum {
    ADHOC_MSG_CLASS_A = 0u,
    ADHOC_MSG_CLASS_D = 1u
} adhoc_msg_class_t;

typedef struct
{
    uint16_t domain_id;
    uint32_t node_id;
} adhoc_sender_t;

typedef struct
{
    uint8_t id_flag;
    uint32_t node_id;
} adhoc_payload_id_t;

typedef struct
{
    uint8_t msg_class;
    uint8_t gateway_no;
    uint8_t slot_high4;
    uint8_t level;
    adhoc_sender_t sender;
    uint8_t content[ADHOC_FRAME_CONTENT_LEN];
    uint8_t crc8;
} adhoc_frame_fields_t;

uint8_t adhoc_frame_header_make(uint8_t msg_class, uint8_t gateway_no, uint8_t slot_high4);
void adhoc_frame_header_parse(uint8_t header_byte, uint8_t *msg_class, uint8_t *gateway_no, uint8_t *slot_high4);
int adhoc_sender_pack(adhoc_sender_t sender, uint8_t out[ADHOC_FRAME_SENDER_LEN]);
int adhoc_sender_unpack(const uint8_t in[ADHOC_FRAME_SENDER_LEN], adhoc_sender_t *sender);
int adhoc_payload_id_pack(adhoc_payload_id_t packed_id, uint8_t out[4]);
int adhoc_payload_id_unpack(const uint8_t in[4], adhoc_payload_id_t *packed_id);
uint32_t adhoc_lmt_a_from_us(uint32_t now_us);
uint32_t adhoc_lmt_d_from_us(uint32_t now_us);
void adhoc_u24_be_write(uint32_t value, uint8_t out[3]);
uint32_t adhoc_u24_be_read(const uint8_t in[3]);
void adhoc_u32_be_write(uint32_t value, uint8_t out[4]);
uint32_t adhoc_u32_be_read(const uint8_t in[4]);
int adhoc_frame_build(const adhoc_frame_fields_t *fields, uint8_t out_frame[ADHOC_FRAME_SIZE]);
int adhoc_frame_parse(const uint8_t frame[ADHOC_FRAME_SIZE], adhoc_frame_fields_t *out_fields);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_FRAME_H */
