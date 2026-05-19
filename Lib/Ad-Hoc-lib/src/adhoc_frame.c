#include "adhoc_frame.h"

#include "adhoc_crc8.h"

#include <string.h>

static uint64_t adhoc_sender_raw_make(uint16_t domain_id, uint32_t node_id)
{
    uint64_t raw = 0u;

    raw |= ((uint64_t)(domain_id & ADHOC_SENDER_DOMAIN_MAX) << 29);
    raw |= (uint64_t)(node_id & ADHOC_SENDER_NODE_MAX);
    return raw;
}

static void adhoc_sender_raw_split(uint64_t raw, adhoc_sender_t *sender)
{
    sender->domain_id = (uint16_t)((raw >> 29) & ADHOC_SENDER_DOMAIN_MAX);
    sender->node_id = (uint32_t)(raw & ADHOC_SENDER_NODE_MAX);
}

static uint32_t adhoc_seconds_from_us(uint32_t now_us)
{
    return now_us / 1000000u;
}

uint8_t adhoc_frame_header_make(uint8_t msg_class, uint8_t gateway_no, uint8_t slot_high4)
{
    uint8_t header = 0u;

    header |= (uint8_t)((msg_class & 0x01u) << 7);
    header |= (uint8_t)((gateway_no & 0x07u) << 4);
    header |= (uint8_t)(slot_high4 & 0x0Fu);
    return header;
}

void adhoc_frame_header_parse(uint8_t header_byte, uint8_t *msg_class, uint8_t *gateway_no, uint8_t *slot_high4)
{
    if (msg_class != 0)
    {
        *msg_class = (uint8_t)((header_byte >> 7) & 0x01u);
    }
    if (gateway_no != 0)
    {
        *gateway_no = (uint8_t)((header_byte >> 4) & 0x07u);
    }
    if (slot_high4 != 0)
    {
        *slot_high4 = (uint8_t)(header_byte & 0x0Fu);
    }
}

int adhoc_sender_pack(adhoc_sender_t sender, uint8_t out[ADHOC_FRAME_SENDER_LEN])
{
    uint64_t raw;

    if (out == 0)
    {
        return 0;
    }
    if (sender.domain_id > ADHOC_SENDER_DOMAIN_MAX || sender.node_id == 0u || sender.node_id > ADHOC_SENDER_NODE_MAX)
    {
        return 0;
    }

    raw = adhoc_sender_raw_make(sender.domain_id, sender.node_id);
    out[0] = (uint8_t)((raw >> 32) & 0xFFu);
    out[1] = (uint8_t)((raw >> 24) & 0xFFu);
    out[2] = (uint8_t)((raw >> 16) & 0xFFu);
    out[3] = (uint8_t)((raw >> 8) & 0xFFu);
    out[4] = (uint8_t)(raw & 0xFFu);
    return 1;
}

int adhoc_sender_unpack(const uint8_t in[ADHOC_FRAME_SENDER_LEN], adhoc_sender_t *sender)
{
    uint64_t raw = 0u;

    if (in == 0 || sender == 0)
    {
        return 0;
    }

    raw |= ((uint64_t)in[0] << 32);
    raw |= ((uint64_t)in[1] << 24);
    raw |= ((uint64_t)in[2] << 16);
    raw |= ((uint64_t)in[3] << 8);
    raw |= (uint64_t)in[4];
    adhoc_sender_raw_split(raw, sender);

    if (sender->node_id == 0u || sender->domain_id > ADHOC_SENDER_DOMAIN_MAX || sender->node_id > ADHOC_SENDER_NODE_MAX)
    {
        return 0;
    }
    return 1;
}

int adhoc_payload_id_pack(adhoc_payload_id_t packed_id, uint8_t out[4])
{
    uint32_t raw = 0u;

    if (out == 0)
    {
        return 0;
    }
    if (packed_id.id_flag > ADHOC_PAYLOAD_ID_FLAG_MAX || packed_id.node_id == 0u || packed_id.node_id > ADHOC_SENDER_NODE_MAX)
    {
        return 0;
    }

    raw |= ((uint32_t)(packed_id.id_flag & ADHOC_PAYLOAD_ID_FLAG_MAX) << 29);
    raw |= (packed_id.node_id & ADHOC_SENDER_NODE_MAX);
    out[0] = (uint8_t)((raw >> 24) & 0xFFu);
    out[1] = (uint8_t)((raw >> 16) & 0xFFu);
    out[2] = (uint8_t)((raw >> 8) & 0xFFu);
    out[3] = (uint8_t)(raw & 0xFFu);
    return 1;
}

int adhoc_payload_id_unpack(const uint8_t in[4], adhoc_payload_id_t *packed_id)
{
    uint32_t raw = 0u;

    if (in == 0 || packed_id == 0)
    {
        return 0;
    }

    raw |= ((uint32_t)in[0] << 24);
    raw |= ((uint32_t)in[1] << 16);
    raw |= ((uint32_t)in[2] << 8);
    raw |= (uint32_t)in[3];
    packed_id->id_flag = (uint8_t)((raw >> 29) & ADHOC_PAYLOAD_ID_FLAG_MAX);
    packed_id->node_id = raw & ADHOC_SENDER_NODE_MAX;

    if (packed_id->node_id == 0u)
    {
        return 0;
    }
    return 1;
}

uint32_t adhoc_lmt_a_from_us(uint32_t now_us)
{
    return adhoc_seconds_from_us(now_us);
}

uint32_t adhoc_lmt_d_from_us(uint32_t now_us)
{
    return adhoc_seconds_from_us(now_us) & 0x00FFFFFFu;
}

void adhoc_u24_be_write(uint32_t value, uint8_t out[3])
{
    if (out == 0)
    {
        return;
    }
    out[0] = (uint8_t)((value >> 16) & 0xFFu);
    out[1] = (uint8_t)((value >> 8) & 0xFFu);
    out[2] = (uint8_t)(value & 0xFFu);
}

uint32_t adhoc_u24_be_read(const uint8_t in[3])
{
    if (in == 0)
    {
        return 0u;
    }
    return ((uint32_t)in[0] << 16) | ((uint32_t)in[1] << 8) | (uint32_t)in[2];
}

void adhoc_u32_be_write(uint32_t value, uint8_t out[4])
{
    if (out == 0)
    {
        return;
    }
    out[0] = (uint8_t)((value >> 24) & 0xFFu);
    out[1] = (uint8_t)((value >> 16) & 0xFFu);
    out[2] = (uint8_t)((value >> 8) & 0xFFu);
    out[3] = (uint8_t)(value & 0xFFu);
}

uint32_t adhoc_u32_be_read(const uint8_t in[4])
{
    if (in == 0)
    {
        return 0u;
    }
    return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) | ((uint32_t)in[2] << 8) | (uint32_t)in[3];
}

int adhoc_frame_build(const adhoc_frame_fields_t *fields, uint8_t out_frame[ADHOC_FRAME_SIZE])
{
    uint8_t sender_bytes[ADHOC_FRAME_SENDER_LEN];

    if (fields == 0 || out_frame == 0)
    {
        return 0;
    }
    if (fields->msg_class > 1u || fields->gateway_no > 7u || fields->slot_high4 > 0x0Fu)
    {
        return 0;
    }
    if (!adhoc_sender_pack(fields->sender, sender_bytes))
    {
        return 0;
    }

    out_frame[ADHOC_FRAME_IDX_HEAD] = adhoc_frame_header_make(fields->msg_class, fields->gateway_no, fields->slot_high4);
    out_frame[ADHOC_FRAME_IDX_LEVEL] = fields->level;
    memcpy(&out_frame[ADHOC_FRAME_IDX_SENDER], sender_bytes, ADHOC_FRAME_SENDER_LEN);
    memcpy(&out_frame[ADHOC_FRAME_IDX_CONTENT], fields->content, ADHOC_FRAME_CONTENT_LEN);
    out_frame[ADHOC_FRAME_IDX_CRC] = 0u;
    adhoc_crc8_write_frame(out_frame);
    return 1;
}

int adhoc_frame_parse(const uint8_t frame[ADHOC_FRAME_SIZE], adhoc_frame_fields_t *out_fields)
{
    if (frame == 0 || out_fields == 0)
    {
        return 0;
    }
    if (!adhoc_crc8_verify_frame(frame))
    {
        return 0;
    }

    adhoc_frame_header_parse(frame[ADHOC_FRAME_IDX_HEAD], &out_fields->msg_class, &out_fields->gateway_no, &out_fields->slot_high4);
    out_fields->level = frame[ADHOC_FRAME_IDX_LEVEL];
    if (!adhoc_sender_unpack(&frame[ADHOC_FRAME_IDX_SENDER], &out_fields->sender))
    {
        return 0;
    }

    memcpy(out_fields->content, &frame[ADHOC_FRAME_IDX_CONTENT], ADHOC_FRAME_CONTENT_LEN);
    out_fields->crc8 = frame[ADHOC_FRAME_IDX_CRC];
    return 1;
}
