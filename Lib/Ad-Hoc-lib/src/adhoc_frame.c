/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_frame.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            帧编解码实现
**                          32B帧的 Header/Sender/Content 字段编解码, 时间戳转换, CRC8 校验
**--------------------------------------------------------------------------------------------------------
*/

#include "adhoc_frame.h"

#include "adhoc_crc8.h"

#include <string.h>

static volatile uint8_t  s_bdt_base_valid = 0u;
static volatile uint32_t s_bdt_base_mono_us = 0u;
static volatile uint32_t s_bdt_base_second = 0u;
static volatile uint32_t s_lmt_a_t5_us = 60000u;

/**
 * @brief 将 domain_id(11bit) + node_id(29bit) 组合为 64bit 中间值
 */
static uint64_t adhoc_sender_raw_make(uint16_t domain_id, uint32_t node_id)
{
    uint64_t raw = 0u;

    raw |= ((uint64_t)(domain_id & ADHOC_SENDER_DOMAIN_MAX) << 29);
    raw |= (uint64_t)(node_id & ADHOC_SENDER_NODE_MAX);
    return raw;
}

/**
 * @brief 从 64bit 中间值拆解出 domain_id 和 node_id
 */
static void adhoc_sender_raw_split(uint64_t raw, adhoc_sender_t *sender)
{
    sender->domain_id = (uint16_t)((raw >> 29) & ADHOC_SENDER_DOMAIN_MAX);
    sender->node_id   = (uint32_t)(raw & ADHOC_SENDER_NODE_MAX);
}

/**
 * @brief 微秒转秒
 */
static uint32_t adhoc_seconds_from_us(uint32_t now_us)
{
    if (s_bdt_base_valid != 0u)
    {
        return s_bdt_base_second + ((uint32_t)(now_us - s_bdt_base_mono_us) / 1000000u);
    }
    return now_us / 1000000u;
}

/**
 * @brief 计算秒内微秒值
 */
static uint32_t adhoc_sub_us_from_us(uint32_t now_us)
{
    if (s_bdt_base_valid != 0u)
    {
        return (uint32_t)(now_us - s_bdt_base_mono_us) % 1000000u;
    }
    return now_us % 1000000u;
}

/**
 * @brief 构造帧头字节: [msg_class:1][gateway_no:3][slot_high4:4]
 */
uint8_t adhoc_frame_header_make(uint8_t msg_class, uint8_t gateway_no, uint8_t slot_high4)
{
    uint8_t header = 0u;

    header |= (uint8_t)((msg_class & 0x01u) << 7);
    header |= (uint8_t)((gateway_no & 0x07u) << 4);
    header |= (uint8_t)(slot_high4 & 0x0Fu);
    return header;
}

/**
 * @brief 解析帧头字节为三个字段
 */
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

/**
 * @brief 打包发送者 ID 为 5 字节大端: domain_id[11] + node_id[29]
 */
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

/**
 * @brief 从 5 字节大端解出发送者 ID
 */
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

/**
 * @brief 打包载荷 ID 为 4 字节大端: id_flag[3] + node_id[29]
 */
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

/**
 * @brief 从 4 字节大端解出载荷 ID
 * @return 1成功 0失败(node_id=0视为非法)
 */
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

/**
 * @brief 从微秒时间戳生成 LMT_A(秒级, 32bit)
 */
uint32_t adhoc_lmt_a_from_us(uint32_t now_us)
{
    uint32_t second = adhoc_seconds_from_us(now_us);
    uint32_t sub_us = adhoc_sub_us_from_us(now_us);
    uint8_t period = 0u;

    if (s_lmt_a_t5_us != 0u)
    {
        uint32_t period_u32 = sub_us / s_lmt_a_t5_us;
        if (period_u32 > ADHOC_LMT_A_PERIOD_MASK)
        {
            period_u32 = ADHOC_LMT_A_PERIOD_MASK;
        }
        period = (uint8_t)period_u32;
    }

    return adhoc_lmt_a_pack(second, period);
}

/**
 * @brief 从微秒时间戳生成 LMT_D(秒级, 24bit)
 */
uint32_t adhoc_lmt_d_from_us(uint32_t now_us)
{
    return adhoc_seconds_from_us(now_us) & 0x00FFFFFFu;
}

void adhoc_time_set_lmt_a_t5_us(uint32_t t5_us)
{
    if (t5_us != 0u)
    {
        s_lmt_a_t5_us = t5_us;
    }
}

uint32_t adhoc_lmt_a_pack(uint32_t bdt_second, uint8_t period)
{
    uint32_t sec_part = bdt_second & ADHOC_LMT_A_SEC_MASK;
    uint32_t period_part = ((uint32_t)period & ADHOC_LMT_A_PERIOD_MASK) << ADHOC_LMT_A_PERIOD_SHIFT;
    return period_part | sec_part;
}

uint32_t adhoc_lmt_a_sec_get(uint32_t lmt_a)
{
    return lmt_a & ADHOC_LMT_A_SEC_MASK;
}

uint8_t adhoc_lmt_a_period_get(uint32_t lmt_a)
{
    return (uint8_t)((lmt_a >> ADHOC_LMT_A_PERIOD_SHIFT) & ADHOC_LMT_A_PERIOD_MASK);
}

void adhoc_time_set_bdt_base(uint32_t mono_us, uint32_t bdt_second)
{
    s_bdt_base_mono_us = mono_us;
    s_bdt_base_second = bdt_second;
    s_bdt_base_valid = 1u;
}

void adhoc_time_clear_bdt_base(void)
{
    s_bdt_base_valid = 0u;
    s_bdt_base_mono_us = 0u;
    s_bdt_base_second = 0u;
}

uint32_t adhoc_time_bdt_seconds_from_mono_us(uint32_t mono_us)
{
    return adhoc_seconds_from_us(mono_us);
}

/**
 * @brief 写入 24bit 大端无符号整数
 */
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

/**
 * @brief 读取 24bit 大端无符号整数
 */
uint32_t adhoc_u24_be_read(const uint8_t in[3])
{
    if (in == 0)
    {
        return 0u;
    }
    return ((uint32_t)in[0] << 16) | ((uint32_t)in[1] << 8) | (uint32_t)in[2];
}

/**
 * @brief 写入 32bit 大端无符号整数
 */
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

/**
 * @brief 读取 32bit 大端无符号整数
 */
uint32_t adhoc_u32_be_read(const uint8_t in[4])
{
    if (in == 0)
    {
        return 0u;
    }
    return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) | ((uint32_t)in[2] << 8) | (uint32_t)in[3];
}

/**
 * @brief 从字段结构体构建完整 32B 帧(自动计算并写入 CRC8)
 */
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

    out_frame[ADHOC_FRAME_IDX_HEAD]    = adhoc_frame_header_make(fields->msg_class, fields->gateway_no, fields->slot_high4);
    out_frame[ADHOC_FRAME_IDX_LEVEL]   = fields->level;
    memcpy(&out_frame[ADHOC_FRAME_IDX_SENDER], sender_bytes, ADHOC_FRAME_SENDER_LEN);
    memcpy(&out_frame[ADHOC_FRAME_IDX_CONTENT], fields->content, ADHOC_FRAME_CONTENT_LEN);
    out_frame[ADHOC_FRAME_IDX_CRC] = 0u;
    adhoc_crc8_write_frame(out_frame);
    return 1;
}

/**
 * @brief 解析 32B 完整帧为字段结构体(先 CRC8 校验, 再逐字段解包)
 */
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
