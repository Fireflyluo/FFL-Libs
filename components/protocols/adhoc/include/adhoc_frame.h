/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_frame.h
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            帧编解码模块
**                          提供32B固定帧的字段编解码、发送者ID/载荷ID打包解包、时间戳转换
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_FRAME_H
#define ADHOC_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 协议帧总长度(字节) */
#define ADHOC_FRAME_SIZE          32u
/** @brief Header 字节索引 */
#define ADHOC_FRAME_IDX_HEAD      0u
/** @brief Level 字节索引 */
#define ADHOC_FRAME_IDX_LEVEL     1u
/** @brief Sender 字段起始索引 */
#define ADHOC_FRAME_IDX_SENDER    2u
/** @brief Sender 字段长度(字节) */
#define ADHOC_FRAME_SENDER_LEN    5u
/** @brief Content 字段起始索引 */
#define ADHOC_FRAME_IDX_CONTENT   7u
/** @brief Content 字段长度(字节) */
#define ADHOC_FRAME_CONTENT_LEN   24u
/** @brief CRC8 字节索引 */
#define ADHOC_FRAME_IDX_CRC       31u

/** @brief 域号最大值(11bit) */
#define ADHOC_SENDER_DOMAIN_MAX   0x07FFu
/** @brief 节点ID最大值(29bit) */
#define ADHOC_SENDER_NODE_MAX     0x1FFFFFFFu
/** @brief 载荷ID标志最大值(3bit) */
#define ADHOC_PAYLOAD_ID_FLAG_MAX 0x07u
/** @brief LMT_A 秒字段位宽 */
#define ADHOC_LMT_A_SEC_BITS      25u
/** @brief LMT_A 周期字段位宽 */
#define ADHOC_LMT_A_PERIOD_BITS   7u
/** @brief LMT_A 秒字段掩码 */
#define ADHOC_LMT_A_SEC_MASK      0x01FFFFFFu
/** @brief LMT_A 周期字段位移 */
#define ADHOC_LMT_A_PERIOD_SHIFT  25u
/** @brief LMT_A 周期字段掩码 */
#define ADHOC_LMT_A_PERIOD_MASK   0x7Fu
/** @brief ID标志: 绑定编号最小值 */
#define ADHOC_PAYLOAD_ID_FLAG_BIND_MIN 0u
/** @brief ID标志: 绑定编号最大值 */
#define ADHOC_PAYLOAD_ID_FLAG_BIND_MAX 5u
/** @brief ID标志: 已确认上级声明 */
#define ADHOC_PAYLOAD_ID_FLAG_UPSTREAM_CONFIRMED 6u
/** @brief ID标志: 入网确认(上级对下级) */
#define ADHOC_PAYLOAD_ID_FLAG_JOIN_CONFIRM 7u

/** @brief 消息类别: A帧(组网/管理) = 0, D帧(数据) = 1 */
typedef enum {
    ADHOC_MSG_CLASS_A = 0u,  /**< A类帧: 组网控制/管理 */
    ADHOC_MSG_CLASS_D = 1u   /**< D类帧: 业务数据 */
} adhoc_msg_class_t;

/**
 * @brief 发送者标识(域号 + 节点ID)
 */
typedef struct
{
    uint16_t domain_id;  /**< 组网域号(11bit) */
    uint32_t node_id;    /**< 节点ID(29bit) */
} adhoc_sender_t;

/**
 * @brief 载荷ID(标志位 + 节点ID)
 */
typedef struct
{
    uint8_t  id_flag;  /**< ID标志(3bit), 见 ADHOC_PAYLOAD_ID_FLAG_* */
    uint32_t node_id;  /**< 节点ID(29bit) */
} adhoc_payload_id_t;

/**
 * @brief 解析后的协议帧字段
 */
typedef struct
{
    uint8_t         msg_class;    /**< 消息类别(A=0/D=1) */
    uint8_t         gateway_no;   /**< 网关编号(0~7) */
    uint8_t         slot_high4;   /**< 时隙号高4位 */
    uint8_t         level;        /**< 节点级别 */
    adhoc_sender_t  sender;       /**< 发送者标识 */
    uint8_t         content[ADHOC_FRAME_CONTENT_LEN]; /**< 帧内容(24B) */
    uint8_t         crc8;         /**< CRC8校验值 */
} adhoc_frame_fields_t;

/* ========== Header 编解码 ========== */

/**
 * @brief 构造帧头字节
 * @param msg_class  消息类别(0或1)
 * @param gateway_no 网关编号(0~7)
 * @param slot_high4 时隙号高4位
 * @return 帧头字节
 */
uint8_t adhoc_frame_header_make(uint8_t msg_class, uint8_t gateway_no, uint8_t slot_high4);

/**
 * @brief 解析帧头字节
 * @param header_byte 帧头字节
 * @param msg_class   输出消息类别
 * @param gateway_no  输出网关编号
 * @param slot_high4  输出时隙号高4位
 */
void adhoc_frame_header_parse(uint8_t header_byte, uint8_t *msg_class, uint8_t *gateway_no, uint8_t *slot_high4);

/* ========== Sender / Payload ID 编解码 ========== */

/**
 * @brief 打包发送者ID(域号+节点ID)为5字节大端
 * @param sender 发送者标识
 * @param out    输出5字节缓冲区
 * @return 1成功 0失败
 */
int adhoc_sender_pack(adhoc_sender_t sender, uint8_t out[ADHOC_FRAME_SENDER_LEN]);

/**
 * @brief 从5字节大端解出发送者ID
 * @param in     5字节输入
 * @param sender 输出发送者标识
 * @return 1成功 0失败
 */
int adhoc_sender_unpack(const uint8_t in[ADHOC_FRAME_SENDER_LEN], adhoc_sender_t *sender);

/**
 * @brief 打包载荷ID为4字节大端
 * @param packed_id 载荷ID
 * @param out       输出4字节缓冲区
 * @return 1成功 0失败
 */
int adhoc_payload_id_pack(adhoc_payload_id_t packed_id, uint8_t out[4]);

/**
 * @brief 从4字节大端解出载荷ID
 * @param in        4字节输入
 * @param packed_id 输出载荷ID
 * @return 1成功 0失败(含node_id=0非法)
 */
int adhoc_payload_id_unpack(const uint8_t in[4], adhoc_payload_id_t *packed_id);

/* ========== 时间戳辅助 ========== */

/**
 * @brief 从微秒时间戳生成 A 帧时间戳 LMT_A(秒级)
 * @param now_us 当前微秒时间戳
 * @return LMT_A 值
 */
uint32_t adhoc_lmt_a_from_us(uint32_t now_us);

/**
 * @brief 从微秒时间戳生成 D 帧时间戳 LMT_D(24bit秒级)
 * @param now_us 当前微秒时间戳
 * @return LMT_D 值
 */
uint32_t adhoc_lmt_d_from_us(uint32_t now_us);

/**
 * @brief 设置 LMT_A 周期字段计算所使用的 T5(μs)
 * @param t5_us T5 周期(μs), 0 表示保持当前配置
 */
void adhoc_time_set_lmt_a_t5_us(uint32_t t5_us);

/**
 * @brief 打包 LMT_A 位域
 * @param bdt_second BDT 秒计数
 * @param period     周期字段(秒内微秒/T5)
 * @return 32bit LMT_A
 */
uint32_t adhoc_lmt_a_pack(uint32_t bdt_second, uint8_t period);

/**
 * @brief 提取 LMT_A 中的秒字段(25bit)
 * @param lmt_a LMT_A 原始值
 * @return 秒字段
 */
uint32_t adhoc_lmt_a_sec_get(uint32_t lmt_a);

/**
 * @brief 提取 LMT_A 中的周期字段(7bit)
 * @param lmt_a LMT_A 原始值
 * @return 周期字段
 */
uint8_t adhoc_lmt_a_period_get(uint32_t lmt_a);

/**
 * @brief 设置 BDT 时间基线映射
 * @param mono_us    当前单调微秒时基（用于协议调度的 now_us）
 * @param bdt_second 当前 BDT 秒计数（2006-01-01 00:00:00 起）
 */
void adhoc_time_set_bdt_base(uint32_t mono_us, uint32_t bdt_second);

/**
 * @brief 清除 BDT 基线映射（恢复为 now_us/1e6）
 */
void adhoc_time_clear_bdt_base(void);

/**
 * @brief 将单调微秒时间换算为 BDT 秒
 * @param mono_us 单调微秒时基
 * @return BDT 秒值
 */
uint32_t adhoc_time_bdt_seconds_from_mono_us(uint32_t mono_us);

/* ========== 大端整数读写 ========== */

/**
 * @brief 写入24bit大端无符号整数
 * @param value 值
 * @param out   3字节输出缓冲区
 */
void adhoc_u24_be_write(uint32_t value, uint8_t out[3]);

/**
 * @brief 读取24bit大端无符号整数
 * @param in 3字节输入
 * @return 整数值
 */
uint32_t adhoc_u24_be_read(const uint8_t in[3]);

/**
 * @brief 写入32bit大端无符号整数
 * @param value 值
 * @param out   4字节输出缓冲区
 */
void adhoc_u32_be_write(uint32_t value, uint8_t out[4]);

/**
 * @brief 读取32bit大端无符号整数
 * @param in 4字节输入
 * @return 整数值
 */
uint32_t adhoc_u32_be_read(const uint8_t in[4]);

/* ========== 帧整体构建与解析 ========== */

/**
 * @brief 从字段结构体构建完整32B帧(含CRC8)
 * @param fields   帧字段
 * @param out_frame 输出32字节帧缓冲区
 * @return 1成功 0失败
 */
int adhoc_frame_build(const adhoc_frame_fields_t *fields, uint8_t out_frame[ADHOC_FRAME_SIZE]);

/**
 * @brief 解析32B完整帧为字段结构体(含CRC8校验)
 * @param frame      32字节帧输入
 * @param out_fields 输出解析字段
 * @return 1成功 0失败(CRC校验失败或字段非法)
 */
int adhoc_frame_parse(const uint8_t frame[ADHOC_FRAME_SIZE], adhoc_frame_fields_t *out_fields);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_FRAME_H */
