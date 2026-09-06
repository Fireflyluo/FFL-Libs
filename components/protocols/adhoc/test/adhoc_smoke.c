#include "adhoc_crc8.h"
#include "adhoc_frame.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

int main(void)
{
    adhoc_frame_fields_t fields = {0};
    adhoc_frame_fields_t parsed = {0};
    uint8_t frame[ADHOC_FRAME_SIZE] = {0};
    uint8_t index;

    fields.msg_class = ADHOC_MSG_CLASS_D;
    fields.gateway_no = 3u;
    fields.slot_high4 = 9u;
    fields.level = 2u;
    fields.sender.domain_id = 0x345u;
    fields.sender.node_id = 0x1234567u;
    for (index = 0u; index < ADHOC_FRAME_CONTENT_LEN; ++index)
    {
        fields.content[index] = (uint8_t)(index ^ 0x5Au);
    }

    assert(adhoc_frame_build(&fields, frame) == 1);
    assert(adhoc_crc8_verify_frame(frame) == 1);
    assert(adhoc_frame_parse(frame, &parsed) == 1);
    assert(parsed.msg_class == fields.msg_class);
    assert(parsed.gateway_no == fields.gateway_no);
    assert(parsed.slot_high4 == fields.slot_high4);
    assert(parsed.level == fields.level);
    assert(parsed.sender.domain_id == fields.sender.domain_id);
    assert(parsed.sender.node_id == fields.sender.node_id);
    assert(memcmp(parsed.content, fields.content, ADHOC_FRAME_CONTENT_LEN) == 0);

    frame[ADHOC_FRAME_IDX_CONTENT] ^= 0x01u;
    assert(adhoc_crc8_verify_frame(frame) == 0);

    return 0;
}
