#include "adhoc_api.h"
#include "adhoc_config.h"
#include "adhoc_crc8.h"
#include "adhoc_data_plane.h"
#include "adhoc_frame.h"
#include "adhoc_link.h"
#include "adhoc_reply_list.h"
#include "adhoc_sm.h"
#include "adhoc_timing.h"

#include <cassert>
#include <cstdint>
#include <cstring>

int main()
{
    adhoc_frame_fields_t fields{};
    adhoc_frame_fields_t parsed{};
    std::uint8_t frame[ADHOC_FRAME_SIZE]{};

    fields.msg_class = ADHOC_MSG_CLASS_A;
    fields.gateway_no = 5u;
    fields.slot_high4 = 0x0Fu;
    fields.level = 1u;
    fields.sender = {0x2AAu, 0x1ABCDEu};
    fields.content[0] = 0xC3u;
    fields.content[ADHOC_FRAME_CONTENT_LEN - 1u] = 0x3Cu;

    assert(adhoc_frame_build(&fields, frame) == 1);
    assert(adhoc_crc8_verify_frame(frame) == 1);
    assert(adhoc_frame_parse(frame, &parsed) == 1);
    assert(parsed.msg_class == fields.msg_class);
    assert(parsed.gateway_no == fields.gateway_no);
    assert(parsed.slot_high4 == fields.slot_high4);
    assert(parsed.level == fields.level);
    assert(parsed.sender.domain_id == fields.sender.domain_id);
    assert(parsed.sender.node_id == fields.sender.node_id);
    assert(std::memcmp(parsed.content, fields.content, ADHOC_FRAME_CONTENT_LEN) == 0);

    return 0;
}
