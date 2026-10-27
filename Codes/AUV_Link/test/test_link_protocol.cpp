// Host-side tests for the link framing. Build and run from Codes/AUV_Link:
//   g++ -std=c++17 -Wall -Wextra -I. -o test_link test/test_link_protocol.cpp link_protocol.cpp && ./test_link
#include "link_protocol.h"

#include <cstdio>
#include <cstring>

using namespace link;

static int failures = 0;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                                \
        }                                                              \
    } while (0)

static bool bytes_equal(const uint8_t *a, size_t na, const uint8_t *b, size_t nb)
{
    return na == nb && std::memcmp(a, b, na) == 0;
}

// Feed bytes to a parser and return true if the last byte completed a valid frame.
static bool feed_all(Parser &p, const uint8_t *data, size_t n)
{
    bool last = false;
    for (size_t i = 0; i < n; i++) last = p.feed(data[i]);
    return last;
}

static void test_golden_encode()
{
    uint8_t buf[32];
    size_t n;

    uint8_t hb[] = {7};
    n = encode(MSG_HEARTBEAT, hb, 1, buf, sizeof buf);
    const uint8_t hb_expect[] = {0xAA, 0x55, 0x01, 0x01, 0x07, 0x6B};
    CHECK(bytes_equal(buf, n, hb_expect, sizeof hb_expect));

    uint8_t sp[] = {50, 0xF6, 0x00, 100};  // thrust 50, yaw -10, pitch 0, ballast 100
    n = encode(MSG_SETPOINT, sp, 4, buf, sizeof buf);
    const uint8_t sp_expect[] = {0xAA, 0x55, 0x04, 0x02, 0x32, 0xF6, 0x00, 0x64, 0xCF};
    CHECK(bytes_equal(buf, n, sp_expect, sizeof sp_expect));

    uint8_t ack[] = {7};
    n = encode(MSG_ACK, ack, 1, buf, sizeof buf);
    const uint8_t ack_expect[] = {0xAA, 0x55, 0x01, 0x03, 0x07, 0x41};
    CHECK(bytes_equal(buf, n, ack_expect, sizeof ack_expect));

    uint8_t tm[] = {1, 50, 0xF6, 0x00, 100, 0, 0, 0, 0};
    n = encode(MSG_TELEMETRY, tm, 9, buf, sizeof buf);
    const uint8_t tm_expect[] = {0xAA, 0x55, 0x09, 0x10, 0x01, 0x32, 0xF6, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x21};
    CHECK(bytes_equal(buf, n, tm_expect, sizeof tm_expect));

    uint8_t lk[] = {85, 1};  // wetness 85 %, alert on
    n = encode(MSG_LEAK, lk, 2, buf, sizeof buf);
    const uint8_t lk_expect[] = {0xAA, 0x55, 0x02, 0x11, 0x55, 0x01, 0xAF};
    CHECK(bytes_equal(buf, n, lk_expect, sizeof lk_expect));
}

static void test_roundtrip_and_fields()
{
    uint8_t frame[32];
    uint8_t sp[] = {50, 0xF6, 0x00, 100};
    size_t n = encode(MSG_SETPOINT, sp, 4, frame, sizeof frame);

    Parser p;
    CHECK(feed_all(p, frame, n));
    CHECK(p.type() == MSG_SETPOINT);
    CHECK(p.len() == 4);
    CHECK(bytes_equal(p.payload(), 4, sp, 4));
}

static void test_bad_crc_rejected()
{
    uint8_t frame[] = {0xAA, 0x55, 0x01, 0x01, 0x07, 0x6C};  // CRC should be 0x6B
    Parser p;
    CHECK(!feed_all(p, frame, sizeof frame));
}

static void test_garbage_before_frame()
{
    uint8_t stream[] = {0x00, 0xFF, 0xAA, 0xAA, 0x12, 0xAA, 0x55, 0x01, 0x01, 0x07, 0x6B};
    Parser p;
    bool got = false;
    for (uint8_t b : stream) {
        if (p.feed(b)) got = true;
    }
    CHECK(got);
    CHECK(p.type() == MSG_HEARTBEAT);
    CHECK(p.len() == 1);
    CHECK(p.payload()[0] == 7);
}

static void test_two_frames_back_to_back()
{
    uint8_t a[32], b[32];
    uint8_t hb[] = {1};
    uint8_t ack[] = {1};
    size_t na = encode(MSG_HEARTBEAT, hb, 1, a, sizeof a);
    size_t nb = encode(MSG_ACK, ack, 1, b, sizeof b);

    Parser p;
    int count = 0;
    for (size_t i = 0; i < na; i++) if (p.feed(a[i])) count++;
    for (size_t i = 0; i < nb; i++) if (p.feed(b[i])) count++;
    CHECK(count == 2);
}

static void test_zero_length_payload()
{
    uint8_t frame[8];
    size_t n = encode(0x42, nullptr, 0, frame, sizeof frame);
    CHECK(n == FRAME_OVERHEAD);
    Parser p;
    CHECK(feed_all(p, frame, n));
    CHECK(p.type() == 0x42);
    CHECK(p.len() == 0);
}

static void test_encode_limits()
{
    uint8_t big[MAX_PAYLOAD + 1] = {0};
    uint8_t out[64];
    CHECK(encode(MSG_TELEMETRY, big, MAX_PAYLOAD + 1, out, sizeof out) == 0);
    CHECK(encode(MSG_TELEMETRY, big, 4, out, 4) == 0);  // buffer too small
}

static void test_oversize_length_rejected()
{
    uint8_t stream[] = {0xAA, 0x55, 0x11};  // LEN 17 exceeds MAX_PAYLOAD
    Parser p;
    CHECK(!feed_all(p, stream, sizeof stream));
    // Parser should have reset, so a good frame afterwards still parses.
    uint8_t good[] = {0xAA, 0x55, 0x01, 0x01, 0x07, 0x6B};
    CHECK(feed_all(p, good, sizeof good));
}

int main()
{
    test_golden_encode();
    test_roundtrip_and_fields();
    test_bad_crc_rejected();
    test_garbage_before_frame();
    test_two_frames_back_to_back();
    test_zero_length_payload();
    test_encode_limits();
    test_oversize_length_rejected();

    if (failures == 0) {
        std::printf("All link protocol tests passed.\n");
        return 0;
    }
    std::printf("%d check(s) failed.\n", failures);
    return 1;
}
