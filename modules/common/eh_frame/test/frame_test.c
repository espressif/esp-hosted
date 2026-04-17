/* frame_test.c — full V1 + V2 test for eh_frame
 * Compile: gcc -std=c11 -Wall -Wextra -Wshadow -Wpedantic
 *            -I .../eh_common/include -I .../eh_frame/include
 *            eh_frame.c frame_test.c -o frame_test
 */
#include "eh_frame.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#ifndef ESP_MAX_IF
#define ESP_MAX_IF 6
#endif

/* ── Test bookkeeping ───────────────────────────────────────────────────── */
static int g_pass = 0, g_fail = 0;
#define PASS(name) do { printf("  PASS  %s\n", (name)); g_pass++; } while(0)
#define FAIL(name, msg) do { printf("  FAIL  %s  (%s)\n", (name), (msg)); g_fail++; } while(0)
#define CHECK(name, cond) do { if (cond) PASS(name); else FAIL(name, #cond); } while(0)

/* ── Replicate exact compute_checksum from eh_transport.h ────────── */
static uint16_t ref_checksum(const uint8_t *buf, uint16_t len) {
    uint16_t s = 0, i;
    for (i = 0; i < len; i++) s += buf[i];
    return s;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Helper: make cfg
 * ══════════════════════════════════════════════════════════════════════════ */
static eh_frame_cfg_t make_cfg(uint8_t ver, uint8_t cksum)
{
    eh_frame_cfg_t c = (eh_frame_cfg_t)
        EH_FRAME_CFG_CP_FG_LINUX_SPI_DEFAULT;
    c.hdr_version     = ver;
    c.checksum_enabled = cksum;
    return c;
}

/* ══════════════════════════════════════════════════════════════════════════
 * init / validation
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_init_validation(void)
{
    eh_frame_cfg_t c;

    CHECK("init_null",          eh_frame_init(NULL) == ESP_ERR_INVALID_ARG);

    c = make_cfg(0xFF, 1);
    CHECK("init_bad_version",   eh_frame_init(&c) == ESP_ERR_INVALID_ARG);

    c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 1);
    c.max_buf_size = 0;
    CHECK("init_zero_bufsize",  eh_frame_init(&c) == ESP_ERR_INVALID_ARG);

    c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 1);
    c.transport = (eh_transport_t)99;
    CHECK("init_bad_transport", eh_frame_init(&c) == ESP_ERR_INVALID_ARG);

    c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 1);
    CHECK("init_valid",         eh_frame_init(&c) == ESP_OK);
}

static void test_hdr_sizes(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 1);
    eh_frame_init(&c);
    CHECK("hdr_v1_12",          eh_frame_hdr_size() == 12);

    c.hdr_version = ESP_HOSTED_HDR_VERSION_V2;
    eh_frame_init(&c);
    CHECK("hdr_v2_20",          eh_frame_hdr_size() == 20);

    CHECK("for_ver_v1",         eh_frame_hdr_size_for_ver(ESP_HOSTED_HDR_VERSION_V1) == 12);
    CHECK("for_ver_v2",         eh_frame_hdr_size_for_ver(ESP_HOSTED_HDR_VERSION_V2) == 20);
    CHECK("for_ver_unknown",    eh_frame_hdr_size_for_ver(0xFF) == 0);
}

/* ══════════════════════════════════════════════════════════════════════════
 * V1 encode / decode — checksum ON
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_v1_roundtrip_cksum_on(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 1);
    eh_frame_init(&c);

    uint8_t buf[256] = {0};
    const char *pl   = "hello v1 world";
    uint16_t    plen = (uint16_t)strlen(pl);
    memcpy(buf + 12, pl, plen);

    interface_buffer_handle_t tx = {
        .if_type=3, .if_num=0, .flags=0x06, .pkt_type=0x07,
        .seq_num=100, .throttle_cmd=2,
    };
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    CHECK("v1_enc_sz_12",       wr == 12);
    CHECK("v1_not_magic",       buf[0] != 0xE9);
    CHECK("v1_detect_v1",       eh_frame_detect_version(buf, sizeof(buf)) == ESP_HOSTED_HDR_VERSION_V1);

    /* Verify checksum byte-for-byte against reference */
    uint16_t stored_le = (uint16_t)(buf[6] | ((uint16_t)buf[7] << 8));
    uint8_t tmp6 = buf[6], tmp7 = buf[7];
    buf[6] = 0; buf[7] = 0;
    uint16_t ref = ref_checksum(buf, (uint16_t)(12 + plen));
    buf[6] = tmp6; buf[7] = tmp7;
    /* ref is host-order; stored_le is LE — on LE host they're equal */
    CHECK("v1_cksum_matches_ref", stored_le == ref);

    interface_buffer_handle_t rx = {0};
    eh_frame_result_t r = eh_frame_decode(buf, (uint16_t)(wr+plen), &rx);
    CHECK("v1_dec_ok",          r == EH_FRAME_OK);
    CHECK("v1_if_type",         rx.if_type    == 3);
    CHECK("v1_if_num",          rx.if_num     == 0);
    CHECK("v1_flags",           rx.flags      == 0x06);
    CHECK("v1_pkt_type",        rx.pkt_type   == 0x07);
    CHECK("v1_seq_num",         rx.seq_num    == 100);
    CHECK("v1_throttle",        rx.throttle_cmd == 2);
    CHECK("v1_payload_len",     rx.payload_len == plen);
    CHECK("v1_payload_ptr",     rx.payload == buf + 12);
    CHECK("v1_payload_data",    memcmp(rx.payload, pl, plen) == 0);
    CHECK("v1_frag_seq_0",      rx.frag_seq   == 0);
    CHECK("v1_tlv_offset_0",    rx.tlv_offset == 0);
    CHECK("v1_priv_null",       rx.priv_buffer_handle == NULL);
}

static void test_v1_cksum_off_corrupt_ignored(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 0);
    eh_frame_init(&c);

    uint8_t buf[256] = {0};
    const char *pl = "no checksum";
    uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf + 12, pl, plen);

    interface_buffer_handle_t tx = {.if_type=1, .seq_num=5};
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    /* checksum field zero (disabled) */
    uint16_t cs = (uint16_t)(buf[6] | ((uint16_t)buf[7]<<8));
    CHECK("v1_cksum_field_zero", cs == 0);

    /* corrupt checksum bytes — must be ignored */
    buf[6] = 0xDE; buf[7] = 0xAD;
    interface_buffer_handle_t rx = {0};
    eh_frame_result_t r = eh_frame_decode(buf, (uint16_t)(wr+plen), &rx);
    CHECK("v1_cksum_off_ok",    r == EH_FRAME_OK);
    CHECK("v1_cksum_off_data",  memcmp(rx.payload, pl, plen) == 0);
}

static void test_v1_corrupt_detected(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 1);
    eh_frame_init(&c);

    uint8_t buf[128] = {0};
    const char *pl = "testdata";
    uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+12, pl, plen);
    interface_buffer_handle_t tx = {.if_type=1};
    uint8_t wr = eh_frame_encode(buf, &tx, plen);

    buf[6] ^= 0xFF;  /* flip one checksum byte */
    interface_buffer_handle_t rx = {0};
    CHECK("v1_corrupt",
          eh_frame_decode(buf, (uint16_t)(wr+plen), &rx) == EH_FRAME_CORRUPT);
}

/* ══════════════════════════════════════════════════════════════════════════
 * V2 encode / decode
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_v2_roundtrip_cksum_on(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 1);
    eh_frame_init(&c);

    uint8_t buf[256] = {0};
    const char *pl   = "v2 frame payload data";
    uint16_t    plen = (uint16_t)strlen(pl);
    memcpy(buf + 20, pl, plen);

    interface_buffer_handle_t tx = {
        .if_type=2, .if_num=0, .flags=0x04, .pkt_type=0x09,
        .seq_num=500, .frag_seq=3, .tlv_offset=0,
    };
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    CHECK("v2_enc_sz_20",    wr == 20);
    CHECK("v2_magic",        buf[0] == 0xE9);
    CHECK("v2_hdr_ver",      buf[1] == 0x02);
    CHECK("v2_detect_v2",    eh_frame_detect_version(buf, sizeof(buf)) == ESP_HOSTED_HDR_VERSION_V2);

    uint16_t pkt_num = (uint16_t)(buf[2] | ((uint16_t)buf[3]<<8));
    CHECK("v2_pkt_num",      pkt_num == 500);
    uint16_t off = (uint16_t)(buf[8] | ((uint16_t)buf[9]<<8));
    CHECK("v2_offset_20",    off == 20);
    uint16_t lf  = (uint16_t)(buf[10]| ((uint16_t)buf[11]<<8));
    CHECK("v2_len_field",    lf == plen);
    CHECK("v2_frag_byte7",   buf[7] == 3);

    /* Verify checksum against reference */
    uint16_t stored_le = (uint16_t)(buf[12]|((uint16_t)buf[13]<<8));
    uint8_t t12=buf[12], t13=buf[13]; buf[12]=0; buf[13]=0;
    uint16_t ref = ref_checksum(buf, (uint16_t)(20 + plen));
    buf[12]=t12; buf[13]=t13;
    CHECK("v2_cksum_matches_ref", stored_le == ref);

    interface_buffer_handle_t rx = {0};
    eh_frame_result_t r = eh_frame_decode(buf, (uint16_t)(wr+plen), &rx);
    CHECK("v2_dec_ok",       r == EH_FRAME_OK);
    CHECK("v2_if_type",      rx.if_type    == 2);
    CHECK("v2_flags",        rx.flags      == 0x04);
    CHECK("v2_pkt_type",     rx.pkt_type   == 0x09);
    CHECK("v2_seq_num",      rx.seq_num    == 500);
    CHECK("v2_frag_seq",     rx.frag_seq   == 3);
    CHECK("v2_tlv_offset",   rx.tlv_offset == 0);
    CHECK("v2_payload_len",  rx.payload_len == plen);
    CHECK("v2_payload_ptr",  rx.payload == buf + 20);
    CHECK("v2_payload_data", memcmp(rx.payload, pl, plen) == 0);
    CHECK("v2_throttle_0",   rx.throttle_cmd == 0);
}

static void test_v2_corrupt_detected(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 1);
    eh_frame_init(&c);
    uint8_t buf[128] = {0};
    const char *pl = "v2test"; uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+20, pl, plen);
    interface_buffer_handle_t tx = {.if_type=1, .seq_num=1};
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    buf[12] ^= 0xFF;
    interface_buffer_handle_t rx = {0};
    CHECK("v2_corrupt",
          eh_frame_decode(buf, (uint16_t)(wr+plen), &rx) == EH_FRAME_CORRUPT);
}

static void test_v2_bad_hdr_version(void)
{
    /* buf[0]=0xE9 but buf[1] is wrong — must return INVALID */
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 0);
    eh_frame_init(&c);
    uint8_t buf[64] = {0};
    const char *pl = "data"; uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+20, pl, plen);
    interface_buffer_handle_t tx = {.if_type=1, .seq_num=1};
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    /* corrupt the hdr_version byte */
    buf[1] = 0x99;
    interface_buffer_handle_t rx = {0};
    CHECK("v2_bad_hdrver",
          eh_frame_decode(buf, (uint16_t)(wr+plen), &rx) == EH_FRAME_INVALID);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Auto-detect: decode always uses magic byte regardless of cfg.hdr_version
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_auto_version_detect(void)
{
    /* Encode V2 frame */
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 0);
    eh_frame_init(&c);
    uint8_t buf[128] = {0};
    const char *pl = "autodetect"; uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+20, pl, plen);
    interface_buffer_handle_t tx = {.if_type=4, .seq_num=7, .frag_seq=1};
    uint8_t wr = eh_frame_encode(buf, &tx, plen);

    /* Switch cfg to V1 — decode must still pick V2 from buf[0] */
    c.hdr_version = ESP_HOSTED_HDR_VERSION_V1;
    eh_frame_init(&c);

    interface_buffer_handle_t rx = {0};
    eh_frame_result_t r = eh_frame_decode(buf, (uint16_t)(wr+plen), &rx);
    CHECK("auto_v2_in_v1_cfg", r == EH_FRAME_OK);
    CHECK("auto_if_type",      rx.if_type == 4);
    CHECK("auto_seq",          rx.seq_num == 7);
    CHECK("auto_frag",         rx.frag_seq == 1);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Dummy frames
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_dummy_frames(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 0);
    eh_frame_init(&c);
    uint8_t buf[64] = {0};

    uint8_t wr = eh_frame_encode_dummy(buf, 1);
    CHECK("dummy_v1_sz",   wr == 12);
    CHECK("dummy_v1_nomag",buf[0] != 0xE9);
    interface_buffer_handle_t rx = {0};
    CHECK("dummy_v1_dec",
          eh_frame_decode(buf, sizeof(buf), &rx) == EH_FRAME_DUMMY);

    c.hdr_version = ESP_HOSTED_HDR_VERSION_V2;
    eh_frame_init(&c);
    memset(buf, 0, sizeof(buf));
    wr = eh_frame_encode_dummy(buf, 0);
    CHECK("dummy_v2_sz",   wr == 20);
    CHECK("dummy_v2_mag",  buf[0] == 0xE9);
    CHECK("dummy_v2_dec",
          eh_frame_decode(buf, sizeof(buf), &rx) == EH_FRAME_DUMMY);

    CHECK("dummy_null",    eh_frame_encode_dummy(NULL, 0) == 0);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Edge cases
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_toobig(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 0);
    c.max_buf_size = 20;
    eh_frame_init(&c);

    uint8_t buf[256] = {0};
    const char *pl = "way longer than twenty bytes total";
    uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+12, pl, plen);
    interface_buffer_handle_t tx = {.if_type=1};

    /* encode with large limit so the frame is valid structurally */
    eh_frame_cfg_t big = c; big.max_buf_size = 4096;
    eh_frame_init(&big);
    uint8_t wr = eh_frame_encode(buf, &tx, plen);

    eh_frame_init(&c);  /* tiny limit back */
    interface_buffer_handle_t rx = {0};
    CHECK("toobig_v1",
          eh_frame_decode(buf, (uint16_t)(wr+plen), &rx) == EH_FRAME_TOOBIG);
}

static void test_bad_offset(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 0);
    eh_frame_init(&c);
    uint8_t buf[64] = {0};
    buf[0] = 1;           /* if_type=1, if_num=0 */
    buf[2] = 10;          /* len = 10 */
    buf[4] = 6;           /* offset = 6, must be 12 for V1 */
    memset(buf+12, 0xAB, 10);
    interface_buffer_handle_t rx = {0};
    CHECK("bad_offset_v1",
          eh_frame_decode(buf, 64, &rx) == EH_FRAME_INVALID);
}

static void test_null_inputs(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 1);
    eh_frame_init(&c);
    interface_buffer_handle_t rx = {0};
    uint8_t buf[64] = {0};

    CHECK("dec_null_buf",  eh_frame_decode(NULL, 64, &rx)  == EH_FRAME_INVALID);
    CHECK("dec_null_h",    eh_frame_decode(buf,  64, NULL) == EH_FRAME_INVALID);
    CHECK("dec_zero_len",  eh_frame_decode(buf,   0, &rx)  == EH_FRAME_INVALID);
    CHECK("enc_null_buf",  eh_frame_encode(NULL, &rx, 4)   == 0);
    CHECK("enc_null_h",    eh_frame_encode(buf, NULL, 4)   == 0);
    CHECK("dec_short_v1",  eh_frame_decode(buf,  5,  &rx)  == EH_FRAME_INVALID);
    buf[0] = 0xE9;
    CHECK("dec_short_v2",  eh_frame_decode(buf, 10,  &rx)  == EH_FRAME_INVALID);
}

static void test_deinit(void) { eh_frame_deinit(); CHECK("deinit_noop", 1); }

/* ══════════════════════════════════════════════════════════════════════════
 * Review-pass-2 additions
 * ══════════════════════════════════════════════════════════════════════════ */

/* V2 dummy: if_type=ESP_MAX_IF, hdr_version=0x02, len=0 → DUMMY */
static void test_v2_dummy_manual(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 1);
    eh_frame_init(&c);
    uint8_t buf[64] = {0};
    buf[0] = 0xE9; buf[1] = 0x02;
    buf[4] = 6 & 0x3F;   /* if_type=ESP_MAX_IF=6, if_num=0 */
    buf[8] = 20;          /* offset=20 LE */
    buf[10] = 0;          /* len=0 */
    interface_buffer_handle_t rx = {0};
    CHECK("v2_dummy_manual",
          eh_frame_decode(buf, sizeof(buf), &rx) == EH_FRAME_DUMMY);
}

/* V2 len=0 but if_type is a valid (non-max) interface → still DUMMY */
static void test_v2_len_zero_dummy(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 0);
    eh_frame_init(&c);
    uint8_t buf[64] = {0};
    buf[0] = 0xE9; buf[1] = 0x02;
    buf[4] = 1;    /* if_type=1 (not ESP_MAX_IF) */
    buf[8] = 20;   /* offset=20 */
    /* len stays 0 */
    interface_buffer_handle_t rx = {0};
    CHECK("v2_len0_dummy",
          eh_frame_decode(buf, sizeof(buf), &rx) == EH_FRAME_DUMMY);
}

/* V2 TOOBIG: offset+len > max_buf_size */
static void test_v2_toobig(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 0);
    c.max_buf_size = 25;
    eh_frame_init(&c);
    uint8_t buf[256] = {0};
    const char *pl = "123456789012";  /* 12 bytes: 20+12=32 > 25 */
    uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+20, pl, plen);
    interface_buffer_handle_t tx = {.if_type=1, .seq_num=1};
    eh_frame_cfg_t big = c; big.max_buf_size = 4096;
    eh_frame_init(&big);
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    eh_frame_init(&c);  /* restore tiny limit */
    interface_buffer_handle_t rx = {0};
    CHECK("v2_toobig",
          eh_frame_decode(buf, (uint16_t)(wr+plen), &rx) == EH_FRAME_TOOBIG);
}

/* V2 bad offset (not 20) → INVALID */
static void test_v2_bad_offset(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 0);
    eh_frame_init(&c);
    uint8_t buf[64] = {0};
    buf[0] = 0xE9; buf[1] = 0x02;
    buf[4] = 1;    /* if_type=1 */
    buf[8] = 10;   /* offset=10, must be 20 */
    buf[10] = 5;   /* len=5 */
    interface_buffer_handle_t rx = {0};
    CHECK("v2_bad_offset",
          eh_frame_decode(buf, sizeof(buf), &rx) == EH_FRAME_INVALID);
}

/* V1 if_type=5 (ESP_PRIV_IF) encodes and decodes cleanly (not dummy) */
static void test_v1_priv_if(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 0);
    eh_frame_init(&c);
    uint8_t buf[128] = {0};
    const char *pl = "priv data";
    uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+12, pl, plen);
    interface_buffer_handle_t tx = {.if_type=5, .if_num=0};
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    interface_buffer_handle_t rx = {0};
    eh_frame_result_t r = eh_frame_decode(buf, (uint16_t)(wr+plen), &rx);
    CHECK("v1_priv_if_ok",   r == EH_FRAME_OK);
    CHECK("v1_priv_if_type", rx.if_type == 5);
}

/* V1 if_type=ESP_MAX_IF (6) with valid len → still DUMMY */
static void test_v1_max_if_dummy(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 0);
    eh_frame_init(&c);
    uint8_t buf[64] = {0};
    /* byte0: if_type=6 in bits[3:0], if_num=0xF in bits[7:4] */
    buf[0] = (uint8_t)((0xF << 4) | 6);
    buf[2] = 10; buf[3] = 0;  /* len=10 */
    buf[4] = 12; buf[5] = 0;  /* offset=12 */
    memset(buf+12, 0xAA, 10);
    interface_buffer_handle_t rx = {0};
    CHECK("v1_max_if_dummy",
          eh_frame_decode(buf, sizeof(buf), &rx) == EH_FRAME_DUMMY);
}

/* V2 checksum disabled: corrupt bytes pass through */
static void test_v2_cksum_off_corrupt_ignored(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V2, 0);
    eh_frame_init(&c);
    uint8_t buf[256] = {0};
    const char *pl = "v2 no checksum";
    uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+20, pl, plen);
    interface_buffer_handle_t tx = {.if_type=1, .seq_num=3};
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    uint16_t cs = (uint16_t)(buf[12] | ((uint16_t)buf[13]<<8));
    CHECK("v2_cksum_off_zero", cs == 0);
    buf[12] = 0xDE; buf[13] = 0xAD;  /* corrupt checksum bytes */
    interface_buffer_handle_t rx = {0};
    CHECK("v2_cksum_off_ok",
          eh_frame_decode(buf, (uint16_t)(wr+plen), &rx) == EH_FRAME_OK);
}

/* re-init V1→V2: hdr_size and encode version both switch */
static void test_reinit_version_switch(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 0);
    eh_frame_init(&c);
    CHECK("reinit_v1_hdr", eh_frame_hdr_size() == 12);

    c.hdr_version = ESP_HOSTED_HDR_VERSION_V2;
    eh_frame_init(&c);
    CHECK("reinit_v2_hdr", eh_frame_hdr_size() == 20);

    uint8_t buf[64] = {0};
    interface_buffer_handle_t tx = {.if_type=1, .seq_num=42};
    /* encode with plen=4 so len!=0 and not dummy */
    memset(buf+20, 0x55, 4);
    uint8_t wr = eh_frame_encode(buf, &tx, 4);
    CHECK("reinit_v2_enc",    wr == 20);
    CHECK("reinit_v2_magic",  buf[0] == 0xE9);
    CHECK("reinit_v2_hdrver", buf[1] == 0x02);

    interface_buffer_handle_t rx = {0};
    CHECK("reinit_v2_dec",
          eh_frame_decode(buf, (uint16_t)(wr+4), &rx) == EH_FRAME_OK);
    CHECK("reinit_v2_seq", rx.seq_num == 42);
}

/* V1 decode: exact buf_len == total is OK; one byte short → INVALID */
static void test_v1_exact_buf_len(void)
{
    eh_frame_cfg_t c = make_cfg(ESP_HOSTED_HDR_VERSION_V1, 1);
    eh_frame_init(&c);
    uint8_t buf[128] = {0};
    const char *pl = "exact";
    uint16_t plen = (uint16_t)strlen(pl);
    memcpy(buf+12, pl, plen);
    interface_buffer_handle_t tx = {.if_type=1};
    uint8_t wr = eh_frame_encode(buf, &tx, plen);
    uint16_t exact = (uint16_t)(wr + plen);
    interface_buffer_handle_t rx = {0};
    CHECK("v1_exact_ok",    eh_frame_decode(buf, exact, &rx)           == EH_FRAME_OK);
    CHECK("v1_one_short",   eh_frame_decode(buf, (uint16_t)(exact-1), &rx) == EH_FRAME_INVALID);
}

int main(void)
{
    printf("eh_frame — V1 + V2 full test suite\n\n");
    test_init_validation();
    test_hdr_sizes();
    test_v1_roundtrip_cksum_on();
    test_v1_cksum_off_corrupt_ignored();
    test_v1_corrupt_detected();
    test_v2_roundtrip_cksum_on();
    test_v2_corrupt_detected();
    test_v2_bad_hdr_version();
    test_auto_version_detect();
    test_dummy_frames();
    test_toobig();
    test_bad_offset();
    test_null_inputs();
    test_deinit();
    /* review-pass-2 */
    test_v2_dummy_manual();
    test_v2_len_zero_dummy();
    test_v2_toobig();
    test_v2_bad_offset();
    test_v1_priv_if();
    test_v1_max_if_dummy();
    test_v2_cksum_off_corrupt_ignored();
    test_reinit_version_switch();
    test_v1_exact_buf_len();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
