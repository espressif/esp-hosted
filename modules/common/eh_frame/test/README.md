# esp_hosted_frame — unit tests

Host-compiled C test suite for the frame encode/decode component.
No IDF, no FreeRTOS, no kernel headers — plain `gcc`.

## Quick start

```sh
# From this directory
make run

# Or from repo root
make -C components/common/esp_hosted_frame/test run
```

## Coverage (92 tests)

| Group | What is tested |
|-------|----------------|
| `init_*` | NULL cfg, bad version, zero bufsize, bad transport, valid init |
| `hdr_*` | `hdr_size()` returns 12/20; `hdr_size_for_ver()` V1/V2/unknown |
| `v1_*` | Full V1 roundtrip, all fields (if_type, if_num, flags, pkt_type, seq_num, throttle_cmd, payload_len, payload_ptr, payload_data, frag_seq=0, tlv_offset=0); checksum-on reference match; checksum-off (no false corrupt); corrupt detection; PRIV_IF (if_type=5) not treated as dummy; exact buf_len; one-byte-short → INVALID |
| `v2_*` | Full V2 roundtrip, all fields (magic, hdr_version, pkt_num, offset=20, frag_seq_num, tlv_offset, throttle=0); checksum-on reference match; checksum-off corrupt ignored; corrupt detection; bad hdr_version → INVALID; TOOBIG; bad offset → INVALID; dummy (if_type=ESP_MAX_IF); len=0 → DUMMY |
| `auto_*` | V2 frame decoded correctly when cfg says V1 (auto-detect by magic byte); if_type and frag_seq preserved across auto-detect |
| `dummy_*` | V1 + V2 encode_dummy size/magic/decode; NULL buf → 0 |
| `toobig_*` | V1 + V2 TOOBIG (len+offset > max_buf_size) |
| `bad_offset_*` | V1 + V2 offset ≠ hdr_size → INVALID |
| `dec_*` / `enc_*` | NULL buf, NULL handle, zero buf_len, short buf (V1 < 12, V2 < 20) |
| `reinit_*` | V1 → V2 re-init: hdr_size(), encode version, decode all switch immediately |
| `v1_exact_ok` / `v1_one_short` | Exact buf_len == total → OK; total−1 → INVALID |
| `deinit_noop` | `esp_hosted_frame_deinit()` is a no-op, doesn't crash |

## Adding tests

Append a `static void test_*(void)` function to `frame_test.c` and add a call
in `main()`.  Use the `CHECK(name, condition)` macro — any failing assertion is
counted and printed with the condition text.
