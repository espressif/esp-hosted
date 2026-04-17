# SPDX-License-Identifier: Apache-2.0
# eh_proto_v2 — Unified V2 protobuf (placeholder)
# Schema pending PENDING-006. No SRCS until Phase F.


add_library(eh_proto_v2 STATIC
    "src/eh_proto_v2_stub.c"
)

target_include_directories(eh_proto_v2
    PUBLIC
        "include"
)
