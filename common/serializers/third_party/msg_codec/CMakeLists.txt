# SPDX-License-Identifier: Apache-2.0
#
# msg_codec — Vendored protobuf-c codec library.
#
# DESIGN RULE: This is the ONLY component that owns protobuf-c.
# All other components must depend on msg_codec and include "msg_codec.h".
# Direct use of <protobuf-c/protobuf-c.h> by other components is forbidden.
#
# The include layout exposed to consumers is:
#   "msg_codec.h"               → via include/
#   <protobuf-c/protobuf-c.h>  → via protobuf-c/ (required by protoc-generated pb-c.h files)


add_library(msg_codec STATIC
    "protobuf-c/protobuf-c/protobuf-c.c"
)

target_include_directories(msg_codec
    PUBLIC
        "include"       # exposes msg_codec.h (the abstraction entry point)
        "protobuf-c"    # exposes <protobuf-c/protobuf-c.h> (needed by generated pb-c.h files)
)
