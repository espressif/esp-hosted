# msg_codec — Design Notes

## Role

`msg_codec` is the **single owner** of the vendored `protobuf-c` library in
this codebase.  It provides the message serialisation / deserialisation
runtime used by all RPC and event layers.

## Abstraction Rule (mandatory)

> **No component other than `msg_codec` itself may directly include
> `<protobuf-c/protobuf-c.h>`.**

All consumers must depend on `msg_codec` and include the public umbrella
header:

```c
#include "msg_codec.h"
```

This applies to hand-written source files.  Auto-generated `*.pb-c.h` files
produced by `protoc-gen-c` are an exception at the *source text* level (the
generator always emits `#include <protobuf-c/protobuf-c.h>`), but those files
must be patched post-generation (or the generator wrapped) to use
`#include "msg_codec.h"` instead — see the two existing patched files:

- `esp_hosted_proto_mcu_v1/include/esp_hosted_rpc.pb-c.h`
- `esp_hosted_proto_linux_v1/include/esp_hosted_config.pb-c.h`

## Why

- **Single upgrade point**: bumping the protobuf-c version only requires
  touching `msg_codec`.
- **Portability**: alternative codec backends (e.g. nanopb) can be swapped
  in by changing `msg_codec` alone.
- **Build correctness**: IDF's component graph only propagates include paths
  one level by default; owning the path in one place prevents include leakage.

## Include path layout (what msg_codec exports)

| Path on disk (relative to msg_codec/) | Exposed as |
|---|---|
| `include/msg_codec.h` | `"msg_codec.h"` |
| `protobuf-c/` | `<protobuf-c/protobuf-c.h>` (for generated pb-c files) |

## CMake dependency

Any component that needs protobuf-c types must list `msg_codec` in its
`REQUIRES` or `PRIV_REQUIRES`:

```cmake
idf_component_register(
    ...
    REQUIRES msg_codec
)
```

Do **not** add `protobuf-c` paths to any other component's `INCLUDE_DIRS` or
`target_include_directories`.
