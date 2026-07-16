## License information

The SPDX identifier in each source file is authoritative. This file is an
inventory for release packaging; it does not replace a file-level notice.

### Project source

The root-level `LICENSE` is the Apache-2.0 primary project license used for
repository discovery. It does not override a file-level SPDX identifier or a
bundled third-party notice.

- Most project source is licensed under [Apache-2.0](Apache-2.0).
- Linux kernel-module source is licensed under [GPL-2.0-only](GPL-2.0).
- Shared protocol and transport files marked `GPL-2.0-only OR Apache-2.0` may
  be used under either license, at the recipient's option.
- Example and test source marked `CC0-1.0` or `Unlicense OR CC0-1.0` is covered
  by [CC0-1.0](CC0-1.0) and, where selected by the SPDX expression, the
  [Unlicense](Unlicense).

### Included third-party material

- `common/serializers/third_party/msg_codec/protobuf-c/` carries its own
  BSD-3-Clause notice in `LICENSE`.
- Managed components and vendored dependencies retain their own license files.
  Preserve those notices when distributing the corresponding material.

### Release requirements

1. Preserve every file-level SPDX identifier and all bundled third-party
   license/notice files.
2. Include `Apache-2.0`, `GPL-2.0`, `CC0-1.0`, and `Unlicense` from this
   directory whenever the distributed source or binaries include material under
   those terms.
3. Include the protobuf-c BSD-3-Clause notice whenever that serializer is
   distributed.
4. Do not infer a directory-wide license from this summary; check the SPDX
   identifier of every changed or distributed file.
