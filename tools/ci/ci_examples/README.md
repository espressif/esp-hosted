# CI-only "super" build aggregates

These are **not** shipped examples — they live under `tools/ci/` on purpose, so
the example-structure gate (`tools/check_example_structure.py`, scoped to
`examples/`) ignores them.

Each aggregate enables *many* features in **one** build so a single compile
proves they all build + link together. This is **build/link coverage only** —
runtime behaviour is covered by the emu regression, not here.

| Aggregate  | Role | Covers |
|------------|------|--------|
| `cp_super` | coprocessor | every chip-independent CP feature (Wi-Fi, network-split, BT controller, peer-data, GPIO-expander, host-power-save, debug, CLI) in one binary; 802.15.4 OpenThread added on c6/h2; Wi-Fi dropped on h2 (no radio) |

Chip-specific deltas are in `sdkconfig.defaults.<chip>` (idf-build-apps applies
them automatically per `--target`). Transport is chosen at build time via
`--override-sdkconfig-items=CONFIG_EH_TRANSPORT_CP_<BUS>=y`.

`ext_coex` and `coprocessor_ota` are intentionally *excluded* — they need a
special partition/coex-wire setup and stay covered by their real examples.

Build one directly:

```bash
idf-build-apps build -p tools/ci/ci_examples/cp_super --target esp32c6 -vv
```
