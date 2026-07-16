# Pending Things

[Home](../README.md) · [Migration Guide](migration.md) · [Testing](testing.md)

This is the release hand-off register. It identifies exact commits and documentation work that must be brought into a destination checkout. Keep it current; an item is not complete merely because its code has been copied.

## How to use this register

For each entry, preserve the commit ID, apply it to the destination checkout, and attach build and test evidence. Before updating a release branch, generate the authoritative range from the source checkout:

```sh
git log --oneline <destination-base>..HEAD
git diff --check <destination-base>..HEAD
```

Replace `<destination-base>` with the commit that the destination currently contains. Do not replace the listed commit IDs with branch names: branch names can move, while commit IDs make the hand-off auditable.

## Commits pending downstream integration

| Commit | Change | Required downstream action | Validation required |
| --- | --- | --- | --- |
| `6d16747a` | Make network-split iperf connection console-driven and add the test matrix | Carry the application change and the full matching test matrix together. | Station connection plus the complete network-split iperf matrix. |
| `105c09f2` | Remove duplicate transport sources | Carry the deletion together with any build-file changes; check no downstream patch names the removed sources. | Clean configure and build. |
| `5726eb46` | Make transport init/deinit symmetric for host power-save wake | Integrate the lifecycle fix as one unit. | Power-save sleep/wake and repeated init/deinit. |
| `b3b08035` | Restore SDIO software-aggregation diagnostics and test markers | Preserve the diagnostics and matching tests. | SDIO aggregation test run and log review. |
| `29ce3fb4` | Preflight that firmware tests use a correctly patched SDK | Include the test preflight with the test infrastructure. | Run the firmware-test preflight. |
| `6d9240c1` | Bound SDIO receive and reset/drain state during deinitialization | Integrate after the symmetric lifecycle change. | Host power-save wake, disconnect, and repeated reset/deinit. |

The destination must take these in ancestry order. If a commit is already present, record the equivalent destination commit rather than applying it again.

## Documentation work still pending

| Area | Exact pending work | Completion evidence |
| --- | --- | --- |
| MCU-focused predecessor documentation | Review board setup, lab/shield-box procedures, compatibility-test instructions, and all release notes against the current examples and targets. Port only material that remains accurate. | Source revision, destination page, technical reviewer, and a successful reproduced procedure. |
| Combined predecessor documentation | Review FG and NG setup, porting, protocol, Bluetooth, sleep, raw-throughput, and troubleshooting pages. Port FG material after code-link verification; treat NG-only material as pending feature/documentation work. | Per-page disposition: migrated, superseded, not applicable, or pending. |
| Release record | Attach the exact source range, destination commit IDs, build configurations, test reports, and known limitations. | Signed release checklist or equivalent approval record. |

## Limitations that must remain visible

- **ESP-Hosted-FG is migrated fully to this codebase.**
- **ESP-Hosted-NG is not migrated fully to this codebase.**

## Release exit criteria
