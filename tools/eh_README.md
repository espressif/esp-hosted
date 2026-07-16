# `tools/` — repo-level developer utilities


Brings the IDF-style Kconfig flow to plain Linux user-space builds.
The CLI surface mirrors `idf.py`'s Kconfig-side subcommands so a
developer moving between IDF and Linux builds sees the same
workflow.  Generates the same `sdkconfig.h` artifact `idf.py` does
on IDF, but driven by the existing `Kconfig` + `Kconfig.ext` tree
without needing the IDF environment.

**Backed by**: a vendored copy of `esp-idf-kconfig` (the
Espressif fork of [Kconfiglib](https://github.com/ulfalizer/Kconfiglib))
under `tools/vendor/`.  No runtime dependency on a system
kconfiglib install or on IDF being sourced — works on a fresh
Linux box with stock Python 3.  See `tools/vendor/README.md` for
the refresh procedure.

### Quick start

```bash
# One-time: pick the host port-type (analogous to `idf.py set-target esp32p4`).
./tools/eh.py set-target linux_user

# Optional: tweak interactively.  Same TUI idf.py menuconfig uses.
./tools/eh.py menuconfig

# After editing defaults or sdkconfig, regenerate headers:
./tools/eh.py reconfigure

# Build the linux examples — they auto-pick up build/config/sdkconfig.h:
bash examples/build_all_linux_host.sh
```

### CLI reference (mirrors `idf.py`)

| Command | Purpose | idf.py analogue |
|---|---|---|
| `eh.py set-target <port>` | Persist the port-type choice; apply its defaults; regenerate headers. | `idf.py set-target esp32p4` |
| `eh.py defconfig` | Re-apply the persisted target's defaults to `sdkconfig`. | `idf.py defconfig` |
| `eh.py menuconfig` | Launch the kconfiglib curses TUI. | `idf.py menuconfig` |
| `eh.py save-defconfig` | Write a minimal defconfig (only the diff from defaults). | `idf.py save-defconfig` |
| `eh.py reconfigure` | Regenerate `build/config/sdkconfig.h` + `.cmake` from `sdkconfig`. | `idf.py reconfigure` |
| `eh.py show` | Diagnostic — print current state. | (no exact analogue) |

Valid `<port>` values today: `linux_user`.  `stm32` / `zephyr`
arrive when those ports land — each adds one
`tools/sdkconfig.defaults.<port>` file.

### Output paths (mirror `idf.py`)

```
<repo>/sdkconfig                       # user-persisted Kconfig choices
<repo>/build/config/sdkconfig.h        # `-include sdkconfig.h` lands here
<repo>/build/config/sdkconfig.cmake    # `set(CONFIG_*)` for any future
                                       # CMake project
<repo>/.eh.target              # tiny state file recording the
                                       # persisted port (extra to idf.py
                                       # because we use port-type instead
                                       # of chip)
<cwd>/sdkconfig.defconfig              # default output of `save-defconfig`
                                       # (when invoked)
```

All four are git-ignored — they're build artifacts.

### What this closes

Before: linux_user builds used a 45-line hand-curated `sdkconfig.h`
fixture plus 4 `-D CONFIG_ESP_HOSTED_HOST_FEAT_*_READY=1` flags in
`EH_HOST_FORCE_FEATURES_ON_CFLAGS` to force per-feature gates on.
That meant Kconfig was the source of truth on IDF, but linux had a
parallel hand-curated source of truth — drift risk.

After: one `Kconfig` + `Kconfig.ext` tree.  IDF reads it via
`idf.py menuconfig` (unchanged); Linux reads it via
`./tools/eh.py menuconfig`.  Both produce a matching
`sdkconfig.h`.  The hand-curated fixture stays as the no-Kconfig-tool
fallback.

### Defaults

Per-target defaults live in `tools/sdkconfig.defaults.<target>`:

  - `tools/sdkconfig.defaults.linux_user` — baseline for the Linux
    user-space role.  Sets `ESP_HOSTED_HOST_TYPE_LINUX_USER=y`,
    `ESP_HOSTED_HOST_VSERIAL_LINUX_READY=y`, every per-feature
    `*_READY=y`, and the IDF-flavoured logging knobs.

A future port adds a new defaults file under
`tools/sdkconfig.defaults.<port>` and a `<port>` entry in
`VALID_TARGETS` in `eh.py`.

### Build wiring

`examples/common/linux_host/c/build_lib.sh` checks for
`<repo>/build/config/sdkconfig.h`.  When present, its directory is
prepended to the include path so `-include sdkconfig.h` resolves to
the generated header instead of the hand-curated fallback at
`host/compat/include/sdkconfig.h`.

Both paths stay green:
  - With generator run: 11/11 linux examples PASS, libeh_host.so
    196 + 11 symbols, no leaks.
  - Without (fixture fallback): 11/11 linux examples PASS.

### Why the fallback fixture isn't deleted yet

A fresh checkout doesn't have `build/config/` populated until
someone runs `eh.py set-target linux_user`.  The fixture at
`host/compat/include/sdkconfig.h` keeps the build green during that
window.  Retired once a top-level CMake project for linux_user runs
`eh.py reconfigure` automatically as a configure step.
