# `tools/vendor/` — vendored Python packages

These are byte-for-byte copies from `esp-idf-kconfig` v2.5.4
(installed by IDF into its Python env at
`<idf_env>/lib/python3.12/site-packages/`).  Vendored here so
`tools/eh.py` works identically on any Linux without
needing IDF to be installed or sourced.

| Package | Source | License |
|---|---|---|
| `kconfiglib/` | `esp-idf-kconfig` v2.5.4 — Espressif's Kconfig parser fork (extends upstream Kconfiglib with `help` on `choice` / `menu` and other IDF-needed conveniences) | Apache-2.0 |
| `menuconfig/` | `esp-idf-kconfig` v2.5.4 — Espressif's curses TUI (depends only on `kconfiglib` + Python stdlib) | Apache-2.0 |

## Why vendored, not pip-installed

Upstream PyPI `kconfiglib` rejects `help` blocks attached to
`choice` and `menu` nodes (with
`AttributeError: 'MenuNode' object has no attribute 'help'`).  IDF's
fork accepts them.  Our project's Kconfig tree uses choice-help in
several places (matching IDF conventions), so we need IDF's
parser.  Pinning a pip dependency would force users to manage a
specific kconfiglib version; vendoring is more reliable.

## Refresh procedure

When IDF ships a newer `esp-idf-kconfig`, refresh by re-copying:

```bash
IDF_PYENV=$(python3 -c "import sys; print(sys.prefix)")
# (run from a sourced IDF env so $IDF_PYENV resolves to its env)
cp -r "$IDF_PYENV/lib/python3.12/site-packages/kconfiglib" tools/vendor/
cp -r "$IDF_PYENV/lib/python3.12/site-packages/menuconfig"  tools/vendor/
rm -rf tools/vendor/{kconfiglib,menuconfig}/__pycache__
```

Bump the version reference in this file accordingly.

## Original sources

  - `kconfiglib`: <https://github.com/espressif/esp-idf-kconfig/tree/master/kconfiglib>
  - `menuconfig`: <https://github.com/espressif/esp-idf-kconfig/tree/master/menuconfig>

The upstream-of-upstream is Ulf Magnusson's
[Kconfiglib](https://github.com/ulfalizer/Kconfiglib); IDF's fork
maintains compatibility with that base while adding IDF-specific
extensions.
