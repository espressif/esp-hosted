# Component/Kconfig/CMake Hierarchy Design (Hosted)

This document captures the design decisions for hosted components, Kconfig
hierarchy, CMake hierarchy, and extension registration.

These decisions are derived from the working PoC example and are intended to
be authoritative for future refactors.

---

## Goals
- Single IDF component at the top (esp_hosted), with all lower layers as
  pure CMake static libraries.
- Hierarchical Kconfig with explicit gating for each component/extension.
- Extensions register init/deinit in linker sections for auto-discovery.
- No IDF LDFRAGMENTS usage; linker scripts are explicit and carefully scoped.

---

## Kconfig hierarchy

### Root Kconfig (top-level)
- Owns **global** decisions: role (CP/Host), platform type, and main feature
  gates.
- Sources lower levels **conditionally**:
  - If CP role is enabled: source CP subtree.
  - If Host role is enabled: source Host subtree.

### Middle-level Kconfig (per subtree)
- Each subtree Kconfig uses `orsource` to include its leaf `Kconfig.ext` files
  in a fixed order.

### Leaf Kconfig.ext (per component/extension)
Each leaf must declare:

```
menuconfig ESP_HOSTED_XXXX
    bool "Enable XXXX"
    default y

if ESP_HOSTED_XXXX
    # component-specific options
endif
```

### Rule
- `Kconfig` (without `.ext`) is only a **router** (sources other Kconfig.ext).
- All **real options** live in `Kconfig.ext`.

---

## CMake hierarchy

### Root (IDF component)
- `esp_hosted` is the only IDF component.
- It conditionally includes subtrees via `add_subdirectory()` based on Kconfig.
- It links leaf static libraries into `${COMPONENT_LIB}`.

### Middle-level CMake (subtree)
- Only `add_subdirectory()` and conditional linking.
- No `idf_component_register` anywhere below the root.

### Leaf CMake (pure CMake)
- Must be **static libraries only** (`add_library(NAME STATIC ...)`).
- May depend on IDF components **only via** `idf::<component>` target alias.
- Must not use `idf_component_register`.

### Link forcing
Each leaf extension must ensure its init function is pulled in:

```
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--undefined=<extension_init_fn>"
)
```

---

## Extension registration

### Section and symbols
- CP uses `.eh_cp_ext_descs`
- Host uses `.eh_host_ext_descs`

Each side has its own linker script:
- `esp_hosted_cp_descs.ld`
- `esp_hosted_host_descs.ld`

### Linker script contract
Example (CP):

```
SECTIONS
{
    .eh_cp_ext_descs : ALIGN(4)
    {
        _eh_cp_ext_descs_start = .;
        KEEP(*(.eh_cp_ext_descs))
        _eh_cp_ext_descs_end = .;
    }
} INSERT BEFORE .flash.text
```

Host is identical with `.eh_host_ext_descs` and `_eh_host_ext_descs_*`.

### Extension C code (leaf example)

The entire extension `.c` file must be wrapped with its own feature flag to
prevent global space pollution when the extension is disabled:

```c
#ifdef CONFIG_ESP_HOSTED_XXXX

typedef esp_err_t (*_init_fn_t)(void);

typedef struct {
    _init_fn_t  init_fn;
    _init_fn_t  deinit_fn;
    const char *name;
    int         affinity;
    int         priority;
} _ext_desc_t;

/* init and deinit MUST be non-static.
 * -Wl,--undefined=<init_fn> in CMakeLists forces the linker to pull this
 * object from the static archive. A static function has no external symbol
 * so --undefined cannot reference it -- the object would never be pulled. */
esp_err_t my_ext_init(void)  { ... }
esp_err_t my_ext_deinit(void) { ... }

static const _ext_desc_t __my_ext_desc
    __attribute__((section(".eh_cp_ext_descs"), used, aligned(4))) = {
    .init_fn   = my_ext_init,
    .deinit_fn = my_ext_deinit,
    .name      = "my_ext",
    .affinity  = -1,
    .priority  = 150,
};

#endif /* CONFIG_ESP_HOSTED_XXXX */
```

### Macro policy
Use a common macro (e.g., in `esp_hosted_cp_core.h` /
`esp_hosted_host_core.h`) for registration to avoid boilerplate and ensure
consistency.

---

## Dos and Don'ts

### Do
- Keep **only one** IDF component (root).
- Keep Kconfig options in leaf `Kconfig.ext` files.
- Use explicit linker scripts per role (CP/Host).
- Use `-Wl,--undefined=<init_fn>` to force extraction of leaf extensions.
- Keep init/deinit signatures consistent across extensions.
- Wrap the entire extension `.c` content in `#ifdef CONFIG_ESP_HOSTED_XXXX`.

### Don't
- Don't use IDF LDFRAGMENTS anywhere for hosted extensions.
- Don't put options directly in leaf `Kconfig` files.
- Don't use `idf_component_register` in leaf or middle components.
- Don't rely on linker section KEEP without also forcing object extraction.
- Don't mix CP/Host linker section names or symbols.
- Don't make `init`/`deinit` functions `static` -- `--undefined` cannot
  reference a static symbol; the object will not be pulled from the archive.
- Don't define extension code outside `#ifdef CONFIG_ESP_HOSTED_XXXX` guards.

---

## Notes
- Linker scripts are delicate; avoid changes unless required.
- CP and Host sections must remain separate and consistent.
- The PoC example is the reference implementation for structure.
