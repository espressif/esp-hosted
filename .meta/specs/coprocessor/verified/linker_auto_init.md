<!-- %% sp.co.ve-la.o %% - always -->
---
type: spec
last_verified: 2026-03-30
---

# Linker Auto-Init

<!-- %% sp.co.ve-la.sym.o %% - always -->
## Linker Symbols

```c
extern eh_cp_feat_desc_t _eh_cp_ext_descs_start[];
extern eh_cp_feat_desc_t _eh_cp_ext_descs_end[];
```

Section name: `.eh_cp_ext_descs`

Core iterates `[start, end)` on boot, sorts by `priority` ascending (lower = runs first), calls each `init_fn`.
<!-- %% sp.co.ve-la.sym.c %% -->

<!-- %% sp.co.ve-la.mac.o %% - always -->
## Registration Macro

```c
// In extension source file:
EH_CP_FEAT_REGISTER(my_init, my_deinit, "my_ext", tskNO_AFFINITY, 200);

// Expands to (inlines struct into linker section):
static const eh_cp_feat_desc_t __eh_cp_feat_desc_my_init
    __attribute__((section(".eh_cp_ext_descs"), used, aligned(4))) = {
        .init_fn   = my_init,
        .deinit_fn = my_deinit,
        .name      = "my_ext",
        .affinity  = tskNO_AFFINITY,
        .priority  = 200,
    };
```

`eh_cp_feat_desc_t` fields: `init_fn`, `deinit_fn`, `name`, `affinity`, `priority`.
<!-- %% sp.co.ve-la.mac.c %% -->

<!-- %% sp.co.ve-la.build.o %% - context -->
## Build System Integration

**IDF (ldgen):** Fragment file generates the section in the linker script automatically. No manual `.ld` edit needed.

**Non-IDF / pure CMake:** Two options:
1. Include-style linker fragment (add via `target_link_options`)
2. CMake helper that generates a minimal `.ld` fragment and links it

Extension CMakeLists must have `REQUIRES eh_cp_core` so the section survives LTO/GC.
<!-- %% sp.co.ve-la.build.c %% -->

<!-- %% sp.co.ve-la.c %% -->
