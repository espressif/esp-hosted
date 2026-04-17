# Implementation Plan: Hierarchical Kconfig/CMake Migration

This plan describes how to migrate the repo to the hierarchical Kconfig/CMake
design defined in `specs/22_component_kconfig_cmake_hierarchy.md`.

The plan explicitly calls out linker script and macro work, and uses a
single-extension pilot before full rollout.

---

## Goals (from design)
- Root IDF component only (`esp_hosted`); all lower layers are pure CMake
  static libs.
- Hierarchical Kconfig: root decides role/type and sources subtrees; each
  leaf Kconfig.ext is a `menuconfig` gate.
- No IDF LDFRAGMENTS; linker scripts are explicit and role-specific.
- Extensions register init/deinit via a common macro into role-specific
  sections.

---

## Risks / Cautions
- Linker scripts are fragile: do not assume any existing `.ld` or `.lf` is
  correct.
- Section placement errors can cause ELF segment conflicts and esptool
  failures. Verify placement per target.
- Avoid “silent” cross-linking: ensure each extension’s init is forced
  (`-Wl,--undefined=<init>`).
- Top-level role gating must wrap **both** CP and Host subtrees so menus
  remain consistent.

---

## Phase 0: Preparation
- Inventory current Kconfig tree and identify “root” configs that must be
  centralized (role, type, global gates).
- Inventory current linker scripts (`.ld`) and linker fragments (`.lf`) and
  mark all as **stale until verified**.
- Identify one extension to use as a pilot (e.g., CLI or host_ps).

---

## Phase 1: Core linker + macro infrastructure (CP/Host)

### 1.1 Linker scripts
Create role-specific scripts:
- `esp_hosted_cp_descs.ld`
- `esp_hosted_host_descs.ld`

Each must:
- Define `_eh_*_ext_descs_start/_end`
- Keep `.eh_*_ext_descs` section
- Insert into correct flash section without creating a new segment

### 1.2 Core CMake hooks
In the role-specific core CMake (coprocessor / host):
- Add `target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,-T<role_ld>")`
- Ensure this is gated by role selection (CP/Host).

### 1.3 Registration macros
In core headers:
- `esp_hosted_cp_core.h` exposes `EH_CP_EXT_REGISTER(...)`
- `esp_hosted_host_core.h` exposes `EH_HOST_EXT_REGISTER(...)`

Macros must:
- Declare a descriptor struct with common signature
- Place it in the correct section
- Enforce `used` and `aligned` attributes

---

## Phase 2: Kconfig hierarchy (core + subtree)

### 2.1 Root Kconfig
- Move global role/type decisions into a top-level root Kconfig.
- Add conditional `orsource` for CP subtree when CP is enabled.
- Add conditional `orsource` for Host subtree when Host is enabled.
- Ensure a single top-level gate (e.g., `ESP_HOSTED` or `USE_ESP_HOSTED`)
  wraps both CP and Host trees consistently.

### 2.2 Subtree Kconfig
- For CP and Host subtrees, create a “routing” Kconfig that `orsource`s
  leaf Kconfig.ext files in fixed order.

### 2.3 Leaf Kconfig.ext
Each leaf must follow:
```
menuconfig ESP_HOSTED_<LEAF>_ENABLED
    bool "Enable <leaf>"
    default y
if ESP_HOSTED_<LEAF>_ENABLED
    # options
endif
```

---

## Phase 3: CMake hierarchy

### 3.1 Root component
- Root CMake remains IDF component.
- Root includes CP/Host subtree CMake as required by role.

### 3.2 Subtree (non-leaf) CMake
- Only `add_subdirectory()` and conditional linking to child libs.
- Each conditional block must be gated by the leaf’s Kconfig flag.
- Subtree CMake files rely on `${COMPONENT_LIB}` from the root IDF component.

### 3.3 Leaf CMake
- Always `add_library(NAME STATIC ...)`
- No `idf_component_register`
- Link to IDF components via `idf::<name>` if needed.
- Each leaf must add `-Wl,--undefined=<init_fn>` to force extraction.
- Ensure init symbol names are unique and stable across extensions.

---

## Phase 4: Pilot extension

### 4.1 Pick one extension
Use one extension as a pilot (e.g., `esp_hosted_cp_ext_feat_cli`).

### 4.2 Apply full pattern
- Move its Kconfig to `Kconfig.ext` with a menuconfig gate.
- Update subtree Kconfig routing.
- Convert CMake to pure static lib if needed.
- Register init/deinit via new macro.
- Add linker `--undefined=<init_fn>` in parent CMake.

### 4.3 Validate
Build and boot:
- Ensure `.eh_*_ext_descs` contains the extension descriptor.
- Ensure init/deinit fires in correct priority order.
- Verify the linker map shows `.eh_*_ext_descs` placed in the same flash
  segment as `.flash.text` (no new segment).

If pilot fails, stop and fix before rolling out.

---

## Phase 5: Full rollout
- Repeat the pilot pattern for all extensions/components.
- Verify CP and Host trees independently.
- Keep the root Kconfig clean and authoritative.

---

## Deliverables
- New core linker scripts (CP/Host).
- New registration macros in core headers.
- Hierarchical Kconfig files.
- Hierarchical CMake with conditional subtree linking.
- Pilot extension validated.

---

## Cleanup (post-pilot)
- Remove temporary `set(common_shared_inc ...)` include propagation once all headers flow strictly through target dependencies.
- Remove `if(CMAKE_SCRIPT_MODE_FILE)` guards once IDF component discovery no longer evaluates pure-CMake subdirectories.
