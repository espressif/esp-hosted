# Component Enumeration & Kconfig Gating Plan

Status: OPEN
Owner: TBD
Last updated: 2026-03-19

## Problem Statement
We want a consistent, maintainable way to build ESP-Hosted examples that:
- Enumerates all repo components up front (IDF first pass),
- Builds only components that are required by dependencies,
- Applies a second‑level Kconfig gate for feature‑toggled components,
- Avoids per‑example `EXTRA_COMPONENT_DIRS` boilerplate.

## What the user wants (goals)
- Use `idf_component.yml` with `override_path` to point at a single canonical path.
- Keep example CMakeLists clean (no `EXTRA_COMPONENT_DIRS`).
- Ensure components are discoverable and only activated via dependency + Kconfig.
- Keep checksum behavior controlled by transport preference; code uses the common symbol.

## Current observations
- Examples currently set `EXTRA_COMPONENT_DIRS` in many CMakeLists files.
- Only a couple of examples currently contain `idf_component.yml` entries for esp_hosted.
- IDF component discovery is flat and only scans component search paths.
- Components can use `add_subdirectory()`, but that does not create IDF components.

## Possible approaches
### A) `override_path` → `/components` (IDF‑native enumeration)
**How it works**
- `idf_component.yml` points to repo `/components`.
- IDF enumerates each subcomponent with its own `idf_component_register`.

**Pros**
- Cleanest IDF‑native behavior.
- Preserves per‑component Kconfig and dependency graph.
- No per‑example `EXTRA_COMPONENT_DIRS`.

**Cons**
- Requires each component subdir to have its own `CMakeLists.txt` + `idf_component_register`.
- Requires adding `idf_component.yml` to each example (or a template).

### B) Root as a “super‑component” with `add_subdirectory(...)`
**How it works**
- Root is treated as a component via `override_path`.
- Root `CMakeLists.txt` calls `add_subdirectory(components/...)` and links targets.
- Root `Kconfig` uses `rsource/orsource` to pull in sub‑Kconfigs.

**Pros**
- Single override path.
- Centralized control.

**Cons**
- Subcomponents are no longer IDF components; dependency resolution is manual.
- Kconfig must be manually aggregated.
- Diverges from standard IDF component enumeration.

### C) Wrapper build script + `IDF_EXTRA_COMPONENT_DIRS`
**How it works**
- Build wrapper sets `IDF_EXTRA_COMPONENT_DIRS` once.
- Examples remain clean.

**Pros**
- Minimal repo changes.
- Preserves IDF component enumeration.

**Cons**
- Requires custom build invocation.
- Tooling/IDE support depends on using the wrapper.

## Challenges / Risks
- Maintaining per‑component Kconfig visibility if using a super‑component.
- Avoiding drift between example builds and CI (wrapper vs direct `idf.py`).
- Ensuring all components have proper `idf_component_register` when enumerated directly.

## Approach right now (working direction)
- Option selection still open; prefer an IDF‑native approach if feasible.
- Keep Kconfig gating as the second‑level filter even when dependencies pull components.

