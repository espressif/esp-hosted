<!-- %% sp.co.ve-ep.o %% - always -->
---
type: spec
last_verified: 2026-03-31
---

# Events

This spec defines the event publishing and handling rules for coprocessor
features. It is intentionally minimal and must be kept aligned with legacy
`eh_cp_mcu` (legacy `esp_hosted_mcu`) behavior.

## Ownership Rules

- Feature extensions own feature event publishing.
- Core must not publish feature events if they can be separated.
- Event handlers must live in proto-capable extensions (e.g. RPC MCU),
  not in serializers. The serializer layer remains data-only.
- Core may publish only **minimal, non-separable** events required for boot
  or transport (keep this list minimal).

## Event Flow

- Feature events are posted on the feature’s own ESP_EVENT base.
- Event dispatch/encoding happens in the RPC MCU/FG handler
  (proto context only).

## Numbering / Compatibility

- Each extension defines its own base and starts event IDs at 0.
- Event IDs are extension-local and do not share a global range.
- Preserve legacy event numbering within each extension base.
- New events are appended after legacy IDs.
- Each feature extension defines:
  - An event base named `EH_CP_FEAT_<feature>_EVENT`.
  - An enum named `eh_cp_feat_<feature>_evt_t`.
  - Event IDs named `EH_CP_FEAT_<feature>_EVT_*`.

## Payload Rules

- Fixed-size payloads: allowed through ESP_EVENT.
- Variable/streaming payloads: must use a direct hook (peer-data style)
  instead of ESP_EVENT to avoid lifetime/ownership leaks.

## RPC Event API Boundary

- RPC event APIs are **never** called outside proto context.
  Event originators must post via ESP_EVENT or invoke the direct hook.

## Feature Gating

- All feature event publishing is guarded by the owning extension’s Kconfig
  `_READY` gate.
- All extension source files must be compiled under the owning feature flag.
- All `#include` of extension headers from outside the extension must be
  guarded by the extension’s `EH_CP_*_READY` macro (from `master_config.h`).
<!-- %% sp.co.ve-ep.c %% -->
