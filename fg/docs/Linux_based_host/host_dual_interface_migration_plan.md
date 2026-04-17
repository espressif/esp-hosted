# Host Dual-Interface Compatibility and OTA Migration Plan

## Scope
- Support both legacy wire interface IDs (`STA=0, AP=1, SERIAL=2, HCI=3, PRIV=4, TEST=5`) and new IDs (`INVALID=0, STA=1, AP=2, SERIAL=3, HCI=4, PRIV=5, TEST=6, ETH=7`).
- Support both legacy capability TLVs (`tag 0..4`) and modern boot TLVs (`tag 0x11+`, plus negotiation TLVs).

## Host Implementation Plan
1. **Boot event parser (PRIV init event)**
   - Accept both legacy and modern capability tag ranges in the same parser.
   - Do not fail on unknown TLVs; skip by length.
   - Prefer modern TLVs when both forms are present; fallback to legacy values otherwise.

2. **Interface-ID decode path**
   - Add decode helper that can map both old and new wire IDs to one internal host enum.
   - Use this helper at all ingress points before dispatch (data path and control path).

3. **Interface-ID encode path**
   - Add runtime-selectable encode mode:
     - `auto` (default): use modern IDs, fallback to legacy only when peer advertises legacy-only behavior.
     - `legacy`: force legacy wire IDs.
     - `modern`: force modern wire IDs.

4. **Capability negotiation behavior**
   - During startup, if both legacy and modern capability TLVs are received, mark peer as dual-compatible.
   - If only legacy TLVs are received, force host into `legacy` encode mode for that session.

## OTA / Migration Strategy
1. **Phase A (bridge release)**
   - Deploy coprocessor firmware that can emit modern TLVs and optionally legacy TLVs.
   - Keep host tolerant: parse both wire-ID schemes and both TLV sets.

2. **Phase B (host rollout)**
   - Roll out updated host driver/userspace with dual parser + dual encoder.
   - Default host remains `auto` to avoid field regressions.

3. **Phase C (seamless OTA)**
   - OTA first on coprocessor or host is both safe when host parser is tolerant:
     - Old host + new CP: CP can be built with legacy interface IDs/TLVs enabled.
     - New host + old CP: host decode fallback handles legacy IDs/TLVs.
   - No synchronized downtime required once host dual support is shipped.

4. **Phase D (deprecation)**
   - After fleet metrics show no legacy peers, disable legacy emit by default.
   - Keep legacy parse fallback for one deprecation window.

## Validation Matrix
- Old host + old CP (baseline).
- Old host + new CP (`legacy interface IDs` + `legacy boot TLVs` enabled).
- New host + old CP (host fallback path).
- New host + new CP (modern default path).
