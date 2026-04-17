<!-- %% sp-ac.o %% - always -->
---
type: agent-context
last_updated: 2026-03-30
---

# AGENT_CONTEXT

> Load this once per session before any non-trivial task.
> Subagents: parent has already routed you — read only the files specified.

<!-- %% sp-ac.repo.o %% - always -->
## Repo

`esp_hosted.new` — unified coprocessor firmware. ESP chip = coprocessor.
Hosts: Linux FG (kernel module) or MCU (IDF/FreeRTOS). V1: legacy header + RPC only.
Source: `modules/` | Examples: `fg/cp/examples/cp_{linux_fg,mcu}/` | IDF: `. ~/esp-idf/export.fish`
<!-- %% sp-ac.repo.c %% -->

<!-- %% sp-ac.pre.o %% - always -->
## Before Any Task

1. Read `todo.md` → find first unchecked task → load specs listed under it
2. V1 scope? → `.meta/specs/system/verified/scope.md` (section: v1w)
3. Relevant lessons? → `.meta/specs/tasks/lessons.md` — note any `[PENDING]`
4. New feature? → load `coprocessor/verified/feature_scaffold.md` + `component_map.md`
<!-- %% sp-ac.pre.c %% -->

<!-- %% sp-ac.rt.o %% - always -->
## Routing

Specs under `.meta/specs/`. `ctx.sh --zone always|context FILE`

| Area | Path (under .meta/specs/) |
|------|--------------------------|
| **Active work** | `tasks/todo.md` ← start here |
| Scope / deferred | `system/verified/scope.md` |
| Component→path map | `coprocessor/verified/component_map.md` |
| New feature scaffold | `coprocessor/verified/feature_scaffold.md` |
| Feature system + API | `coprocessor/verified/feature_system.md` |
| Linker auto-init | `coprocessor/verified/linker_auto_init.md` |
| Wire headers V1/V2 | `common/verified/wire_protocol.md` |
| PRIV TLV handshake | `common/verified/priv_handshake.md` |
| RPC dispatch + call flow | `coprocessor/verified/rpc_dispatch.md` |
| RPC versioning + namespaces | `coprocessor/verified/rpc_versioning.md` |
| Events | `coprocessor/verified/events.md` |
| Design decisions D1–D12 | `system/verified/design_decisions.md` |
| Phase status | `system/verified/implementation_status.md` |
| Feature parity map | `system/verified/feature_parity_map.md` |
| Test authoring + infra | `tests/verified/test_principles.md` |
| Unverified (post-V1 only) | `*/unverified/*.md` |
<!-- %% sp-ac.rt.c %% -->

<!-- %% sp-ac.mod.o %% - always -->
## Model Tiers

**`[ARCHITECT]`** — ambiguous design, interface decisions, debugging unknowns. Main session.
**`[MECHANICAL]`** — deterministic from spec: implement, explore, verify. Spawn cheapest subagent (~15-20x cheaper). Batch same-area work into one subagent.
<!-- %% sp-ac.mod.c %% -->

<!-- %% sp-ac.gat.o %% - always -->
## Gates

**G1 Scope**: V1 only. Out of scope → add to `.meta/specs/tasks/todo.md` future section, stop.
**G2 Brainstorm**: interface/component boundary/new extension → brainstorm first. Exception: single function, no interface change → implement directly.
**G3 Verify**: result matches spec? Mismatch → flag user. Never silent spec update.
<!-- %% sp-ac.gat.c %% -->

<!-- %% sp-ac.inv.o %% - always -->
## Invariants & Lessons

- Add to `.meta/specs/tasks/lessons.md` as `[PENDING]` on any correction. User decides verdict.
- Never update `verified/` spec without user approval.
- `unverified/` specs: do not implement for V1 without scope.md approval
- `verified/` spec `last_verified` > 60d: confirm against code before acting
- "X is complete" must cite file:line or build log
- Performance AND memory are always constraints
<!-- %% sp-ac.inv.c %% -->

<!-- %% sp-ac.c %% -->
