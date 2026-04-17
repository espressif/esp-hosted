<!-- %% sp.te.ve-tp.o %% - always -->
---
type: spec
last_verified: 2026-04-13
---

# Test Principles

Authoring rules and infra invariants for the dual-DUT pytest-embedded suite under `tests/`.

<!-- %% sp.te.ve-tp.auth.o %% - always -->
## Authoring — Verify Behavior, Not Logs

Every test case must assert against actual feature output, not just boot or log lines.

| ❌ Avoid | ✅ Do |
|---------|-------|
| `assert "init ok" in dut.expect(...)` | Issue an RPC and assert the response payload, event content, or measurable side effect (data value, GPIO level, throughput, register state) |
| Pass-on-boot dummy tests | Drive the feature through its public API (RPC / event bus) and verify the output |

Audit policy: when adding a test, also confirm that any sibling test in the same file is not a log-only assertion. If found, mark it `xfail` with a TODO until it asserts real behavior.
<!-- %% sp.te.ve-tp.auth.c %% -->

<!-- %% sp.te.ve-tp.infra.o %% - always -->
## Infra — Flash Management

Test infra must flash the correct firmware to each DUT before every test pair.
Never rely on "already flashed" state from a prior run — stale firmware causes
false failures that look like real bugs.

- The matrix runner (`tests/run_all.py`) flashes both DUTs as part of pair setup
- Manual `pytest` runs against pre-flashed hardware are explicitly unsupported in CI
- If a test fails, the first triage step is to confirm the flashed binary matches the test's expected build
<!-- %% sp.te.ve-tp.infra.c %% -->

<!-- %% sp.te.ve-tp.tools.o %% - context -->
## Tooling — Multi-DUT vs Single-DUT

`agent_dev_utils` is single-DUT only. For multi-device tests use pytest-embedded
directly with custom orchestration (see `tests/infra/`).

- `agent_dev_utils` retains value for individual device crash analysis (backtrace decode, core dump inspection)
- Do not introduce it as a primary driver in dual-DUT test pairs
<!-- %% sp.te.ve-tp.tools.c %% -->

<!-- %% sp.te.ve-tp.c %% -->
