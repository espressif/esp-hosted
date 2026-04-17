<!-- %% me.br-260328ss.o %% - always -->
# Brainstorm: Spec System Design

**Date**: 2026-03-28
**Status**: Decisions captured → see SPEC_CREATION_SUMMARY.md + specs/README.md
**Outcome specs**: specs/README.md, specs/AGENT_CONTEXT.md, specs/sy/V/scope.md

---

<!-- %% me.br-260328ss.prob.o %% - always -->
## Problem Statement

25 flat numbered spec files, 13 root-level markdown files, 800-line CLAUDE.md prepended
every turn, no routing, no trust boundary between verified and future design, no cross-session
correction memory, expensive model used for mechanical work.
<!-- %% me.br-260328ss.prob.c %% -->

<!-- %% me.br-260328ss.dec.o %% - always -->
## Key Decisions Made

1. **Segment-based dirs** over flat numbered files — routing by directory name
2. **V/ and NV/** split — trust boundary, prevents implementing future design
3. **AGENT_CONTEXT.md** single load — merges routing + rules + model semantics
4. **Physical zone ordering** — contract first 50 lines, detail next 100, background after
5. **HTML comment tags** `<!-- %% TAG.o %% - pref -->` — functional with ctx.sh
6. **ctx.sh + check.sh scripts** — zone filter, tag extract, @ref resolve, validation
7. **Tag naming**: `S.dirs-file.section.o/c` with capitalized exceptions for most-used codes
8. **Three namespaces**: S (specs), B (brainstorm + transport dir), Z (source code)
9. **lessons.md** [PENDING] pattern — user owns verdict, agent never silently updates V/ spec
10. **Three gates**: scope, brainstorm (with trivial exception), verify
11. **[ARCHITECT]/[MECHANICAL]** labels — model tier guidance, no variables file
12. **Slim CLAUDE.md** to ~15 lines — biggest single cost reduction
<!-- %% me.br-260328ss.dec.c %% -->

<!-- %% me.br-260328ss.rej.o %% - context -->
## Rejected / Simplified

- HTML tags without script → both needed together or neither
- Variables file for model aliases → semantic labels sufficient
- CI lint → premature, defer until >20 specs
- 10-item acceptance checklist → reduced to 3 rules
- Persistent domain keeper subagents → complex, fragile, defer
- ctx.sh --task auto-routing → NLP problem, wrong-match failure mode, dropped
- 60-day hard rule → soft warning only
- SPEC_INDEX + agent_rules + variables as 3 files → merged into AGENT_CONTEXT.md
<!-- %% me.br-260328ss.rej.c %% -->

<!-- %% me.br-260328ss.risk.o %% - context -->
## Risks Identified

- **Spec rot**: stale V/ spec is worse than no spec. Mitigation: last_verified date + 60-day soft warning
- **AGENT_CONTEXT growth**: becomes new 800-line CLAUDE.md. Mitigation: 80-line hard limit
- **Tags without discipline**: tags are noise without ctx.sh. Decision: build both or neither
- **@ref broken refs**: silently empty is worse than duplication. Mitigation: ctx.sh errors loudly
- **Lesson graveyard**: [PENDING] pile up unreviewed. Mitigation: session-end surface rule
<!-- %% me.br-260328ss.risk.c %% -->

<!-- %% me.br-260328ss.cost.o %% - context -->
## Cost Analysis

- Haiku ~15-20x cheaper than Sonnet per token
- 7,000 Haiku tokens ≈ cost of ~400 Sonnet tokens
- Main session (Sonnet): minimize context aggressively
- Haiku subagents: generous context OK, specs are cheap to load
- CLAUDE.md slim: 30-40% session cost reduction immediately
- Spec system: expensive reasoning done once, cheap execution repeated
- Breakeven: 2 uses of a spec = ahead on cost vs no-spec approach
<!-- %% me.br-260328ss.cost.c %% -->

<!-- %% me.br-260328ss.c %% -->
