<!-- %% sp-rm.o %% - always -->
# ESP-Hosted Specs — Tag Reference

<!-- %% sp-rm.lay.o %% - always -->
## Layout

```
.meta/
├── tools/     check.sh  ctx.sh  tag_registry.conf
├── brainstorm/          ← non-normative, never @ref'd
├── mcp_servers/         ← MCP server configs (user-invoked only)
└── specs/
    ├── README.md, AGENT_CONTEXT.md
    ├── tasks/     todo.md  lessons.md
    ├── system/    verified/ unverified/
    ├── common/    verified/ unverified/
    ├── coprocessor/ verified/ unverified/
    ├── host/      verified/ unverified/
    ├── transport/ verified/ unverified/
    ├── features/  extensions/
    └── legacy/    ← old numbered specs, not agent-loaded
```

Dir codes: `me`=.meta `sp`=.meta/specs `sy`=system `co`=coprocessor `cm`=common `ho`=host
`tr`=transport `fe`=features `ex`=extensions `ta`=tasks `br`=brainstorm
`lg`=legacy `tl`=tools `ve`=verified `nv`=unverified
<!-- %% sp-rm.lay.c %% -->

---

<!-- %% sp-rm.tag.o %% - always -->
## Tag Format

```
[!-- %% sp.co.ve-rd.ic.o %% - always --]   ← open (pref on open only)
content
[!-- %% sp.co.ve-rd.ic.c %% --]            ← close
```
(In actual files, use `<!--` / `-->` instead of `[!--` / `--]`)

Path encoding: `dirs.dirs-file.section.o|c`
- dirs separated by `.`; `-` separates last dir from file code; section follows `.`
- all codes lowercase, min 2 chars; globally unique in registry
- section codes local to file (unique per file, not in registry)

**Every file**: file-level tag `sp.sy.ve-sc.o` first line, `.c` last line.
**Every section**: section tag wrapping content.

Loading preference: `always` | `context` | `ignore`
- `always` — agent loads every session (contracts, invariants)
- `context` — agent loads when working in this area
- `ignore` — human only; agents skip
<!-- %% sp-rm.tag.c %% -->

---

<!-- %% sp-rm.ref.o %% - always -->
## Cross-Reference

```
[!-- %% @ref: sp.co.ve-rd.ic %% --]
```
(In actual files, use `<!--` / `-->` HTML comment syntax)

`ctx.sh --ref-resolve` resolves tag → file path → inlines content between `.ic.o` and `.ic.c`.
Broken ref = hard error (exit 1). Never silent.
<!-- %% sp-rm.ref.c %% -->

---

<!-- %% sp-rm.scr.o %% - context -->
## Scripts

```bash
check.sh [--soft]                              # validate all tags
ctx.sh --tag sp.co.ve-rd.ic                   # extract section by tag
ctx.sh --zone always|context|all FILE          # filter by preference
ctx.sh --ref-resolve FILE                      # expand @ref markers
ctx.sh --zone context --ref-resolve FILE       # combine
```

Collision: extend incrementally — `ic` → `ico` → `icoc`
Registry: `.meta/tools/tag_registry.conf` — add dir/file codes here.
<!-- %% sp-rm.scr.c %% -->

<!-- %% sp-rm.c %% -->
