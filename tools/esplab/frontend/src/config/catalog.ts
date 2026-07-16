// Real test catalogue adapter. Loads /api/v1/tests and maps it into the shapes
// the existing UI already consumes (rail suites of selectable ids + a label
// lookup), so PlanRail / SequenceView keep their code and just show real data.
// The dummy matrix.ts stays as the fallback / type contract.

export type Tag = 'cp_only' | 'cp-mcu' | 'cp-linux' | 'linux_only'
export const TAGS: Tag[] = ['cp_only', 'cp-mcu', 'cp-linux', 'linux_only']
export const TAG_LABEL: Record<Tag, string> = { cp_only: 'cp only', 'cp-mcu': 'cp + mcu', 'cp-linux': 'cp + linux', linux_only: 'linux only' }
export function topoOf(b: { host_dir?: string; cp_dir?: string }): Tag {
  const h = (b?.host_dir || '') + ' ' + (b?.cp_dir || '')
  if (h.includes('linux_802_3_host')) return 'cp-linux'
  if (h.includes('mcu_host') || h.includes('esp_host')) return 'cp-mcu'
  if (/\/cp(\s|$)/.test(h) && !h.includes('host')) return 'cp_only'
  return 'cp-mcu'
}

export interface CatCase { id: string; label: string; kind?: string; expect: string[] }
export interface CatScenario {
  id: string; label: string; kind: string; category: string; support: string; setup: string
  build: { cp_dir: string; host_dir: string; host_overlay?: string[]; cp_overlay?: string[]; embed_cp_app?: string }
  cases: CatCase[]; suite: string; suiteLabel: string; topo: Tag
}
export interface RailSuite { id: string; label: string; cases: string[] }   // cases = scenario ids
export interface Catalog {
  railSuites: RailSuite[]
  scenarios: Record<string, CatScenario>
  labelOf: (id: string) => string
  supportedIds: string[]
}

export async function loadCatalog(): Promise<Catalog | null> {
  try {
    const d = await fetch('/api/v1/tests').then(r => r.json())
    const scenarios: Record<string, CatScenario> = {}
    const railSuites: RailSuite[] = []
    for (const su of d.suites as any[]) {
      const ids: string[] = []
      for (const sc of su.scenarios as any[]) {
        scenarios[sc.id] = {
          ...sc, suite: su.id, suiteLabel: su.label, topo: topoOf(sc.build || {}),
          cases: (sc.cases || []).map((c: any) => ({ id: c.id, label: c.label, kind: c.kind, expect: c.assert?.expect || [] })),
        }
        ids.push(sc.id)
      }
      railSuites.push({ id: su.id, label: su.label, cases: ids })
    }
    const supportedIds = Object.values(scenarios).filter(s => s.support === 'supported').map(s => s.id)
    return { railSuites, scenarios, labelOf: (id: string) => scenarios[id]?.label || id, supportedIds }
  } catch {
    return null
  }
}

const clean = (s: string) => s.replace(/[:#;\n]/g, ' ').replace(/[<>]/g, '').slice(0, 42).trim()
const RGB: Record<string, string> = { pass: 'rgb(20,58,44)', fail: 'rgb(68,28,38)', running: 'rgb(20,46,72)', run: 'rgb(20,46,72)' }
const glyph = (st?: string) => st === 'pass' ? ' ✓' : st === 'fail' ? ' ✗' : (st === 'running' || st === 'run') ? ' ⏳' : ''

// Live, dynamic sequence for the scenario currently executing: phases appear and
// tint as the run advances (build CP → build host → bench + cases), so the diagram
// animates with the real run rather than just sitting there.
export function buildLiveMermaid(rs: {
  label: string; status: string
  phases: { build_cp: { status: string }; build_host: { status: string }; emulate: { status: string } }
  cases: { id: string; label: string; status: string }[]
}): string {
  const L = ['sequenceDiagram', 'participant H as Host (P4)', 'participant C as CP (C6)', `Note over H,C: ${clean(rs.label)}`]
  const phase = (st: string, line: string) => {
    if (RGB[st]) L.push(`rect ${RGB[st]}`)
    L.push(line)
    if (RGB[st]) L.push('end')
  }
  phase(rs.phases.build_cp.status, `H->>C: build coprocessor (C6)${glyph(rs.phases.build_cp.status)}`)
  phase(rs.phases.build_host.status, `H->>C: build host (P4)${glyph(rs.phases.build_host.status)}`)
  const em = rs.phases.emulate.status
  if (RGB[em]) L.push(`rect ${RGB[em]}`)
  L.push(`H->>C: launch bench + stimulus${glyph(em)}`)
  rs.cases.forEach(c => L.push(`C-->>H: ${clean(c.label)}${glyph(c.status)}`))
  if (RGB[em]) L.push('end')
  return L.join('\n')
}

// the suites that have at least one selected scenario (catalogue order)
export function selectedSuites(cat: Catalog, selectedIds: string[]): { id: string; label: string }[] {
  const sel = new Set(selectedIds)
  return cat.railSuites
    .filter(su => su.cases.some(id => sel.has(id)))
    .map(su => ({ id: su.id, label: su.label }))
}

// One suite's own host⇄CP sequence: each selected scenario is a region tinted by
// its live status (idle / running / pass / fail), so the run animates per case.
const TINT: Record<string, string> = { pass: 'rgb(20,58,44)', fail: 'rgb(68,28,38)', run: 'rgb(20,46,72)', skip: 'rgb(40,46,58)', idle: 'rgb(38,44,56)' }
export function buildSuiteMermaid(focus: string | null, cat: Catalog, selectedIds: string[], status: Record<string, string>): string {
  const present = selectedSuites(cat, selectedIds)
  const L = ['sequenceDiagram', 'participant H as Host (P4)', 'participant C as CP (C6)']
  if (!present.length) { L.push('Note over H,C: select scenarios in the Plan pane'); return L.join('\n') }
  const suiteId = (focus && present.some(s => s.id === focus)) ? focus : present[0].id
  const su = cat.railSuites.find(s => s.id === suiteId)!
  const sel = new Set(selectedIds)
  L.push(`Note over H,C: ${clean(su.label)}`)
  su.cases.filter(id => sel.has(id)).slice(0, 14).forEach(id => {
    const s = cat.scenarios[id]; if (!s) return
    L.push(`rect ${TINT[status[id] || 'idle'] || TINT.idle}`)
    L.push(`H->>C: ${clean(s.label)}`)
    const marker = s.cases.flatMap(c => c.expect)[0]
    if (marker) L.push(`C-->>H: ${clean(marker)}`)
    L.push('end')
  })
  return L.join('\n')
}
