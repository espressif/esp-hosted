import { useEffect, useRef, useState } from 'react'
import { AnimatePresence, motion } from 'framer-motion'
import JSZip from 'jszip'
import { Panel, PanelGroup, PanelResizeHandle } from 'react-resizable-panels'
import type { ImperativePanelHandle } from 'react-resizable-panels'
import { configFromQuery } from './config/cli'
import { caseLabel, DEFAULT_SELECTED, evalCompat, SCENARIOS, ScenarioConfig } from './config/matrix'
import { buildMermaid, buildMermaidFromBlocks, SeqBlock } from './config/caseScripts'
import { buildLiveMermaid, buildSuiteMermaid, Catalog, loadCatalog, Tag } from './config/catalog'
import SequenceView from './components/SequenceView'
import LabPage, { DEFAULT_LAB_COL } from './components/LabPage'
import ExamplesPage from './components/ExamplesPage'
import type { LabCol } from './components/LabPage'
import { CaseStatus, ConfigPanel, Drawer, PlanConfig, PlanRail, RunResult } from './components/ui'
import { DEFAULT_LAB, isLabState, LabState, labValid } from './config/lab'

interface Plan { name: string; config: ScenarioConfig; presetId?: string; selected: string[] }
interface Report { id: string; start: string; end: string; durMs: number; results: RunResult[]; steps: SeqBlock[] }

const init = configFromQuery()
const STORAGE = 'esplab_plan'
const LAB_STORAGE = 'esplab_lab'
function loadLab(): LabState {
  try {
    const o = JSON.parse(localStorage.getItem(LAB_STORAGE) || 'null')
    if (isLabState(o)) return { ...o, activeLinkId: undefined }   // nothing active on boot
  } catch { /* ignore */ }
  return DEFAULT_LAB
}
const AXIS_KEYS = ['env', 'host', 'cp', 'transport', 'rpc'] as const
function loadPlan(): Plan | null {
  try {
    const raw = localStorage.getItem(STORAGE); if (!raw) return null
    const o = JSON.parse(raw)
    if (!o || typeof o !== 'object' || !o.config) return null
    if (!AXIS_KEYS.every(k => typeof o.config[k] === 'string')) return null
    const selected = Array.isArray(o.selected) ? o.selected : Array.isArray(o.cases) ? o.cases : null
    if (!selected) return null
    return { name: typeof o.name === 'string' ? o.name : 'default', config: o.config, presetId: o.presetId ?? o.scenarioId, selected }
  } catch { return null }
}
const INITIAL_PLAN: Plan = loadPlan() ?? { name: 'default', config: init.scenario, presetId: init.scenarioId, selected: [...DEFAULT_SELECTED] }
const fmt = (d: Date) => d.toLocaleTimeString([], { hour12: false })

function diffCount(a: Plan, b: Plan): number {
  let n = 0
  ;(AXIS_KEYS as readonly (keyof ScenarioConfig)[]).forEach(k => { if (a.config[k] !== b.config[k]) n++ })
  if (JSON.stringify(a.selected) !== JSON.stringify(b.selected)) n++
  return n
}
const handleV = 'w-1.5 mx-px rounded bg-white/5 hover:bg-emerald-400/40 transition-colors'
const handleH = 'h-1.5 my-px rounded bg-white/5 hover:bg-emerald-400/40 transition-colors'

export default function App() {
  const [page, setPage] = useState<'run' | 'lab' | 'report' | 'glance'>('glance')
  const [setupCol, setSetupCol] = useState<LabCol>(DEFAULT_LAB_COL)
  const [lab, setLab] = useState<LabState>(loadLab)
  const [plan, setPlan] = useState<Plan>(INITIAL_PLAN)
  const [catalog, setCatalog] = useState<Catalog | null>(null)
  const [active, setActive] = useState<Plan | null>(null)
  const [runId, setRunId] = useState<string | null>(null)
  const [focusSuite, setFocusSuite] = useState<string | null>(null)   // centre pane auto-follows the running suite
  const [planOn, setPlanOn] = useState<Record<'sw' | 'hw', boolean>>({ sw: true, hw: false })
  const [tileOn, setTileOn] = useState<Record<'sw' | 'hw', Record<Tag, boolean>>>({
    sw: { cp_only: true, 'cp-mcu': true, 'cp-linux': true, linux_only: true },
    hw: { cp_only: true, 'cp-mcu': true, 'cp-linux': true, linux_only: true },
  })
  const [runMode, setRunMode] = useState<null | 'run' | 'flash'>(null)
  const [drawer, setDrawer] = useState<'logs' | 'sdkconfig' | 'report' | null>(null)
  const [fitted, setFitted] = useState(false)
  const [planCollapsed, setPlanCollapsed] = useState(false)
  const [configCollapsed, setConfigCollapsed] = useState(false)
  const [theme, setTheme] = useState(init.theme)
  const [caseStatus, setCaseStatus] = useState<Record<string, CaseStatus>>({})
  const [log, setLog] = useState<{ build: string[]; host: string[]; cp: string[] }>({ build: [], host: [], cp: [] })
  const [liveRun, setLiveRun] = useState<any | null>(null)   // running scenario (phases+cases) for the live sequence
  const [reports, setReports] = useState<Report[]>([])
  const [viewId, setViewId] = useState<string | null>(null)   // report being replayed in the main pane
  const [savedJson, setSavedJson] = useState(JSON.stringify(INITIAL_PLAN))
  const [toast, setToast] = useState<string | null>(null)
  const runN = useRef(0)
  const runStart = useRef<Date | null>(null)
  const drawerRef = useRef<ImperativePanelHandle>(null)
  const planRef = useRef<ImperativePanelHandle>(null)
  const configRef = useRef<ImperativePanelHandle>(null)
  const pollRef = useRef<number | null>(null)

  const running = runMode !== null
  const cases = active?.selected ?? plan.selected
  const viewed = viewId ? reports.find(r => r.id === viewId) : null
  const lbl = catalog?.labelOf ?? caseLabel
  const mermaidText = viewed ? buildMermaidFromBlocks(viewed.steps)
    : liveRun ? buildLiveMermaid(liveRun)
    : catalog ? buildSuiteMermaid(focusSuite, catalog, plan.selected, caseStatus)
    : buildMermaid(cases, caseStatus, running)
  const pending = active ? diffCount(plan, active) : 0
  const runnable = evalCompat(plan.config).enabled && plan.selected.length > 0
  const dirty = JSON.stringify(plan) !== savedJson
  const labOk = labValid(lab).ok

  useEffect(() => {
    try { localStorage.setItem(LAB_STORAGE, JSON.stringify(lab)) } catch { /* ignore */ }
    fetch('/api/v1/lab', { method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(lab) })
      .catch(() => {})
  }, [lab])

  useEffect(() => {
    fetch('/api/v1/lab')
      .then(r => r.ok ? r.json() : null)
      .then(data => { if (isLabState(data)) setLab({ ...data, activeLinkId: undefined }) })
      .catch(() => {})
  }, [])

  // load the real catalogue; if the current selection is dummy (no id exists in
  // the catalogue), seed it with the real supported scenarios.
  useEffect(() => {
    loadCatalog().then(c => {
      if (!c) return
      setCatalog(c)
      setPlan(p => p.selected.some(id => c.scenarios[id]) ? p : { ...p, selected: [...c.supportedIds] })
    })
  }, [])
  useEffect(() => { document.documentElement.classList.toggle('light', theme === 'light') }, [theme])
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && drawer) { closeDrawer(); return }
      const t = e.target as HTMLElement | null
      if (t && /^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName)) return
      if (e.metaKey || e.ctrlKey || e.altKey) return
      if (e.key === '1') setPage('run')
      else if (e.key === '2') setPage('lab')
      else if (e.key === '3') setPage('report')
      else if (e.key === '4') setPage('glance')
      else if (e.key === 'r' && !running && runnable) start('run')
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [drawer, running, runnable, plan])

  const toStatus = (s: string): CaseStatus =>
    s === 'pass' ? 'pass' : s === 'fail' ? 'fail' : s === 'skip' ? 'skip'
      : ['queued', 'building', 'emulating', 'running'].includes(s) ? 'run' : 'idle'

  // poll the real backend run: map per-scenario status into caseStatus (PlanRail
  // dots + the suite sequence light up live), auto-follow the running suite, feed
  // the drawer real logs, and capture a real report when it finishes.
  useEffect(() => {
    if (!runId) return
    const tick = () => fetch(`/api/v1/run/${runId}`).then(r => r.json()).then((d: any) => {
      const status: Record<string, CaseStatus> = {}
      let active: any = null
      for (const sc of d.scenarios || []) {
        status[sc.id] = toStatus(sc.status)
        if (status[sc.id] === 'run' && !active) active = sc
      }
      // fall back to the last scenario that has produced output, so logs persist when done
      if (!active) active = (d.scenarios || []).filter((sc: any) => sc.logs?.build || sc.logs?.host).slice(-1)[0] || null
      setCaseStatus(status)
      setLiveRun(active)
      if (active) {
        setFocusSuite(catalog?.scenarios[active.id]?.suite ?? null)
        setLog({
          build: (active.logs?.build || '').split('\n'),
          host: (active.logs?.host || '').split('\n'),
          cp: (active.logs?.cp || '').split('\n'),
        })
      }
      if (d.status === 'done' || d.status === 'stopped') {
        const startDate = runStart.current ?? new Date(), endDate = new Date()
        setReports(prev => [{
          id: `run-${runN.current}`, start: fmt(startDate), end: fmt(endDate), durMs: endDate.getTime() - startDate.getTime(),
          results: (d.scenarios || []).map((sc: any) => ({
            id: sc.id, label: lbl(sc.id),
            status: (sc.status === 'pass' ? 'pass' : sc.status === 'fail' ? 'fail' : 'skip') as RunResult['status'],
            ms: Math.round((sc.dur_s || 0) * 1000),
          })),
          steps: [] as SeqBlock[],
        }, ...prev].slice(0, 25))
        setRunMode(null); setActive(null); setRunId(null)
        if (pollRef.current) { clearInterval(pollRef.current); pollRef.current = null }
      }
    }).catch(() => {})
    tick(); pollRef.current = window.setInterval(tick, 1500)
    return () => { if (pollRef.current) { clearInterval(pollRef.current); pollRef.current = null } }
  }, [runId, catalog])

  const editConfig = (patch: Partial<ScenarioConfig>) =>
    setPlan(p => ({ ...p, presetId: undefined, config: { ...p.config, ...patch } }))
  const onPreset = (id: string) => {
    const s = SCENARIOS.find(s => s.id === id); if (!s) return
    setPlan(p => ({ ...p, presetId: id, config: { ...s.config } }))
  }
  const toggleCase = (id: string) =>
    setPlan(p => ({ ...p, selected: p.selected.includes(id) ? p.selected.filter(c => c !== id) : [...p.selected, id] }))
  const toggleMany = (ids: string[], on: boolean) => setPlan(p => on
    ? { ...p, selected: [...p.selected, ...ids.filter(i => !p.selected.includes(i))] }
    : { ...p, selected: p.selected.filter(i => !ids.includes(i)) })
  const reorder = (ids: string[]) => setPlan(p => ({ ...p, selected: ids }))
  // a topology is visible if it's enabled under any active plan
  const topoEnabled = (t: Tag) => (planOn.sw && tileOn.sw[t]) || (planOn.hw && tileOn.hw[t])
  const benchOf = (p: 'sw' | 'hw', t: Tag) => (p === 'sw' && t === 'cp-mcu') ? 'esp-emu · C6 + P4' : ''
  // suites shown in the Plan rail, filtered to the enabled topology tiles
  const railSuites = catalog
    ? catalog.railSuites
        .map(su => ({ ...su, cases: su.cases.filter(id => topoEnabled(catalog.scenarios[id].topo)) }))
        .filter(su => su.cases.length)
    : undefined
  const onPlan = (p: 'sw' | 'hw', v: boolean) => setPlanOn(s => ({ ...s, [p]: v }))
  // toggling a tile auto-populates the selection: on → add its supported scenarios; off → drop that topology
  const onTile = (p: 'sw' | 'hw', t: Tag, v: boolean) => {
    setTileOn(s => ({ ...s, [p]: { ...s[p], [t]: v } }))
    if (!catalog) return
    setPlan(pl => {
      if (v) {
        const add = catalog.supportedIds.filter(id => catalog.scenarios[id].topo === t && !pl.selected.includes(id))
        return { ...pl, selected: [...pl.selected, ...add] }
      }
      return { ...pl, selected: pl.selected.filter(id => catalog.scenarios[id]?.topo !== t) }
    })
  }
  const clearSel = () => setPlan(p => ({ ...p, selected: [] }))
  const resetSel = () => setPlan(p => ({ ...p, selected: catalog ? [...catalog.supportedIds] : [...DEFAULT_SELECTED] }))
  const start = (mode: 'run' | 'flash') => {
    if (!plan.selected.length) return
    runN.current++; runStart.current = new Date()
    setViewId(null)   // a new run shows the live diagram
    setLog({ build: [], host: [], cp: [] }); setLiveRun(null)
    const idle: Record<string, CaseStatus> = {}; plan.selected.forEach(id => (idle[id] = 'idle')); setCaseStatus(idle)
    setActive({ ...plan, selected: [...plan.selected] }); setRunMode(mode)
    fetch('/api/v1/run', { method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ids: plan.selected, includeUnsupported: true }) })
      .then(r => r.json()).then(d => {
        if (d.runId) setRunId(d.runId)
        else { setRunMode(null); setActive(null); setToast(d.error || 'run failed'); window.setTimeout(() => setToast(null), 2500) }
      }).catch(() => { setRunMode(null); setActive(null); setToast('backend unreachable'); window.setTimeout(() => setToast(null), 2500) })
  }
  const stop = () => {
    if (runId) fetch(`/api/v1/run/${runId}/stop`, { method: 'POST' }).catch(() => {})
    setRunMode(null); setActive(null); setRunId(null)
    if (pollRef.current) { clearInterval(pollRef.current); pollRef.current = null }
  }
  const save = () => {
    const j = JSON.stringify(plan)
    try { localStorage.setItem(STORAGE, j) } catch { /* ignore */ }
    setSavedJson(j); setToast('Plan saved'); window.setTimeout(() => setToast(null), 1800)
  }
  const closeDrawer = () => { setFitted(false); setDrawer(null) }
  const toggleFit = () => { const t = fitted ? 38 : 80; drawerRef.current?.resize(t); setFitted(f => !f) }
  // bottom strip: jump to a Run pane (switch tab + open/toggle it)
  const jumpRun = (pane: 'plan' | 'env' | 'logs' | 'sdkconfig' | 'report') => {
    const switching = page !== 'run'
    setPage('run')
    if (pane === 'plan' || pane === 'env') {
      const p = (pane === 'plan' ? planRef : configRef).current
      if (p) { if (switching) p.expand(); else p.isCollapsed() ? p.expand() : p.collapse() }
    } else { setFitted(false); setDrawer(d => switching ? pane : d === pane ? null : pane) }
  }
  const jumpSetup = (pane: keyof LabCol) => {
    const switching = page !== 'lab'
    setPage('lab')
    setSetupCol(c => ({ ...c, [pane]: switching ? false : !c[pane] }))
  }
  const exportLogs = async () => {
    const zip = new JSZip()
    zip.file('host.log', log.host.join('\n') + '\n')
    zip.file('cp.log', log.cp.join('\n') + '\n')
    zip.file('reports.json', JSON.stringify(reports, null, 2))
    const blob = await zip.generateAsync({ type: 'blob' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a'); a.href = url; a.download = 'esplab-logs.zip'; a.click()
    setTimeout(() => URL.revokeObjectURL(url), 1000)
    setToast('Logs exported (.zip)'); window.setTimeout(() => setToast(null), 1800)
  }

  return (
    <div className="h-full w-full flex flex-col gap-2 p-2 overflow-hidden">
      {/* Set-up page — mounted when shown; pane visibility driven by setupCol from the strip */}
      {page === 'lab' && <div className="flex-1 min-h-0"><LabPage lab={lab} setLab={setLab} col={setupCol} /></div>}

      {/* At-a-glance page — real backend runner over the C6+P4 emulator bridge */}
      {page === 'glance' && <div className="flex-1 min-h-0"><ExamplesPage /></div>}

      {/* Reports page — run history; viewing a report replays its diagram on the Run tab */}
      {page === 'report' && (
        <div className="flex-1 min-h-0">
          <Drawer kind="report" log={log} reports={reports} viewId={viewId}
            onView={id => { setViewId(id); setPage('run') }} fitted={false} onToggleFit={() => {}} onClose={() => setPage('run')} />
        </div>
      )}

      {/* Run page — kept mounted, hidden via CSS so Mermaid + panels never re-init (no flicker) */}
      <div className={`flex-1 min-h-0 ${page === 'run' ? '' : 'hidden'}`}>
        <PanelGroup direction="vertical" className="h-full">
          <Panel order={1} minSize={25}>
            <PanelGroup direction="horizontal" autoSaveId="eh-cols" className="h-full">
              <Panel ref={planRef} order={1} collapsible collapsedSize={0} minSize={8} defaultSize={20}
                onCollapse={() => setPlanCollapsed(true)} onExpand={() => setPlanCollapsed(false)}>
                <div className="h-full pr-1"><PlanRail selected={plan.selected} caseStatus={caseStatus}
                  running={running} pending={pending} onToggleCase={toggleCase} onToggleMany={toggleMany} onReorder={reorder}
                  suites={railSuites} labelOf={catalog?.labelOf}
                  onClear={catalog ? clearSel : undefined} onReset={catalog ? resetSel : undefined} /></div>
              </Panel>
              <PanelResizeHandle className={handleV} />
              <Panel order={2} minSize={30}>
                <div className="h-full px-1">
                  <div className="glass h-full relative overflow-hidden">
                    <SequenceView text={mermaidText} />
                    {viewed && !running && (
                      <span className="absolute top-2 left-2 chip text-sky-300 cursor-pointer hover:bg-white/10"
                        onClick={() => setViewId(null)} title="back to live plan">viewing {viewed.id} · ✕ live</span>
                    )}
                  </div>
                </div>
              </Panel>
              <PanelResizeHandle className={handleV} />
              <Panel ref={configRef} order={3} collapsible collapsedSize={0} minSize={8} defaultSize={20}
                onCollapse={() => setConfigCollapsed(true)} onExpand={() => setConfigCollapsed(false)}>
                <div className="h-full pl-1">{catalog
                  ? <PlanConfig planOn={planOn} tileOn={tileOn} benchOf={benchOf} onPlan={onPlan} onTile={onTile} />
                  : <ConfigPanel config={plan.config} setConfig={editConfig} presetId={plan.presetId} onPreset={onPreset} />}</div>
              </Panel>
            </PanelGroup>
          </Panel>
          {drawer && (
            <>
              <PanelResizeHandle className={handleH} />
              <Panel ref={drawerRef} order={2} defaultSize={38} minSize={15}>
                <div className="h-full pt-1">
                  <Drawer kind={drawer} log={log} reports={reports} viewId={viewId} onView={setViewId}
                    fitted={fitted} onToggleFit={toggleFit} onClose={closeDrawer} />
                </div>
              </Panel>
            </>
          )}
        </PanelGroup>
      </div>

      {/* unified bottom strip — two tab groups; click a name to switch tab, click a pane to open it */}
      <div className="glass px-3 py-2 flex items-center gap-3 flex-wrap">
        <div className={`flex items-center gap-1.5 rounded-lg px-2 py-1 border transition-all ${page === 'run' ? 'border-emerald-400/40 bg-emerald-500/5' : 'border-white/10 opacity-50 hover:opacity-100'}`}>
          <button onClick={() => setPage('run')} className={`text-[11px] uppercase tracking-wider mr-1 ${page === 'run' ? 'text-emerald-300' : 'text-slate-400 hover:text-slate-200'}`}>Run</button>
          {([['plan', 'Plan', !planCollapsed], ['env', 'Env', !configCollapsed]] as const).map(([k, label, open]) => (
            <button key={k} onClick={() => jumpRun(k)} className={`btn !px-2 !py-0.5 text-xs ${page === 'run' && open ? 'btn-primary' : ''}`}>{label}</button>
          ))}
          <span className="opacity-20">|</span>
          {([['logs', 'Logs'], ['sdkconfig', 'sdkcfg']] as const).map(([k, label]) => (
            <button key={k} onClick={() => jumpRun(k)} className={`btn !px-2 !py-0.5 text-xs ${page === 'run' && drawer === k ? 'btn-primary' : ''}`}>{label}</button>
          ))}
        </div>

        <div className={`flex items-center gap-1.5 rounded-lg px-2 py-1 border transition-all ${page === 'lab' ? 'border-emerald-400/40 bg-emerald-500/5' : 'border-white/10 opacity-50 hover:opacity-100'}`}>
          <button onClick={() => setPage('lab')} className={`text-[11px] uppercase tracking-wider mr-1 ${page === 'lab' ? 'text-emerald-300' : 'text-slate-400 hover:text-slate-200'}`}>Set-up</button>
          {([['cp', 'cp'], ['host', 'host'], ['help', 'helpers'], ['types', 'types'], ['setup', 'set-up']] as const).map(([k, label]) => (
            <button key={k} onClick={() => jumpSetup(k)} className={`btn !px-2 !py-0.5 text-xs ${page === 'lab' && !setupCol[k] ? 'btn-primary' : ''}`}>{label}</button>
          ))}
        </div>

        <div className={`flex items-center gap-1.5 rounded-lg px-2 py-1 border transition-all ${page === 'report' ? 'border-emerald-400/40 bg-emerald-500/5' : 'border-white/10 opacity-50 hover:opacity-100'}`}>
          <button onClick={() => setPage('report')} className={`text-[11px] uppercase tracking-wider mr-1 ${page === 'report' ? 'text-emerald-300' : 'text-slate-400 hover:text-slate-200'}`}>Reports</button>
          <span className="chip">{reports.length}</span>
          <button onClick={exportLogs} className="btn !px-2 !py-0.5 text-xs" title="zip host + cp logs + reports">⤓ .zip</button>
        </div>

        <div className={`flex items-center gap-1.5 rounded-lg px-2 py-1 border transition-all ${page === 'glance' ? 'border-emerald-400/40 bg-emerald-500/5' : 'border-white/10 opacity-50 hover:opacity-100'}`}>
          <button onClick={() => setPage('glance')} className={`text-[11px] uppercase tracking-wider ${page === 'glance' ? 'text-emerald-300' : 'text-slate-400 hover:text-slate-200'}`}
            title="real C6+P4 emulator runs">Tests</button>
        </div>

        <span className="hidden xl:block text-[10px] text-slate-500 ml-2" title="keyboard shortcuts">1·2·3 tabs · ←→ select · ↵ edit · esc clear · r run</span>
        <div className="ml-auto flex items-center gap-2">
          {page === 'run' && (
            <>
              <button className={`btn !px-2 !py-1 text-xs ${dirty ? 'btn-primary' : ''}`} disabled={!dirty} onClick={save}
                title={dirty ? 'save plan' : 'no unsaved changes'}>✓ Save</button>
              <button className="btn btn-primary !px-3 !py-1 text-sm" disabled={running || !runnable} onClick={() => start('run')}>▶ Run</button>
              <button className="btn !px-3 !py-1 text-sm" disabled={running || !runnable || !labOk} onClick={() => start('flash')}
                title={labOk ? 'flash bound boards, then run' : 'bind Host + CP in Set-up first'}>⚡ Flash</button>
              <button className="btn !px-3 !py-1 text-sm" disabled={!running} onClick={stop}>■ Stop</button>
            </>
          )}
          <button className="btn !px-2 !py-1 text-xs" onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')} title="theme">{theme === 'dark' ? '☾' : '☀'}</button>
        </div>
      </div>

      <AnimatePresence>
        {toast && (
          <motion.div initial={{ y: 20, opacity: 0 }} animate={{ y: 0, opacity: 1 }} exit={{ opacity: 0 }}
            className="glass px-4 py-2 fixed bottom-24 left-1/2 -translate-x-1/2 text-sm z-50 text-ok">✓ {toast}</motion.div>
        )}
      </AnimatePresence>
    </div>
  )
}
