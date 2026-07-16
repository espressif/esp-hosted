import { useEffect, useMemo, useRef, useState } from 'react'
import { AnimatePresence, motion, Reorder } from 'framer-motion'
import {
  AXES, AxisKey, caseLabel, DEFAULT_CONFIG, evalCompat, optLabel,
  SCENARIOS, ScenarioConfig, SUITES,
} from '../config/matrix'
import { CP_DEFAULTS, CP_SDKCONFIG, HOST_DEFAULTS, HOST_SDKCONFIG, LOG_PATHS, SDKCONFIG_PATHS } from '../config/samples'
import { Tag, TAG_LABEL, TAGS } from '../config/catalog'

type PlanId = 'sw' | 'hw'
const PLAN_PILL: Record<PlanId, string> = { sw: 'bg-emerald-400 text-emerald-950', hw: 'bg-amber-400 text-amber-950' }

export type CaseStatus = 'idle' | 'run' | 'pass' | 'fail' | 'skip'
export interface RunResult { id: string; label: string; status: 'pass' | 'fail' | 'skip'; ms: number }

const slide = { initial: { x: 20, opacity: 0 }, animate: { x: 0, opacity: 1 }, exit: { x: 20, opacity: 0 } }
const up = { initial: { y: 28, opacity: 0 }, animate: { y: 0, opacity: 1 }, exit: { y: 28, opacity: 0 } }

/* ── header summary: active plan + environment ─────────────────────────── */
export function ScenarioBar({ config, caseCount }: {
  config: ScenarioConfig; caseCount: number
}) {
  const compat = evalCompat(config)
  return (
    <div className="flex items-center gap-2 flex-1 min-w-0">
      <span className="text-xs uppercase tracking-wider text-slate-400">Plan</span>
      <span className="chip">{caseCount} cases</span>
      <div className="flex gap-1 ml-auto flex-wrap">
        {(['env', 'host', 'cp', 'transport', 'rpc'] as AxisKey[]).map(a =>
          <span key={a} className="chip">{optLabel(a, (config as any)[a])}</span>)}
      </div>
      <span className={`chip ${compat.enabled ? 'text-ok' : 'text-bad'}`} title={compat.reason}>
        {compat.enabled ? 'valid' : 'disabled'}</span>
    </div>
  )
}

function AxisPicker({ axis, value, onChange }: { axis: AxisKey; value: string; onChange: (v: string) => void }) {
  return (
    <div>
      <div className="text-xs text-slate-400 mb-1">{AXES[axis].label}</div>
      <div className="flex flex-wrap gap-1.5">
        {AXES[axis].options.map(o => (
          <button key={o.id} onClick={() => onChange(o.id)}
            className={`chip ${value === o.id ? 'bg-emerald-500/25 border-emerald-400/40 text-emerald-100' : ''}`}>{o.label}</button>
        ))}
      </div>
    </div>
  )
}

/* ── left rail: plan = suites (amendable) + run order ──────────────────── */
function CaseDot({ status }: { status?: CaseStatus }) {
  if (status === 'pass') return <span className="text-ok text-sm leading-none">✓</span>
  if (status === 'fail') return <span className="text-bad text-sm leading-none">✗</span>
  if (status === 'skip') return <span className="inline-flex h-2 w-2 rounded-full bg-slate-500" />
  if (status === 'run') return <span className="inline-flex h-2.5 w-2.5 rounded-full bg-sky-400" style={{ animation: 'hbpulse 1s ease-in-out infinite' }} />
  return <span className="inline-flex h-2 w-2 rounded-full bg-white/25" />
}

function TriCheck({ state, onClick, title }: { state: 'all' | 'some' | 'none'; onClick: () => void; title?: string }) {
  return (
    <button onClick={onClick} title={title || 'select all / none'}
      className={`h-4 w-4 shrink-0 rounded border flex items-center justify-center text-[10px] leading-none
        ${state === 'all' ? 'bg-emerald-500/30 border-emerald-400/50 text-emerald-100'
        : state === 'some' ? 'bg-emerald-500/15 border-emerald-400/40 text-emerald-200' : 'border-white/25 hover:border-white/50'}`}>
      {state === 'all' ? '✓' : state === 'some' ? '–' : ''}
    </button>
  )
}

export function PlanRail(props: {
  selected: string[]; caseStatus: Record<string, CaseStatus>
  running: boolean; pending: number
  onToggleCase: (id: string) => void; onToggleMany: (ids: string[], on: boolean) => void; onReorder: (ids: string[]) => void
  suites?: { id: string; label: string; cases: string[] }[]; labelOf?: (id: string) => string
  onClear?: () => void; onReset?: () => void
}) {
  const { selected, caseStatus, running, pending } = props
  const SU = props.suites ?? SUITES                 // real catalogue when provided, else the dummy
  const lbl = props.labelOf ?? caseLabel
  const [expanded, setExpanded] = useState<Record<string, boolean>>({})   // boot collapsed; open on demand
  const [sec, setSec] = useState({ suites: true, run: false })
  const [resetArm, setResetArm] = useState(false)
  const allIds = useMemo(() => Array.from(new Set(SU.flatMap(s => s.cases))), [SU])
  const selAll = allIds.filter(i => selected.includes(i)).length
  const masterState: 'all' | 'some' | 'none' = selAll === 0 ? 'none' : selAll === allIds.length ? 'all' : 'some'
  // live progress tint + auto-browse: as a scenario runs, open its suite and scroll it into view
  const tint = (st?: CaseStatus) => st === 'run' ? 'bg-sky-500/15' : st === 'pass' ? 'bg-emerald-500/10'
    : st === 'fail' ? 'bg-rose-500/10' : st === 'skip' ? 'opacity-50' : ''
  const runningId = useMemo(() => Object.keys(caseStatus).find(id => caseStatus[id] === 'run'), [caseStatus])
  const rowRefs = useRef<Record<string, HTMLDivElement | null>>({})
  useEffect(() => {
    if (!runningId) return
    const su = SU.find(s => s.cases.includes(runningId))
    if (su) setExpanded(e => e[su.id] ? e : { ...e, [su.id]: true })
    rowRefs.current[runningId]?.scrollIntoView({ block: 'nearest', behavior: 'smooth' })
  }, [runningId])
  return (
    <div className="glass h-full w-full p-3 flex flex-col gap-3 overflow-hidden">
      <div className="flex items-center gap-2">
        <span className="font-semibold">Plan</span>
        {running && pending > 0 && <span className="chip text-warn" title="applied after this run">+{pending}</span>}
        <span className="flex-1" />
        {props.onClear && <button className="btn !px-2 !py-0.5 text-xs" title="deselect everything" disabled={running}
          onClick={() => { setResetArm(false); props.onClear!() }}>Clear</button>}
        {props.onReset && <button className={`btn !px-2 !py-0.5 text-xs ${resetArm ? 'text-bad border-rose-400/40' : ''}`}
          title="reset selection to the supported scenarios" disabled={running}
          onClick={() => { if (resetArm) { props.onReset!(); setResetArm(false) } else setResetArm(true) }}
          onBlur={() => setResetArm(false)}>{resetArm ? 'Confirm reset?' : 'Reset'}</button>}
      </div>

      {/* suites — collapsible section, gives space to run order when closed */}
      <div className={`flex flex-col min-h-0 ${sec.suites ? 'flex-1' : ''}`}>
        <div className="flex items-center gap-2">
          <button onClick={() => setSec(s => ({ ...s, suites: !s.suites }))}
            className="flex items-center gap-2 flex-1 text-xs uppercase tracking-wider text-slate-400 hover:text-slate-200">
            <span>{sec.suites ? '▾' : '▸'}</span>Suites
          </button>
          <TriCheck state={masterState} onClick={() => props.onToggleMany(allIds, masterState !== 'all')} />
          <span className="chip">{selAll}/{allIds.length}</span>
        </div>
        {sec.suites && (
          <div className="flex-1 overflow-auto -mx-1 px-1 mt-1 space-y-1">
            {SU.map(s => {
              const sel = s.cases.filter(c => selected.includes(c)).length
              const isOpen = !!expanded[s.id]
              const cs = s.cases.map(c => caseStatus[c])
              const sRun = cs.includes('run')
              const sFail = cs.includes('fail')
              const sDone = s.cases.length > 0 && cs.every(x => x === 'pass' || x === 'fail' || x === 'skip')
              return (
                <div key={s.id} className={`rounded-lg border border-white/10 ${sRun ? 'bg-sky-500/10' : 'bg-white/5'}`}>
                  <div className="w-full flex items-center gap-2 px-2 py-1.5">
                    <button onClick={() => setExpanded(e => ({ ...e, [s.id]: !e[s.id] }))} className="flex items-center gap-2 flex-1 text-left">
                      <span className="text-slate-500 text-xs">{isOpen ? '▾' : '▸'}</span>
                      {sRun ? <CaseDot status="run" /> : sDone ? <span className={`text-sm leading-none ${sFail ? 'text-bad' : 'text-ok'}`}>{sFail ? '✗' : '✓'}</span> : null}
                      <span className="text-sm">{s.label}</span>
                    </button>
                    <TriCheck state={sel === 0 ? 'none' : sel === s.cases.length ? 'all' : 'some'}
                      onClick={() => props.onToggleMany(s.cases, sel !== s.cases.length)} />
                    <span className="chip">{sel}/{s.cases.length}</span>
                  </div>
                  {isOpen && (
                    <div className="px-2 pb-2 space-y-0.5">
                      {s.cases.map(c => (
                        <div key={c} ref={el => (rowRefs.current[c] = el)}
                          className={`flex items-center gap-2 text-sm rounded px-1 py-0.5 transition-colors ${tint(caseStatus[c])}`}>
                          <input type="checkbox" checked={selected.includes(c)} onChange={() => props.onToggleCase(c)} className="accent-emerald-400" />
                          <CaseDot status={caseStatus[c]} />
                          <span className="flex-1 truncate cursor-pointer hover:text-white text-slate-300" onClick={() => props.onToggleCase(c)}>{lbl(c)}</span>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              )
            })}
          </div>
        )}
      </div>

      {/* run order — collapsible section */}
      <div className={`flex flex-col min-h-0 ${sec.run ? 'flex-1' : ''}`}>
        <button onClick={() => setSec(s => ({ ...s, run: !s.run }))}
          className="flex items-center gap-2 text-xs uppercase tracking-wider text-slate-400 hover:text-slate-200">
          <span>{sec.run ? '▾' : '▸'}</span>Run order · drag<span className="chip ml-1">{selected.length}</span>
        </button>
        {sec.run && (
          <div className="flex-1 overflow-auto -mx-1 px-1 mt-1">
            {selected.length === 0 && <div className="text-xs text-slate-500">select cases from suites above</div>}
            <Reorder.Group axis="y" values={selected} onReorder={props.onReorder} className="space-y-1">
              {selected.map(id => (
                <Reorder.Item key={id} value={id}
                  className="flex items-center gap-2 rounded-lg border border-white/10 bg-white/5 px-2 py-1.5 cursor-grab active:cursor-grabbing">
                  <CaseDot status={caseStatus[id]} />
                  <span className="text-slate-500 select-none text-xs">⋮⋮</span>
                  <span className="text-sm flex-1 truncate">{lbl(id)}</span>
                  <button className="text-slate-500 hover:text-bad text-xs" onClick={() => props.onToggleCase(id)}>✕</button>
                </Reorder.Item>
              ))}
            </Reorder.Group>
          </div>
        )}
      </div>
    </div>
  )
}

/* ── right: environment config (preset + axis tabs). Logs are NOT here. ── */
const TAB_LABEL: Record<string, string> = { host: 'Host', transport: 'Transport', cp: 'CP', env: 'Env' }
export function ConfigPanel({ config, setConfig, presetId, onPreset }: {
  config: ScenarioConfig; setConfig: (patch: Partial<ScenarioConfig>) => void
  presetId?: string; onPreset: (id: string) => void
}) {
  const [tab, setTab] = useState<'host' | 'transport' | 'cp' | 'env'>('host')
  const changed = (Object.keys(config) as (keyof ScenarioConfig)[]).filter(k => config[k] !== DEFAULT_CONFIG[k]).length
  return (
    <div className="glass h-full w-full p-3 flex flex-col gap-3 overflow-auto">
      <div className="flex items-center justify-between">
        <span className="font-semibold">Environment</span>
        {changed > 0 && <span className="chip text-warn">{changed} changed</span>}
      </div>
      <div>
        <div className="text-xs text-slate-400 mb-1">Preset</div>
        <select value={presetId ?? ''} onChange={e => onPreset(e.target.value)}
          className="w-full bg-white/5 border border-white/10 rounded px-2 py-1 text-sm outline-none">
          {!presetId && <option value="">custom</option>}
          {SCENARIOS.map(s => <option key={s.id} value={s.id} className="bg-slate-900">{s.title}</option>)}
        </select>
      </div>
      <div className="flex gap-1">
        {(['host', 'transport', 'cp', 'env'] as const).map(t => (
          <button key={t} onClick={() => setTab(t)}
            className={`chip ${tab === t ? 'bg-emerald-500/25 border-emerald-400/40 text-emerald-100' : ''}`}>{TAB_LABEL[t]}</button>
        ))}
      </div>
      <AnimatePresence mode="wait">
        <motion.div key={tab} {...slide} transition={{ duration: 0.18 }} className="flex flex-col gap-3">
          {tab === 'host' && <><AxisPicker axis="host" value={config.host} onChange={v => setConfig({ host: v })} /><AxisPicker axis="rpc" value={config.rpc} onChange={v => setConfig({ rpc: v })} /></>}
          {tab === 'transport' && <><AxisPicker axis="transport" value={config.transport} onChange={v => setConfig({ transport: v })} /><AxisPicker axis="rpc" value={config.rpc} onChange={v => setConfig({ rpc: v })} /></>}
          {tab === 'cp' && <AxisPicker axis="cp" value={config.cp} onChange={v => setConfig({ cp: v })} />}
          {tab === 'env' && <AxisPicker axis="env" value={config.env} onChange={v => setConfig({ env: v })} />}
        </motion.div>
      </AnimatePresence>
    </div>
  )
}

/* ── right pane (real): SW/HW plans + topology tiles. Toggling a tile filters and
   auto-populates the suite list in the Plan rail. ─────────────────────────── */
export function PlanConfig({ planOn, tileOn, benchOf, onPlan, onTile }: {
  planOn: Record<PlanId, boolean>
  tileOn: Record<PlanId, Record<Tag, boolean>>
  benchOf: (plan: PlanId, tag: Tag) => string
  onPlan: (p: PlanId, v: boolean) => void
  onTile: (p: PlanId, t: Tag, v: boolean) => void
}) {
  return (
    <div className="glass h-full w-full p-3 flex flex-col gap-3 overflow-auto">
      <div className="flex items-center justify-between">
        <span className="font-semibold">Plan</span>
        <span className="chip">SW / HW · topology</span>
      </div>
      {(['sw', 'hw'] as PlanId[]).map(p => (
        <div key={p} className={`rounded-lg border border-white/10 bg-white/5 ${planOn[p] ? '' : 'opacity-60'}`}>
          <div className="flex items-center gap-2 px-2 py-1.5">
            <TriCheck state={planOn[p] ? 'all' : 'none'} title={`include the ${p.toUpperCase()} plan`} onClick={() => onPlan(p, !planOn[p])} />
            <span className={`text-[10px] font-semibold px-1.5 rounded leading-tight ${PLAN_PILL[p]}`}>{p.toUpperCase()}</span>
            <span className="text-sm">{p === 'sw' ? 'Emulator' : 'Hardware'}</span>
            {p === 'hw' && <span className="text-[11px] text-slate-500 ml-auto">bench from Set-up</span>}
          </div>
          {planOn[p] && (
            <div className="px-2 pb-2 grid grid-cols-2 gap-1.5">
              {TAGS.map(tag => {
                const on = tileOn[p][tag], bench = benchOf(p, tag)
                return (
                  <div key={tag} title={TAG_LABEL[tag]}
                    className={`rounded-lg border px-2 py-1 flex flex-col gap-1 ${on ? 'border-emerald-400/30 bg-emerald-500/10' : 'border-white/10 bg-white/5 opacity-70'}`}>
                    <div className="flex items-center gap-1.5">
                      <TriCheck state={on ? 'all' : 'none'} title={`filter to ${TAG_LABEL[tag]}`} onClick={() => onTile(p, tag, !on)} />
                      <span className="text-xs">{TAG_LABEL[tag]}</span>
                    </div>
                    <span className={`text-[10px] ${bench ? 'text-slate-500' : 'text-slate-600'}`}>{bench || '—'}</span>
                  </div>
                )
              })}
            </div>
          )}
        </div>
      ))}
    </div>
  )
}

/* ── searchable code pane (sdkconfig); search + Enter/Shift+Enter ──────── */
const TIER: Record<string, string> = { base: 'text-slate-500', hosted: 'text-sky-300', default: 'text-amber-300' }
function classify(line: string, defaults: Set<string>): 'base' | 'hosted' | 'default' {
  const key = line.match(/^(CONFIG_[A-Z0-9_]+)/)?.[1]
  if (key && defaults.has(key)) return 'default'              // came from sdkconfig.defaults[.<chip>]
  if (key && key.startsWith('CONFIG_ESP_HOSTED')) return 'hosted'
  return 'base'
}
function SearchableCode({ title, path, text, defaults }: { title: string; path: string; text: string; defaults: string[] }) {
  const [q, setQ] = useState('')
  const [idx, setIdx] = useState(0)
  const marks = useRef<Record<number, HTMLElement | null>>({})
  const defSet = useMemo(() => new Set(defaults), [defaults])

  // precomputed once per (text, query): tier per line + search segments. Cheap
  // for a config file; no per-frame work, so resizing/scrolling stays smooth.
  const { lines, total } = useMemo(() => {
    const ql = q.toLowerCase(); let k = 0
    const out = text.split('\n').map(line => {
      const tier = classify(line, defSet)
      let segs: { t: string; m: boolean; k: number }[]
      if (!q) segs = [{ t: line, m: false, k: -1 }]
      else {
        segs = []; const low = line.toLowerCase(); let i = 0
        for (;;) {
          const j = low.indexOf(ql, i)
          if (j < 0) { segs.push({ t: line.slice(i), m: false, k: -1 }); break }
          if (j > i) segs.push({ t: line.slice(i, j), m: false, k: -1 })
          segs.push({ t: line.slice(j, j + q.length), m: true, k: k++ }); i = j + q.length
        }
      }
      return { tier, segs }
    })
    return { lines: out, total: k }
  }, [q, text, defSet])

  useEffect(() => { setIdx(0) }, [q])
  useEffect(() => { marks.current[idx]?.scrollIntoView({ block: 'center' }) }, [idx, q])
  const onKey = (e: React.KeyboardEvent) => {
    if (e.key !== 'Enter' || !total) return
    e.preventDefault(); setIdx(p => e.shiftKey ? (p - 1 + total) % total : (p + 1) % total)
  }
  return (
    <div className="flex-1 min-w-0 flex flex-col">
      <div className="flex items-baseline gap-2 mb-1">
        <span className="text-sm font-medium">{title}</span>
        <span className="text-[11px] text-slate-500 truncate" title={path}>{path}</span>
      </div>
      <div className="flex items-center gap-2 mb-1 text-[10px]">
        <span className="text-amber-300">● defaults</span><span className="text-sky-300">● hosted</span><span className="text-slate-500">● base</span>
        <input value={q} onChange={e => setQ(e.target.value)} onKeyDown={onKey} placeholder="search · Enter / Shift+Enter"
          className="flex-1 ml-auto bg-white/5 border border-white/10 rounded px-2 py-1 text-xs outline-none focus:border-emerald-400/50" />
        {q && <span className="text-slate-500 w-12 text-right">{total ? `${idx + 1}/${total}` : '0/0'}</span>}
      </div>
      <div className="flex-1 overflow-auto font-mono text-[12px] leading-relaxed bg-black/30 rounded p-2">
        {lines.map((ln, n) => (
          <div key={n} className={`${TIER[ln.tier]} whitespace-pre`}>
            {ln.segs.map((s, j) => s.m
              ? <mark key={j} ref={el => { marks.current[s.k] = el }} className={s.k === idx ? 'bg-amber-400 text-black' : 'bg-amber-400/40 text-black'}>{s.t}</mark>
              : <span key={j}>{s.t || ' '}</span>)}
          </div>
        ))}
      </div>
    </div>
  )
}

/* ── bottom drawer: logs (Host|CP side by side) · sdkconfig · report ───── */
export interface RunReport { id: string; start: string; end: string; durMs: number; results: RunResult[] }
export function Drawer({ kind, log, reports, viewId, onView, fitted, onToggleFit, onClose }: {
  kind: 'logs' | 'sdkconfig' | 'report'
  log: { build?: string[]; host: string[]; cp: string[] }
  reports: RunReport[]
  viewId: string | null; onView: (id: string) => void
  fitted: boolean; onToggleFit: () => void; onClose: () => void
}) {
  const [collapsed, setCollapsed] = useState<Record<string, boolean>>({})
  const LogCol = ({ title, path, lines }: { title: string; path: string; lines: string[] }) => (
    <div className="flex-1 min-w-0 flex flex-col">
      <div className="flex items-baseline gap-2 mb-1">
        <span className="text-sm font-medium">{title}</span>
        <span className="text-[11px] text-slate-500 truncate" title={path}>{path}</span>
      </div>
      <div className="flex-1 overflow-auto font-mono text-[12px] leading-relaxed bg-black/30 rounded p-2">
        {lines.length ? lines.map((l, i) =>
          <div key={i} className={l.startsWith('---') ? 'text-slate-500 my-1' : 'text-slate-300'}>{l}</div>)
          : <div className="text-slate-500">no log yet — Run a test</div>}
      </div>
    </div>
  )
  const title = kind === 'sdkconfig' ? 'sdkconfig' : kind === 'logs' ? 'Logs' : 'Report'
  return (
    <div className="glass h-full w-full p-4 flex flex-col rounded-b-none">
      <div className="flex items-center gap-3 mb-2">
        <span className="font-semibold">{title}</span>
        {kind === 'report' && reports.length > 0 && <span className="chip">{reports.length} run{reports.length > 1 ? 's' : ''}</span>}
        <div className="ml-auto flex gap-2">
          <button className="btn" onClick={onToggleFit} title="fit / restore — shrink panes above">{fitted ? '▣ restore' : '⤢ fit'}</button>
          <button className="btn" onClick={onClose}>close ✕</button>
        </div>
      </div>

      {kind === 'logs' && (
        <div className="flex-1 flex gap-3 min-h-0">
          <LogCol title="Build" path="idf.py build" lines={log.build || []} />
          <LogCol title="Host log" path={LOG_PATHS.host} lines={log.host} />
          <LogCol title="CP log" path={LOG_PATHS.cp} lines={log.cp} />
        </div>
      )}
      {kind === 'sdkconfig' && (
        <div className="flex-1 flex gap-3 min-h-0">
          <SearchableCode title="Host sdkconfig" path={SDKCONFIG_PATHS.host} text={HOST_SDKCONFIG} defaults={HOST_DEFAULTS} />
          <SearchableCode title="CP sdkconfig" path={SDKCONFIG_PATHS.cp} text={CP_SDKCONFIG} defaults={CP_DEFAULTS} />
        </div>
      )}
      {kind === 'report' && (reports.length === 0
        ? <div className="text-slate-500 text-sm">no run yet — Run a plan to see results</div>
        : <div className="flex-1 overflow-auto space-y-2">
            {reports.map((r, idx) => {
              const open = collapsed[r.id] ?? (idx === 0)   // latest expanded, older collapsed
              const passed = r.results.filter(x => x.status === 'pass').length
              const ok = passed === r.results.length
              return (
                <div key={r.id} className="rounded-lg border border-white/10">
                  <div onClick={() => setCollapsed(c => ({ ...c, [r.id]: !open }))}
                    className={`w-full flex items-center gap-2 px-3 py-2 cursor-pointer hover:bg-white/5 ${viewId === r.id ? 'bg-sky-500/10' : ''}`}>
                    <span className="text-slate-500 text-xs">{open ? '▾' : '▸'}</span>
                    <span className="font-medium text-sm">{r.id}</span>
                    {idx === 0 && <span className="chip text-ok">latest</span>}
                    <span className={`chip ${ok ? 'text-ok' : 'text-bad'}`}>{passed}/{r.results.length}</span>
                    <button onClick={e => { e.stopPropagation(); onView(r.id) }} title="show this run's sequence in the main pane"
                      className={`chip ${viewId === r.id ? 'bg-sky-500/30 border-sky-400/50 text-sky-100' : 'hover:bg-white/10'}`}>◧ view</button>
                    <span className="ml-auto text-xs text-slate-400">▶ {r.start} · ■ {r.end} · {(r.durMs / 1000).toFixed(1)}s</span>
                  </div>
                  {open && (
                    <table className="w-full text-sm px-3">
                      <tbody>
                        <tr className="border-y border-emerald-400/30"><td colSpan={3} className="px-3 py-1 text-ok text-xs">▶ started · {r.start}</td></tr>
                        {r.results.map(x => (
                          <tr key={x.id} className="border-t border-white/5">
                            <td className="px-3 py-1.5">{x.label}</td>
                            <td className={x.status === 'pass' ? 'text-ok' : x.status === 'fail' ? 'text-bad' : 'text-slate-500'}>
                              {x.status === 'pass' ? '✓ pass' : x.status === 'fail' ? '✗ fail' : 'skip'}</td>
                            <td className="px-3 text-right tabular-nums text-slate-400">{x.ms} ms</td>
                          </tr>
                        ))}
                        <tr className="border-y border-rose-400/30"><td colSpan={3} className="px-3 py-1 text-bad text-xs">■ ended · {r.end}</td></tr>
                      </tbody>
                    </table>
                  )}
                </div>
              )
            })}
          </div>)}
    </div>
  )
}
