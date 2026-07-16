import { useEffect, useMemo, useRef, useState } from 'react'

/* ESP-Hosted test console (real, emulator-backed).
   Renders the Suites ▸ Scenarios ▸ Cases entity tree from /api/v1/tests and
   drives the runner via /api/v1/run. Every element is an entity differentiated
   only by its props (kind / category / support) — so the same row renders a
   positive functional test, a negative robustness test, or an unsupported one. */

type Status = 'pending' | 'building' | 'emulating' | 'running' | 'pass' | 'fail' | 'skip' | 'queued'
interface Phase { status: string; detail: string }
interface Case { id: string; label: string; kind: string; status: string; detail: string }
interface CatScenario {
  id: string; label: string; kind: string; category: string; support: string
  setup: string; cases: { id: string; label: string; kind?: string }[]
}
interface Suite { id: string; label: string; scenarios: CatScenario[] }
interface RunScenario {
  id: string; status: string; detail: string; dur_s: number | null
  phases: { build_cp: Phase; build_host: Phase; emulate: Phase }
  cases: Case[]; logs: { build: string; host: string; cp: string }
}
interface RunState {
  id: string; status: string
  summary: { pass: number; fail: number; skip: number; total: number }
  scenarios: RunScenario[]
}

const DOT: Record<string, string> = {
  pass: 'bg-emerald-400', fail: 'bg-rose-400', skip: 'bg-slate-500',
  running: 'bg-sky-400', building: 'bg-sky-400', emulating: 'bg-sky-400',
  pending: 'bg-white/25', queued: 'bg-white/25',
}
const TONE: Record<string, string> = { pass: 'text-ok', fail: 'text-bad', skip: 'text-slate-400' }
const PHASE_LABEL: Record<string, string> = { build_cp: 'CP', build_host: 'Host', emulate: 'Run' }
const pulse = { animation: 'hbpulse 1s ease-in-out infinite' }

function Dot({ status }: { status: string }) {
  return <span className={`inline-block h-2.5 w-2.5 rounded-full ${DOT[status] || 'bg-white/25'}`}
    style={status === 'building' || status === 'emulating' || status === 'running' ? pulse : undefined} />
}
function PhasePip({ p, label }: { p: Phase; label: string }) {
  return (
    <span className="inline-flex items-center gap-1" title={`${label}: ${p.status}${p.detail ? ' · ' + p.detail : ''}`}>
      <span className={`inline-block h-2 w-2 rounded-full ${DOT[p.status] || 'bg-white/25'}`}
        style={p.status === 'running' ? pulse : undefined} />
      <span className="text-[10px] text-slate-500">{label}</span>
    </span>
  )
}
function supportTag(support: string) {
  if (support === 'supported') return null
  const [kind, reason] = support.split(/:(.+)/)
  const cls = kind === 'tbd' ? 'text-sky-300' : 'text-warn'
  return <span className={`chip !text-[10px] ${cls}`} title={reason || support}>{kind}</span>
}

export default function ExamplesPage() {
  const [suites, setSuites] = useState<Suite[]>([])
  const [preflight, setPreflight] = useState<string[]>([])
  const [targets, setTargets] = useState({ cp: 'esp32c6', host: 'esp32p4' })
  const [sel, setSel] = useState<Record<string, boolean>>({})
  const [clean, setClean] = useState(true)
  const [sandbox, setSandbox] = useState(false)
  const [collapsed, setCollapsed] = useState<Record<string, boolean>>({})
  const [run, setRun] = useState<RunState | null>(null)
  const [runId, setRunId] = useState<string | null>(null)
  const [detail, setDetail] = useState<string | null>(null)
  const [logTab, setLogTab] = useState<'host' | 'cp' | 'build'>('host')
  const [err, setErr] = useState<string | null>(null)
  const poll = useRef<number | null>(null)

  useEffect(() => {
    fetch('/api/v1/tests').then(r => r.json()).then(d => {
      setSuites(d.suites); setPreflight(d.preflight || [])
      setTargets({ cp: d.targets.cp, host: d.targets.host })
      // boot collapsed — open suites on demand
      setCollapsed(Object.fromEntries(d.suites.map((s: Suite) => [s.id, true])))
    }).catch(() => setErr('backend unreachable — run ./esplab.py --serve'))
  }, [])

  const running = run?.status === 'running' || run?.status === 'queued'
  useEffect(() => {
    if (!runId) return
    const tick = () => fetch(`/api/v1/run/${runId}`).then(r => r.json()).then((d: RunState) => {
      setRun(d)
      if (d.status === 'done' || d.status === 'stopped') { if (poll.current) clearInterval(poll.current); poll.current = null }
    }).catch(() => {})
    tick(); poll.current = window.setInterval(tick, 1500)
    return () => { if (poll.current) clearInterval(poll.current) }
  }, [runId])

  const byId = useMemo(() => {
    const m: Record<string, RunScenario> = {}
    run?.scenarios.forEach(s => (m[s.id] = s))
    return m
  }, [run])

  const allScenarios = useMemo(() => suites.flatMap(s => s.scenarios), [suites])
  const supported = allScenarios.filter(s => s.support === 'supported')
  const selectedIds = allScenarios.filter(s => sel[s.id]).map(s => s.id)
  const detailRun = detail ? byId[detail] : null
  const detailCat = detail ? allScenarios.find(s => s.id === detail) : null

  const startRun = (ids: string[] | null) => {
    setErr(null); setRun(null)
    fetch('/api/v1/run', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ids, clean, includeUnsupported: !!ids, sandbox: sandbox ? 'bwrap' : 'none' }),
    }).then(r => r.json()).then(d => { d.error ? setErr(d.error) : setRunId(d.runId) }).catch(() => setErr('failed to start run'))
  }
  const stop = () => { if (runId) fetch(`/api/v1/run/${runId}/stop`, { method: 'POST' }).catch(() => {}) }
  const st = (id: string): string => byId[id]?.status || 'pending'

  return (
    <div className="glass h-full w-full p-4 flex flex-col gap-3 overflow-hidden">
      {/* header + summary + controls */}
      <div className="flex items-center gap-3 flex-wrap">
        <span className="font-semibold">Test console</span>
        <span className="chip">{allScenarios.length} scenarios</span>
        <span className="chip text-ok">{supported.length} supported</span>
        <span className="text-[11px] text-slate-500">C6 {targets.cp} ⇄ P4 {targets.host} · SDIO bench</span>
        {run && (
          <div className="flex items-center gap-1.5">
            <span className="chip text-ok">✓ {run.summary.pass}</span>
            <span className="chip text-bad">✗ {run.summary.fail}</span>
            <span className="chip text-slate-400">– {run.summary.skip}</span>
            <span className="chip">{run.summary.pass + run.summary.fail}/{run.summary.total}</span>
            <span className={`chip ${running ? 'text-sky-300' : run.status === 'done' ? 'text-ok' : 'text-warn'}`}>{run.status}</span>
          </div>
        )}
        <div className="ml-auto flex items-center gap-2">
          <label className="flex items-center gap-1 text-xs text-slate-400" title="fullclean before each build (hermetic, slower)">
            <input type="checkbox" checked={clean} onChange={e => setClean(e.target.checked)} className="accent-emerald-400" disabled={running} /> clean
          </label>
          <label className="flex items-center gap-1 text-xs text-slate-400" title="isolate build+emulate in a bubblewrap sandbox so the main system stays untouched (falls back to native if unavailable)">
            <input type="checkbox" checked={sandbox} onChange={e => setSandbox(e.target.checked)} className="accent-emerald-400" disabled={running} /> sandbox
          </label>
          <button className="btn !px-2 !py-1 text-xs" disabled={running || !selectedIds.length} onClick={() => startRun(selectedIds)}
            title="run the ticked scenarios (forces unsupported if ticked)">▶ Selected ({selectedIds.length})</button>
          <button className="btn btn-primary !px-3 !py-1 text-sm" disabled={running} onClick={() => startRun(null)}
            title="run every supported scenario">▶ Run supported</button>
          <button className="btn !px-3 !py-1 text-sm" disabled={!running} onClick={stop}>■ Stop</button>
        </div>
      </div>

      {preflight.length > 0 && (
        <div className="rounded-lg border border-amber-400/40 bg-amber-500/10 px-3 py-2 text-xs text-amber-200">⚠ preflight: {preflight.join(' · ')}</div>
      )}
      {err && <div className="rounded-lg border border-rose-400/40 bg-rose-500/10 px-3 py-2 text-xs text-bad">{err}</div>}

      {/* suite ▸ scenario tree */}
      <div className="flex-1 overflow-auto -mx-1 px-1 space-y-2">
        {suites.map(suite => {
          const open = !collapsed[suite.id]
          const scs = suite.scenarios
          const done = scs.filter(s => ['pass', 'fail', 'skip'].includes(st(s.id)))
          const passed = scs.filter(s => st(s.id) === 'pass').length
          return (
            <div key={suite.id} className="rounded-lg border border-white/10 bg-white/5">
              <div className="flex items-center gap-2 px-3 py-1.5 cursor-pointer hover:bg-white/5"
                onClick={() => setCollapsed(c => ({ ...c, [suite.id]: open }))}>
                <span className="text-slate-500 text-xs">{open ? '▾' : '▸'}</span>
                <span className="text-sm font-medium">{suite.label}</span>
                <span className="chip !text-[10px]">{scs.length}</span>
                {run && done.length > 0 && <span className={`chip !text-[10px] ${passed === scs.length ? 'text-ok' : 'text-slate-400'}`}>{passed}/{scs.length}</span>}
              </div>
              {open && (
                <div className="px-2 pb-2 grid gap-1.5" style={{ gridTemplateColumns: 'repeat(auto-fill, minmax(320px, 1fr))' }}>
                  {scs.map(sc => {
                    const r = byId[sc.id]
                    const status = st(sc.id)
                    return (
                      <div key={sc.id}
                        className={`rounded-lg border bg-white/5 px-2.5 py-2 flex flex-col gap-1.5 cursor-pointer hover:bg-white/10
                          ${detail === sc.id ? 'border-sky-400/50' : 'border-white/10'}`}
                        onClick={() => { setDetail(sc.id); setLogTab('host') }}>
                        <div className="flex items-center gap-2">
                          <Dot status={status} />
                          <span className="text-sm flex-1 truncate" title={sc.label}>{sc.label}</span>
                          <span className={`chip !text-[10px] ${sc.kind === 'negative' ? 'text-warn' : 'text-slate-400'}`}>{sc.kind === 'negative' ? 'neg' : 'pos'}</span>
                          {supportTag(sc.support)}
                          <input type="checkbox" checked={!!sel[sc.id]} disabled={running}
                            onClick={e => e.stopPropagation()} onChange={e => setSel(s => ({ ...s, [sc.id]: e.target.checked }))}
                            className="accent-emerald-400" />
                        </div>
                        <div className="flex items-center gap-2.5">
                          {(['build_cp', 'build_host', 'emulate'] as const).map(k => (
                            <PhasePip key={k} label={PHASE_LABEL[k]} p={r?.phases[k] || { status: 'pending', detail: '' }} />
                          ))}
                          <span className="flex items-center gap-1 ml-auto">
                            {(r?.cases || sc.cases.map(c => ({ ...c, status: 'pending', detail: '' }))).map(c => (
                              <span key={c.id} title={`${c.label}: ${(c as Case).status || 'pending'}${(c as Case).detail ? ' · ' + (c as Case).detail : ''}`}
                                className={`inline-block h-1.5 w-1.5 rounded-full ${DOT[(c as Case).status] || 'bg-white/25'}`} />
                            ))}
                          </span>
                        </div>
                        <div className="flex items-center gap-2 min-h-[14px]">
                          {r && r.status !== 'pending' && (
                            <span className={`text-[11px] truncate ${TONE[r.status] || 'text-slate-400'}`} title={r.detail}>{r.detail || r.status}</span>
                          )}
                          {r?.dur_s != null && <span className="ml-auto text-[10px] text-slate-500 tabular-nums">{r.dur_s}s</span>}
                        </div>
                      </div>
                    )
                  })}
                </div>
              )}
            </div>
          )
        })}
      </div>

      {/* scenario detail: cases + logs */}
      {detailCat && (
        <div className="rounded-lg border border-white/10 bg-black/20 p-3 flex flex-col" style={{ height: '36%' }}>
          <div className="flex items-center gap-2 mb-2 flex-wrap">
            <span className="font-medium text-sm">{detailCat.label}</span>
            <span className="chip !text-[10px]">{detailCat.kind}</span>
            <span className="chip !text-[10px]">{detailCat.category}</span>
            {detailRun && <span className={`chip ${TONE[detailRun.status] || 'text-slate-400'}`}>{detailRun.status}</span>}
            {(['host', 'cp', 'build'] as const).map(t => (
              <button key={t} onClick={() => setLogTab(t)}
                className={`chip ${logTab === t ? 'bg-emerald-500/25 border-emerald-400/40 text-emerald-100' : ''}`}>{t}</button>
            ))}
            <button className="btn ml-auto" onClick={() => setDetail(null)}>close ✕</button>
          </div>
          {/* cases */}
          <div className="flex flex-wrap gap-1.5 mb-2">
            {(detailRun?.cases || detailCat.cases.map(c => ({ ...c, kind: c.kind || detailCat.kind, status: 'pending', detail: '' }))).map(c => (
              <span key={c.id} className={`chip !text-[10px] ${TONE[(c as Case).status] || 'text-slate-400'}`}
                title={(c as Case).detail || c.label}>
                {(c as Case).status === 'pass' ? '✓' : (c as Case).status === 'fail' ? '✗' : '·'} {c.label}
              </span>
            ))}
          </div>
          {detailCat.support !== 'supported' && (
            <div className="text-[11px] text-amber-200/80 mb-1">⚠ {detailCat.support}</div>
          )}
          <div className="flex-1 overflow-auto font-mono text-[12px] leading-relaxed bg-black/30 rounded p-2 whitespace-pre-wrap">
            {detailRun?.logs[logTab] || <span className="text-slate-500">no {logTab} output yet — run this scenario</span>}
          </div>
        </div>
      )}
    </div>
  )
}
