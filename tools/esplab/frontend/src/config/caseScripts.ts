// Each test case is a time-ordered host <-> CP message exchange. The sequence
// diagram is generated from these as the run executes — participants (AP, BLE
// peer, client) appear ONLY when a message to them occurs (event-driven).
// The backend will later emit the same messages live; this is the mock script.

import { caseLabel } from './matrix'

export interface Msg { from: string; to: string; text: string; resp?: boolean; note?: boolean }

export const CASE_STEPS: Record<string, Msg[]> = {
  bringup: [
    { from: 'Host', to: 'CP', text: 'SDIO card init' },
    { from: 'CP', to: 'Host', text: 'INIT event · TRANSPORT_UP', resp: true },
  ],
  fw_version: [
    { from: 'Host', to: 'CP', text: 'GetFwVersion' },
    { from: 'CP', to: 'Host', text: 'FW 3.0.0', resp: true },
  ],
  wifi_sta: [
    { from: 'Host', to: 'CP', text: 'WifiConnect Yogesh' },
    { from: 'CP', to: 'AP', text: 'assoc request' },
    { from: 'AP', to: 'CP', text: 'assoc ok', resp: true },
    { from: 'CP', to: 'Host', text: 'StaConnected · GotIP', resp: true },
  ],
  wifi_scan: [
    { from: 'Host', to: 'CP', text: 'WifiScan' },
    { from: 'CP', to: 'Host', text: 'ScanDone (7 APs)', resp: true },
  ],
  softap: [
    { from: 'Host', to: 'CP', text: 'SoftAP start' },
    { from: 'Client', to: 'CP', text: 'join' },
    { from: 'CP', to: 'Host', text: 'AP_StaConnected', resp: true },
  ],
  throughput: [
    { from: 'Host', to: 'CP', text: 'iperf TX' },
    { from: 'CP', to: 'Host', text: '42 Mbps', resp: true },
  ],
  heartbeat: [
    { from: 'CP', to: 'Host', text: 'Heartbeat 1', resp: true },
    { from: 'CP', to: 'Host', text: 'Heartbeat 2', resp: true },
  ],
  cp_reset: [
    { from: '', to: 'CP', text: 'co-processor reset', note: true },
    { from: 'Host', to: 'CP', text: 'RESET (GPIO)' },
    { from: 'CP', to: 'Host', text: 'INIT (rebooted)', resp: true },
  ],
  reconnect: [
    { from: 'Host', to: 'CP', text: 'WifiConnect' },
    { from: 'CP', to: 'Host', text: 'GotIP', resp: true },
  ],
  rpc_echo: [
    { from: 'Host', to: 'CP', text: 'CAT' },
    { from: 'CP', to: 'Host', text: 'MEOW', resp: true },
  ],
  ble: [
    { from: 'Host', to: 'CP', text: 'BLE advertise' },
    { from: 'Peer', to: 'CP', text: 'connect' },
    { from: 'CP', to: 'Host', text: 'BLE connected', resp: true },
  ],
  power_save: [
    { from: 'Host', to: 'CP', text: 'PS enter' },
    { from: '', to: 'CP', text: 'sleep', note: true },
    { from: 'Host', to: 'CP', text: 'WAKE (GPIO)' },
    { from: 'CP', to: 'Host', text: 'awake', resp: true },
  ],
}

const RECT: Record<string, string> = {
  run: 'rgba(56,189,248,0.14)', pass: 'rgba(52,211,153,0.13)', fail: 'rgba(251,113,133,0.14)', idle: 'rgba(148,163,184,0.07)',
}

// A captured block of a run: the messages that actually occurred for a case.
// (Mock fills msgs from CASE_STEPS; the backend will capture real events here.)
export interface SeqBlock { id: string; label: string; status: 'idle' | 'run' | 'pass' | 'fail'; msgs: Msg[] }

function render(blocks: SeqBlock[], outline: boolean): string {
  const L = ['sequenceDiagram', 'participant Host', 'participant CP']
  let any = false
  for (const b of blocks) {
    if (outline) { L.push(`Note over Host,CP: ${b.label}`); any = true; continue }
    any = true
    L.push(`rect ${RECT[b.status] || RECT.idle}`)
    L.push(`Note over Host,CP: ${b.label}${b.status === 'pass' ? '  ✓' : b.status === 'fail' ? '  ✗' : ''}`)
    for (const m of b.msgs) {
      if (m.note) L.push(`Note over ${m.to}: ${m.text}`)
      else L.push(`${m.from}${m.resp ? '-->>' : '->>'}${m.to}: ${m.text}`)
    }
    L.push('end')
  }
  if (!any) L.push('Note over Host,CP: press Run to execute')
  return L.join('\n')
}

// Capture blocks for a set of cases (used live + to snapshot a run into a report).
export function blocksFor(cases: string[], status: Record<string, string>): SeqBlock[] {
  return cases.map(id => ({ id, label: caseLabel(id), status: (status[id] as SeqBlock['status']) || 'idle', msgs: CASE_STEPS[id] || [] }))
}

// Live diagram: idle = outline of selected cases; running = revealed per started case.
export function buildMermaid(cases: string[], status: Record<string, string>, running: boolean): string {
  if (!running) return render(blocksFor(cases, status), true)
  return render(blocksFor(cases, status).filter(b => b.status !== 'idle'), false)
}

// Replay a captured run's diagram — built dynamically from the run's own blocks.
export function buildMermaidFromBlocks(blocks: SeqBlock[]): string {
  return render(blocks, false)
}

// Host / CP log lines derived from started cases (logs are NOT config).
export function buildLogs(cases: string[], status: Record<string, string>) {
  const host: string[] = [], cp: string[] = []
  for (const id of cases) {
    const st = status[id]
    if (!st || st === 'idle') continue
    for (const m of CASE_STEPS[id] || []) {
      const line = `[${id}] ${m.text}`
      if (m.from === 'Host') host.push(line); else cp.push(line)
    }
  }
  return { host, cp }
}
