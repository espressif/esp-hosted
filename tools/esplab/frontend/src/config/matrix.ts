// Single source of truth for the test matrix. The UI is generated from this —
// adding a target / RPC version / transport is a data edit here, not UI code.
// (Backend will later load the equivalent matrix.yaml; this mirrors it.)

export type AxisKey = 'env' | 'host' | 'cp' | 'transport' | 'rpc'

export interface Option { id: string; label: string }

export const AXES: Record<AxisKey, { label: string; options: Option[] }> = {
  env:       { label: 'Environment', options: [
    { id: 'esp_sim', label: 'esp-emu (sim)' },
    { id: 'hw',      label: 'real HW' },
  ]},
  host:      { label: 'Host', options: [
    { id: 'mcu_current',   label: 'MCU host (current)' },
    { id: 'mcu_old',       label: 'MCU host (old)' },
    { id: 'linux_current', label: 'Linux host (current)' },
    { id: 'linux_old',     label: 'Linux host (old)' },
  ]},
  cp:        { label: 'Co-processor', options: [
    { id: 'cp_current',      label: 'CP (current)' },
    { id: 'mcu_slave_old',   label: 'MCU slave (old)' },
    { id: 'linux_cp_legacy', label: 'Linux CP (legacy)' },
  ]},
  transport: { label: 'Transport', options: [
    { id: 'sdio', label: 'SDIO' },
    { id: 'spi',  label: 'SPI' },
    { id: 'uart', label: 'UART' },
  ]},
  rpc:       { label: 'RPC', options: [
    { id: 'v2', label: 'RPC v2' },
    { id: 'v1', label: 'RPC v1 (legacy)' },
  ]},
}

export interface ScenarioConfig {
  env: string; host: string; cp: string; transport: string; rpc: string
}

export const DEFAULT_CONFIG: ScenarioConfig = {
  env: 'esp_sim', host: 'mcu_current', cp: 'cp_current', transport: 'sdio', rpc: 'v2',
}

// Compatibility rules → enabled/disabled + human reason. Extend freely.
type Rule = { when: Partial<ScenarioConfig>; reason: string }
const DISABLED_RULES: Rule[] = [
  { when: { host: 'linux_old', rpc: 'v2' }, reason: 'Old Linux host has no RPC v2 support' },
  { when: { host: 'mcu_old', rpc: 'v2' },   reason: 'Old MCU host predates RPC v2' },
  { when: { cp: 'linux_cp_legacy', transport: 'sdio' }, reason: 'Legacy Linux CP is UART/SPI only' },
  { when: { env: 'esp_sim', cp: 'linux_cp_legacy' },    reason: 'No sim image for legacy Linux CP' },
  { when: { env: 'esp_sim', transport: 'uart' },        reason: 'UART transport not modeled in sim yet' },
]

export function evalCompat(c: ScenarioConfig): { enabled: boolean; reason?: string } {
  for (const r of DISABLED_RULES) {
    if (Object.entries(r.when).every(([k, v]) => (c as any)[k] === v)) {
      return { enabled: false, reason: r.reason }
    }
  }
  return { enabled: true }
}

// Test-case catalog. A run executes an ordered subset of these.
export interface TestCase { id: string; label: string; group: string }
export const TEST_CASES: TestCase[] = [
  { id: 'bringup',     label: 'Transport bring-up',          group: 'Core' },
  { id: 'fw_version',  label: 'Get CP firmware version',     group: 'Core' },
  { id: 'wifi_sta',    label: 'Wi-Fi STA connect + DHCP',    group: 'Wi-Fi' },
  { id: 'wifi_scan',   label: 'Wi-Fi scan',                  group: 'Wi-Fi' },
  { id: 'throughput',  label: 'Throughput (iperf)',          group: 'Wi-Fi' },
  { id: 'softap',      label: 'SoftAP + client join',        group: 'Wi-Fi' },
  { id: 'heartbeat',   label: 'Heartbeat monitor',           group: 'Recovery' },
  { id: 'cp_reset',    label: 'CP reset → host reinit',      group: 'Recovery' },
  { id: 'reconnect',   label: 'Reconnect after recovery',    group: 'Recovery' },
  { id: 'rpc_echo',    label: 'RPC peer-data echo',          group: 'RPC' },
  { id: 'ble',         label: 'BLE advertise / connect',     group: 'BLE' },
  { id: 'power_save',  label: 'Power save + wake',           group: 'Power' },
]
export const caseLabel = (id: string) => TEST_CASES.find(c => c.id === id)?.label ?? id

// Suites = the test infrastructure's predefined groupings of cases. The plan is
// built by including cases from suites; suites are amendable in the UI.
export interface Suite { id: string; label: string; cases: string[] }
export const SUITES: Suite[] = [
  { id: 'core',     label: 'Core bring-up', cases: ['bringup', 'fw_version'] },
  { id: 'wifi',     label: 'Wi-Fi',         cases: ['wifi_sta', 'wifi_scan', 'throughput', 'softap'] },
  { id: 'recovery', label: 'Recovery',      cases: ['heartbeat', 'cp_reset', 'reconnect'] },
  { id: 'rpc',      label: 'RPC',           cases: ['rpc_echo'] },
  { id: 'ble',      label: 'Bluetooth',     cases: ['ble'] },
  { id: 'power',    label: 'Power save',    cases: ['power_save'] },
]
// default plan auto-selected on boot
export const DEFAULT_SELECTED = ['bringup', 'fw_version', 'wifi_sta', 'heartbeat', 'cp_reset', 'reconnect']

// Curated named scenarios (auto-population can expand the full cross-product
// later). Each carries a default ordered test-case list.
export interface Scenario { id: string; group: string; title: string; config: ScenarioConfig; cases: string[] }
export const SCENARIOS: Scenario[] = [
  { id: 'quick',        group: 'Quick start', title: 'Current CP + MCU host', config: DEFAULT_CONFIG,
    cases: ['bringup', 'fw_version', 'wifi_sta'] },
  { id: 'recovery',     group: 'Recovery',    title: 'CP reset recovery',     config: { ...DEFAULT_CONFIG },
    cases: ['bringup', 'wifi_sta', 'heartbeat', 'cp_reset', 'reconnect'] },
  { id: 'rpc_v1',       group: 'RPC compat',  title: 'Current CP (v1) + old Linux host', config: { env:'hw', host:'linux_old', cp:'cp_current', transport:'spi', rpc:'v1' },
    cases: ['bringup', 'fw_version', 'rpc_echo'] },
  { id: 'linux_legacy', group: 'RPC compat',  title: 'Legacy Linux CP + old Linux host', config: { env:'hw', host:'linux_old', cp:'linux_cp_legacy', transport:'uart', rpc:'v1' },
    cases: ['bringup', 'rpc_echo'] },
  { id: 'wifi_sta',     group: 'Wi-Fi',       title: 'Wi-Fi STA + throughput', config: DEFAULT_CONFIG,
    cases: ['bringup', 'wifi_sta', 'wifi_scan', 'throughput'] },
]

export function optLabel(axis: AxisKey, id: string): string {
  return AXES[axis].options.find(o => o.id === id)?.label ?? id
}
