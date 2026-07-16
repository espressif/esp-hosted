// 1:1 mapping between CLI switches and UI config. The launcher (esplab.py)
// turns switches into URL query params; the UI reads them here. Anything
// clickable in the UI is therefore also settable from the command line.
//
//   --env --host --cp --transport --rpc --scenario --flash --no-reset
//   --theme --mock --headless   (see esplab.py for the full surface)

import { DEFAULT_CONFIG, ScenarioConfig, SCENARIOS } from './matrix'

export interface AppConfig {
  scenario: ScenarioConfig
  scenarioId?: string
  flash: 'full' | 'off'
  reset: boolean         // host resets CP after flashing (default true)
  theme: 'dark' | 'light'
  mock: boolean
}

const QUERY_TO_AXIS: Record<string, keyof ScenarioConfig> = {
  env: 'env', host: 'host', cp: 'cp', transport: 'transport', rpc: 'rpc',
}

export function configFromQuery(search = window.location.search): AppConfig {
  const q = new URLSearchParams(search)
  let scenario: ScenarioConfig = { ...DEFAULT_CONFIG }
  let scenarioId: string | undefined

  const sid = q.get('scenario')
  if (sid) {
    const s = SCENARIOS.find(s => s.id === sid)
    if (s) { scenario = { ...s.config }; scenarioId = s.id }
  }
  for (const [param, axis] of Object.entries(QUERY_TO_AXIS)) {
    const v = q.get(param)
    if (v) scenario[axis] = v
  }
  return {
    scenario,
    scenarioId,
    flash: (q.get('flash') as AppConfig['flash']) || 'full',
    reset: q.get('no-reset') === null,        // present => false
    theme: (q.get('theme') as AppConfig['theme']) || 'dark',
    mock: q.get('mock') !== '0',              // mock-first by default
  }
}
