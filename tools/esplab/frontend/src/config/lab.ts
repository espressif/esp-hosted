// Device-layer model for the Set-up page. Mock now (authoring ahead of
// hardware) but shaped to donut's `device.list-ok` wire frame, so swapping to a
// live `donut ls` is a data-source change, not a UI change.

export type LabMode = 'donut' | 'local'
export type DevKind = 'host' | 'cp' | 'helper'

export interface LabDevice {
  id: string
  name: string
  chip: string                     // device type name (host / cp / sniffer / …)
  tags: string[]
  addr: string                     // cp/helper: serial path or rfc2217://
  ip?: string                      // host: runtime IP
  fields?: Record<string, string>  // values for the type's custom fields
  hands?: string                   // donut hands box; undefined in local mode
  status: 'online' | 'leased' | 'offline'
}

// A Set-up entry: the host↔cp pairing (drives the run) plus any attached
// member devices (any type, multiple allowed) that participate in the mapping.
export interface Link { id: string; name: string; hostId?: string; cpId?: string; transport: string; memberIds?: string[] }

// A device type is a schema: base fields are implicit (name/type/status/tags/
// address); `fields` are user-added extras (string or combo-with-entries).
export interface TypeField { name: string; kind: 'string' | 'combo'; options: string[] }
export interface DeviceType { name: string; color?: string; fields: TypeField[] }

// Link transports the host↔cp pairing can use — wired (sdio/spi/uart) or
// wireless (bluetooth/sta/softap/thread/zigbee). User-configurable.
export interface LinkType { name: string; wireless: boolean }

export interface LabOpts { flashBeforeRun: boolean; chipGuard: boolean; baud: number }

export interface LabState {
  mode: LabMode
  hub: { url: string; connected: boolean }
  hosts: LabDevice[]
  cps: LabDevice[]
  helpers: LabDevice[]
  deviceTypes: DeviceType[]
  linkTypes: LinkType[]
  links: Link[]
  activeLinkId?: string
  opts: LabOpts
}

export const BAUDS = [115200, 460800, 921600, 1500000, 2000000]
export const TRANSPORTS = ['sdio', 'spi', 'uart']

const DONUT_HOSTS: LabDevice[] = [
  { id: 'd_p4imx', name: 'esp32p4-host-7cdf', chip: 'host', tags: ['imx'], ip: '192.168.1.10',
    addr: '/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1.2:1.0', hands: 'lab-imx', status: 'online' },
  { id: 'd_p4rpi', name: 'esp32p4-host-1be2', chip: 'host', tags: ['rpi'], ip: '192.168.1.11',
    addr: '/dev/serial/by-path/platform-fe9c0000.usb-usb-0:1.3:1.0', hands: 'lab-rpi', status: 'online' },
]
const DONUT_CPS: LabDevice[] = [
  { id: 'd_c5imx', name: 'esp32c5-imx-3c11', chip: 'cp', tags: ['imx', 'c5'],
    addr: '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_3c11-if00', hands: 'lab-imx', status: 'online' },
  { id: 'd_c5rpi', name: 'esp32c5-rpi-9a04', chip: 'cp', tags: ['rpi', 'c5'],
    addr: 'rfc2217://lab-rpi:46211?ign_set_control', hands: 'lab-rpi', status: 'leased' },
]
const LOCAL_HOSTS: LabDevice[] = [
  { id: 'l_p4', name: 'esp32p4 (port 1.2)', chip: 'host', tags: [], ip: '192.168.1.20',
    addr: '/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2:1.0', status: 'online' },
]
const LOCAL_CPS: LabDevice[] = [
  { id: 'l_c5', name: 'esp32c5 (usb-jtag)', chip: 'cp', tags: [],
    addr: '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_9a04-if00', status: 'online' },
]
const DONUT_HELPERS: LabDevice[] = [
  { id: 'd_relay', name: 'usb-relay-imx', chip: 'relay', tags: ['power', 'imx'],
    addr: '/dev/serial/by-id/usb-FTDI_relay_A1', hands: 'lab-imx', status: 'online' },
  { id: 'd_sniffer', name: 'wifi-sniffer-1', chip: 'sniffer', tags: ['80211', 'monitor'],
    addr: '/dev/serial/by-id/usb-Sniffer_X-if00', hands: 'lab-imx', status: 'online' },
  { id: 'd_ap', name: 'ap-backend-1', chip: 'ap-backend', tags: ['ap', 'dhcp'], fields: { ssid: 'eh-lab', band: '5G' },
    addr: 'ssh://ap-backend.lab:22', status: 'online' },
  { id: 'd_hub', name: 'usb-hub-imx', chip: 'usb-hub', tags: ['hub'],
    addr: '/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1', hands: 'lab-imx', status: 'online' },
]
const LOCAL_HELPERS: LabDevice[] = []

export const DEFAULT_TYPES: DeviceType[] = [
  { name: 'host', color: 'teal', fields: [] },
  { name: 'cp', color: 'violet', fields: [] },
  { name: 'setup', color: 'fuchsia', fields: [] },
  { name: 'sniffer', color: 'cyan', fields: [] },
  { name: 'ap-backend', color: 'amber', fields: [
    { name: 'ssid', kind: 'string', options: [] },
    { name: 'band', kind: 'combo', options: ['2.4G', '5G', '6G'] },
  ] },
  { name: 'usb-hub', color: 'lime', fields: [] },
  { name: 'relay', color: 'rose', fields: [] },
  { name: 'power', color: 'orange', fields: [] },
]

export function catalog(mode: LabMode): { hosts: LabDevice[]; cps: LabDevice[]; helpers: LabDevice[] } {
  return mode === 'donut'
    ? { hosts: DONUT_HOSTS, cps: DONUT_CPS, helpers: DONUT_HELPERS }
    : { hosts: LOCAL_HOSTS, cps: LOCAL_CPS, helpers: LOCAL_HELPERS }
}

export const DEFAULT_LAB: LabState = {
  mode: 'donut',
  hub: { url: 'wss://donut.espressif.tools', connected: true },
  hosts: DONUT_HOSTS,
  cps: DONUT_CPS,
  helpers: DONUT_HELPERS,
  deviceTypes: DEFAULT_TYPES,
  linkTypes: [
    { name: 'sdio', wireless: false }, { name: 'spi', wireless: false }, { name: 'uart', wireless: false },
    { name: 'bluetooth', wireless: true }, { name: 'sta', wireless: true }, { name: 'softap', wireless: true },
    { name: 'thread', wireless: true }, { name: 'zigbee', wireless: true },
  ],
  links: [
    { id: 'k_imx', name: 'iMX bench', hostId: 'd_p4imx', cpId: 'd_c5imx', transport: 'sdio', memberIds: ['d_relay'] },
    { id: 'k_rpi', name: 'RPi bench', hostId: 'd_p4rpi', cpId: 'd_c5rpi', transport: 'sdio', memberIds: [] },
  ],
  activeLinkId: undefined,
  opts: { flashBeforeRun: true, chipGuard: true, baud: 460800 },
}

export const activeLink = (lab: LabState) => lab.links.find(l => l.id === lab.activeLinkId)
export const hostById = (lab: LabState, id?: string) => lab.hosts.find(d => d.id === id)
export const cpById = (lab: LabState, id?: string) => lab.cps.find(d => d.id === id)
export const typeByName = (lab: LabState, name: string) => lab.deviceTypes.find(t => t.name === name)
export const typeNames = (lab: LabState) => lab.deviceTypes.map(t => t.name)
export const colorMap = (lab: LabState): Record<string, string> =>
  Object.fromEntries(lab.deviceTypes.map(t => [t.name, t.color ?? '']))

export function labValid(lab: LabState): { ok: boolean; reason?: string } {
  const k = activeLink(lab)
  if (!k) return { ok: false, reason: 'select a Set-up to run' }
  if (!hostById(lab, k.hostId) || !cpById(lab, k.cpId)) return { ok: false, reason: 'Set-up needs a Host and a Co-processor' }
  return { ok: true }
}

export function isLabState(o: unknown): o is LabState {
  const x = o as LabState
  return !!x && Array.isArray(x.hosts) && Array.isArray(x.cps) && Array.isArray(x.helpers)
    && Array.isArray(x.deviceTypes) && (x.deviceTypes.length === 0 || typeof x.deviceTypes[0] === 'object')
    && Array.isArray(x.linkTypes) && Array.isArray(x.links) && !!x.opts && !!x.hub
}
