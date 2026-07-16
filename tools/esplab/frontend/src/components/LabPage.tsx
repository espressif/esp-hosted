import { Fragment, useEffect, useState } from 'react'
import { Panel, PanelGroup, PanelResizeHandle } from 'react-resizable-panels'
import {
  BAUDS, catalog, colorMap, DEFAULT_LAB, DeviceType, DevKind, LabDevice, LabState, Link, LinkType, TypeField,
} from '../config/lab'

// Consistent highlight language, no colour change to the tile itself:
// clicked entity = soft fading ring; related entities (any-to-any) = thin dashes.
const DOT_SEL = 'ring-2 ring-slate-100/70'
const DOT_REL = 'outline outline-1 outline-dashed outline-slate-300/70'

export type LabCol = { cp: boolean; host: boolean; help: boolean; types: boolean; setup: boolean }
export const DEFAULT_LAB_COL: LabCol = { cp: false, host: false, help: true, types: true, setup: false }

const DOT: Record<LabDevice['status'], string> = {
  online: 'bg-emerald-400', leased: 'bg-amber-400', offline: 'bg-slate-500',
}
const inp = 'bg-white/5 border border-white/10 rounded px-2 py-1 text-sm focus:border-emerald-400/50 outline-none'
const handleV = 'w-1.5 mx-px rounded bg-white/5 hover:bg-emerald-400/40 transition-colors'
const handleH = 'h-1.5 my-px rounded bg-white/5 hover:bg-emerald-400/40 transition-colors'
const newId = () => 'x_' + Math.random().toString(36).slice(2, 8)
const stop = (e: { stopPropagation: () => void }) => e.stopPropagation()
const devTitle = (d: LabDevice) =>
  `${d.name}\n${d.chip}${d.tags.length ? ' · ' + d.tags.join(' ') : ''}\n${d.ip ? 'ip ' + d.ip + '  ·  ' : ''}${d.addr || '(no address)'}`
const tileWrap = 'flex-1 min-h-0 overflow-auto flex flex-wrap gap-1.5 content-start -mx-1 px-1'

const NAME_PREFIX: Record<string, string> = { host: 'h', cp: 'cp', sniffer: 'snf', 'ap-backend': 'apb', 'usb-hub': 'usb', relay: 'rly', power: 'pow' }
const prefixFor = (type: string) => NAME_PREFIX[type] ?? (type.replace(/[^a-z0-9]/gi, '').slice(0, 3) || 'dev')

const PALETTE = [
  { key: 'amber', sw: 'bg-amber-400', tile: 'border-amber-400/50 bg-amber-400/10' },
  { key: 'yellow', sw: 'bg-yellow-400', tile: 'border-yellow-400/50 bg-yellow-400/10' },
  { key: 'lime', sw: 'bg-lime-400', tile: 'border-lime-400/50 bg-lime-400/10' },
  { key: 'green', sw: 'bg-green-400', tile: 'border-green-400/50 bg-green-400/10' },
  { key: 'teal', sw: 'bg-teal-400', tile: 'border-teal-400/50 bg-teal-400/10' },
  { key: 'cyan', sw: 'bg-cyan-400', tile: 'border-cyan-400/50 bg-cyan-400/10' },
  { key: 'sky', sw: 'bg-sky-400', tile: 'border-sky-400/50 bg-sky-400/10' },
  { key: 'blue', sw: 'bg-blue-400', tile: 'border-blue-400/50 bg-blue-400/10' },
  { key: 'indigo', sw: 'bg-indigo-400', tile: 'border-indigo-400/50 bg-indigo-400/10' },
  { key: 'violet', sw: 'bg-violet-400', tile: 'border-violet-400/50 bg-violet-400/10' },
  { key: 'purple', sw: 'bg-purple-400', tile: 'border-purple-400/50 bg-purple-400/10' },
  { key: 'fuchsia', sw: 'bg-fuchsia-400', tile: 'border-fuchsia-400/50 bg-fuchsia-400/10' },
  { key: 'pink', sw: 'bg-pink-400', tile: 'border-pink-400/50 bg-pink-400/10' },
  { key: 'rose', sw: 'bg-rose-400', tile: 'border-rose-400/50 bg-rose-400/10' },
  { key: 'red', sw: 'bg-red-400', tile: 'border-red-400/50 bg-red-400/10' },
  { key: 'orange', sw: 'bg-orange-400', tile: 'border-orange-400/50 bg-orange-400/10' },
  { key: 'slate', sw: 'bg-slate-400', tile: 'border-slate-400/50 bg-slate-400/10' },
]
const hashIdx = (s: string) => { let h = 0; for (const c of s) h = (h * 31 + c.charCodeAt(0)) >>> 0; return h % PALETTE.length }
const paletteOf = (type: string, colors: Record<string, string>) =>
  PALETTE.find(p => p.key === colors[type]) ?? PALETTE[hashIdx(type)]

type SelKind = DevKind | 'link' | 'type'
interface Sel { kind: SelKind; id: string }

function Toggle({ on, onClick, label }: { on: boolean; onClick: () => void; label: string }) {
  return (
    <button onClick={onClick} className="flex items-center gap-2 text-sm hover:text-white whitespace-nowrap">
      <span className={`h-4 w-7 rounded-full p-0.5 flex transition-colors ${on ? 'bg-emerald-500/70 justify-end' : 'bg-white/15 justify-start'}`}>
        <span className="h-3 w-3 rounded-full bg-white" />
      </span>
      {label}
    </button>
  )
}

function Field({ label, children }: { label: string; children: React.ReactNode }) {
  return <label className="flex flex-col gap-1 text-xs text-slate-400">{label}{children}</label>
}

// Centered editor box used for devices, set-ups, and device types.
function Modal({ title, onClose, onConfirm, children, footer }: {
  title: string; onClose: () => void; onConfirm?: () => void; children: React.ReactNode; footer: React.ReactNode
}) {
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => { if (e.key === 'Escape') onClose() }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [onClose])
  const onContainerKey = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && onConfirm && (e.target as HTMLElement).tagName === 'INPUT') {
      e.stopPropagation()
      onConfirm()
    }
  }
  return (
    <div className="fixed inset-0 z-40 flex items-center justify-center p-4">
      <div className="absolute inset-0 bg-black/50" onClick={onClose} />
      <div className="relative z-50 glass rounded-xl p-4 w-full max-w-md flex flex-col gap-3 max-h-[85vh] overflow-auto"
        onKeyDown={onContainerKey}>
        <div className="flex items-center gap-2">
          <span className="font-semibold">{title}</span>
          <button onClick={onClose} className="btn !px-2 !py-1 text-xs ml-auto" title="collapse (Esc)">▾ Collapse</button>
        </div>
        {children}
        <div className="flex items-center gap-2 pt-1 border-t border-white/10">{footer}</div>
      </div>
    </div>
  )
}

function DeleteControl({ onDelete }: { onDelete: () => void }) {
  const [c, setC] = useState(false)
  return !c
    ? <button onClick={() => setC(true)} className="btn !px-2 !py-1 text-xs text-bad">✕ Delete…</button>
    : <span className="flex items-center gap-2 text-xs"><span className="text-slate-400">Delete?</span>
        <button onClick={onDelete} className="px-2 py-1 rounded bg-rose-500/30 text-bad">Delete</button>
        <button onClick={() => setC(false)} className="px-2 py-1 rounded hover:bg-white/10">Cancel</button></span>
}

function DeviceTile({ d, selected, related, tileColor, onTap }: {
  d: LabDevice; selected: boolean; related: boolean; tileColor?: string; onTap: () => void
}) {
  const base = tileColor ?? 'border-white/10 bg-white/5 hover:border-white/25'
  const dots = selected ? DOT_SEL : related ? DOT_REL : ''
  return (
    <button onClick={onTap} title={`${devTitle(d)}\n\n(click to select · click again to edit)`}
      className={`rounded border px-2 py-1 inline-flex items-center gap-1.5 text-left max-w-full transition-colors ${base} ${dots}`}>
      <span className={`h-1.5 w-1.5 rounded-full shrink-0 ${DOT[d.status]}`} />
      <span className="text-sm truncate">{d.name}</span>
    </button>
  )
}

function LinkTile({ link, active, selected, related, tileColor, onTap }: {
  link: Link; active: boolean; selected: boolean; related: boolean; tileColor: string; onTap: () => void
}) {
  const dots = selected ? DOT_SEL : related ? DOT_REL : ''
  return (
    <button onClick={onTap} title={`${link.name}\n\n(click to select · click again to edit)`}
      className={`rounded border px-2 py-1 inline-flex items-center gap-1.5 text-left max-w-full transition-colors ${tileColor} ${dots}`}>
      {active && <span className="h-1.5 w-1.5 rounded-full bg-emerald-400 shrink-0" title="run target" />}
      <span className="text-sm truncate">{link.name}</span>
    </button>
  )
}

function TypePill({ type, colors, selected, related, onTap }: {
  type: DeviceType; colors: Record<string, string>; selected: boolean; related: boolean; onTap: () => void
}) {
  const pal = paletteOf(type.name, colors)
  const dots = selected ? DOT_SEL : related ? DOT_REL : ''
  return (
    <button onClick={onTap} title={`${type.name}\n${type.fields.length} custom field(s)\n\n(click to select · click again to edit)`}
      className={`rounded border px-2 py-1 inline-flex items-center gap-1.5 text-left max-w-full transition-colors ${pal.tile} ${dots}`}>
      <span className={`h-2 w-2 rounded-full shrink-0 ${pal.sw}`} />
      <span className="text-sm truncate">{type.name}</span>
    </button>
  )
}

function LinkTypePill({ lt, onRename, onToggle, onRemove }: {
  lt: LinkType; onRename: (next: string) => void; onToggle: () => void; onRemove: () => void
}) {
  const [draft, setDraft] = useState(lt.name)
  return (
    <span className="chip inline-flex items-center gap-1">
      <input value={draft} onChange={e => setDraft(e.target.value)}
        onBlur={() => { const t = draft.trim().toLowerCase(); if (t && t !== lt.name) onRename(t); else setDraft(lt.name) }}
        onKeyDown={e => { if (e.key === 'Enter') (e.target as HTMLInputElement).blur() }}
        className="bg-transparent outline-none text-xs" style={{ width: `${Math.max(draft.length, 3)}ch` }} />
      <button onClick={onToggle} className="text-slate-400 hover:text-white text-[11px]" title="wired ↔ wireless">⇄</button>
      <button onClick={onRemove} className="text-slate-400 hover:text-bad text-[11px]" title="remove">✕</button>
    </span>
  )
}

function TypesPane({ types, colors, selId, relName, linkTypes, onAdd, onTap, onClear, onAddLink, onRenameLink, onToggleLink, onRemoveLink }: {
  types: DeviceType[]; colors: Record<string, string>; selId?: string; relName?: string; linkTypes: LinkType[]
  onAdd: () => void; onTap: (name: string) => void; onClear: () => void
  onAddLink: () => void; onRenameLink: (o: string, n: string) => void; onToggleLink: (n: string) => void; onRemoveLink: (n: string) => void
}) {
  return (
    <div className="glass h-full w-full p-3 flex flex-col gap-2 overflow-hidden" onClick={e => { if (e.target === e.currentTarget) onClear() }}>
      <div className="flex items-center gap-2">
        <span className="font-semibold">Device types</span>
        <span className="chip">{types.length}</span>
        <button onClick={onAdd} className="btn !px-2 !py-0.5 text-xs ml-auto" title="add device type">＋ new</button>
      </div>
      <p className="text-xs text-slate-500 -mt-1">every class + colour · select to highlight matching devices</p>
      <div className={tileWrap} onClick={e => { if (e.target === e.currentTarget) onClear() }}>
        {types.map(t => <TypePill key={t.name} type={t} colors={colors} selected={t.name === selId} related={t.name === relName} onTap={() => onTap(t.name)} />)}
        {!types.length && <p className="w-full text-xs text-slate-500 py-4 text-center">no types — ＋ new</p>}
      </div>

      {/* link types — separated box, not a real pane */}
      <div className="border-t border-white/10 pt-2 flex flex-col gap-1.5 shrink-0">
        <div className="flex items-center gap-2">
          <span className="text-sm font-medium">Link types</span>
          <button onClick={onAddLink} className="btn !px-2 !py-0.5 text-xs ml-auto" title="add link type">＋ new</button>
        </div>
        {(['wired', 'wireless'] as const).map(cat => {
          const list = linkTypes.filter(l => l.wireless === (cat === 'wireless'))
          return (
            <div key={cat}>
              <div className="text-[11px] uppercase tracking-wider text-slate-500 mb-1">{cat}</div>
              <div className="flex flex-wrap gap-1.5">
                {list.map(l => <LinkTypePill key={l.name} lt={l} onRename={n => onRenameLink(l.name, n)} onToggle={() => onToggleLink(l.name)} onRemove={() => onRemoveLink(l.name)} />)}
                {!list.length && <span className="text-xs text-slate-600">none — ＋ new (⇄ to flip)</span>}
              </div>
            </div>
          )
        })}
      </div>
    </div>
  )
}

// Editor for a device type: rename, colour, and custom fields (string / combo).
function TypeEditor({ type, inUse, colors, onRename, onChange, onDelete, onClose, onConfirm }: {
  type: DeviceType; inUse: number; colors: Record<string, string>
  onRename: (next: string) => void; onChange: (p: Partial<DeviceType>) => void; onDelete: () => void; onClose: () => void; onConfirm: () => void
}) {
  const [draft, setDraft] = useState(type.name)
  const fields = type.fields
  const setField = (i: number, p: Partial<TypeField>) => onChange({ fields: fields.map((f, idx) => idx === i ? { ...f, ...p } : f) })
  const pal = paletteOf(type.name, colors)
  return (
    <Modal title="Edit device type" onClose={onClose} onConfirm={onConfirm} footer={<>
      {inUse > 0
        ? <span className="text-xs text-slate-500" title="remove devices of this type first">🔒 used by {inUse}</span>
        : <DeleteControl onDelete={onDelete} />}
      <button onClick={onConfirm} className="btn btn-primary !px-3 !py-1 text-xs ml-auto">Done</button>
    </>}>
      <Field label="Type name">
        <input value={draft} onChange={e => setDraft(e.target.value)}
          onBlur={() => { const t = draft.trim().toLowerCase(); if (t && t !== type.name) onRename(t); else setDraft(type.name) }}
          onKeyDown={e => { if (e.key === 'Enter') (e.target as HTMLInputElement).blur() }} className={`${inp} w-full`} />
      </Field>
      <Field label="Colour">
        <div className="flex flex-wrap gap-1.5">
          {PALETTE.map(p => (
            <button key={p.key} onClick={() => onChange({ color: p.key })}
              className={`h-5 w-5 rounded-full ${p.sw} ${p.key === pal.key ? 'ring-2 ring-white/70' : ''}`} title={p.key} />
          ))}
        </div>
      </Field>
      <div className="text-xs text-slate-500">Built-in fields: <span className="text-slate-300">name · type · status · tags · address</span></div>
      <Field label="Custom fields">
        <div className="flex flex-col gap-1.5">
          {fields.map((f, i) => (
            <div key={i} className="flex gap-1.5 items-center">
              <input value={f.name} onChange={e => setField(i, { name: e.target.value })} className={`${inp} w-24`} placeholder="field" />
              <select value={f.kind} onChange={e => setField(i, { kind: e.target.value as TypeField['kind'] })} className={inp}>
                <option value="string" className="bg-slate-800">string</option>
                <option value="combo" className="bg-slate-800">combo</option>
              </select>
              {f.kind === 'combo' && (
                <input value={f.options.join(', ')} onChange={e => setField(i, { options: e.target.value.split(/\s*,\s*/).filter(Boolean) })}
                  className={`${inp} flex-1 min-w-0`} placeholder="entries, comma" />
              )}
              <button onClick={() => onChange({ fields: fields.filter((_, idx) => idx !== i) })} className="text-slate-500 hover:text-bad px-1" title="remove field">✕</button>
            </div>
          ))}
          <button onClick={() => onChange({ fields: [...fields, { name: `field${fields.length + 1}`, kind: 'string', options: [] }] })}
            className="btn !px-2 !py-1 text-xs self-start">＋ add field</button>
        </div>
      </Field>
    </Modal>
  )
}

// Flexible member editor for a Set-up.
function LinkMembers({ link, devices, deviceTypes, colors, onChange, onAddNew }: {
  link: Link; devices: LabDevice[]; deviceTypes: string[]; colors: Record<string, string>
  onChange: (p: Partial<Link>) => void; onAddNew: (type: string, name: string) => void
}) {
  const members = link.memberIds ?? []
  const [type, setType] = useState(deviceTypes[0] ?? '')
  const [name, setName] = useState('')
  const [confirmId, setConfirmId] = useState<string | null>(null)
  const find = (id: string) => devices.find(d => d.id === id)
  const attached = members.map(find).filter(Boolean) as LabDevice[]
  const available = devices.filter(d => !members.includes(d.id) && d.id !== link.hostId && d.id !== link.cpId)
  const add = () => { if (type) { onAddNew(type, name.trim()); setName('') } }
  return (
    <div className="flex flex-col gap-2">
      <div className="flex flex-wrap gap-1.5">
        {attached.map(d => (
          <span key={d.id} className={`chip inline-flex items-center gap-1 border ${paletteOf(d.chip, colors).tile}`}>
            {d.name}
            {confirmId === d.id
              ? <span className="inline-flex items-center gap-1 text-[11px]">
                  <button onClick={() => { onChange({ memberIds: members.filter(x => x !== d.id) }); setConfirmId(null) }} className="text-bad">remove</button>
                  <button onClick={() => setConfirmId(null)} className="text-slate-400 hover:text-white">cancel</button>
                </span>
              : <button onClick={() => setConfirmId(d.id)} className="text-slate-400 hover:text-bad" title="remove">✕</button>}
          </span>
        ))}
        {!attached.length && <span className="text-xs text-slate-500">none attached</span>}
      </div>
      {available.length > 0 && (
        <div className="flex flex-wrap gap-1.5">
          {available.map(d => (
            <button key={d.id} onClick={() => onChange({ memberIds: [...members, d.id] })} className={`chip border hover:brightness-125 ${paletteOf(d.chip, colors).tile}`}>＋ {d.name}</button>
          ))}
        </div>
      )}
      <div className="flex gap-1.5 items-center">
        <select value={type} onChange={e => setType(e.target.value)} className={inp}>
          {deviceTypes.map(t => <option key={t} value={t} className="bg-slate-800">{t}</option>)}
        </select>
        <input value={name} onChange={e => setName(e.target.value)} placeholder="new device name"
          onKeyDown={e => { if (e.key === 'Enter') { e.stopPropagation(); add() } }} className={`${inp} flex-1 min-w-0`} />
        <button onClick={add} className="btn !px-2 !py-1 text-xs">＋ new</button>
      </div>
    </div>
  )
}

function DeviceEditor({ kind, device, typeFields, types, ports, onChange, onDelete, onClose, onConfirm }: {
  kind: DevKind; device: LabDevice; typeFields: TypeField[]; types: string[]; ports: string[]
  onChange: (p: Partial<LabDevice>) => void; onDelete: () => void; onClose: () => void; onConfirm: () => void
}) {
  const title = kind === 'cp' ? 'Edit co-processor' : kind === 'host' ? 'Edit host' : 'Edit helper'
  const setF = (name: string, v: string) => onChange({ fields: { ...(device.fields ?? {}), [name]: v } })
  const listId = `eh-ports-${device.id}`
  return (
    <Modal title={title} onClose={onClose} onConfirm={onConfirm} footer={<>
      <DeleteControl onDelete={onDelete} />
      <button onClick={onConfirm} className="btn btn-primary !px-3 !py-1 text-xs ml-auto">Done</button>
    </>}>
      <Field label="Name"><input value={device.name} onChange={e => onChange({ name: e.target.value })} className={`${inp} w-full`} placeholder="device name" /></Field>
      <div className="flex gap-2">
        <Field label="Type"><select value={device.chip} onChange={e => onChange({ chip: e.target.value })} className={`${inp} w-full`}>
          {types.map(c => <option key={c} value={c} className="bg-slate-800">{c}</option>)}</select></Field>
        <Field label="Status"><select value={device.status} onChange={e => onChange({ status: e.target.value as LabDevice['status'] })} className={`${inp} w-full`}>
          {(['online', 'leased', 'offline'] as const).map(s => <option key={s} value={s} className="bg-slate-800">{s}</option>)}</select></Field>
      </div>
      <Field label="Tags"><input value={device.tags.join(', ')} onChange={e => onChange({ tags: e.target.value.split(/[,\s]+/).filter(Boolean) })} className={`${inp} w-full`} placeholder="imx, monitor…" /></Field>
      {kind === 'host' && (
        <Field label="IP (host runtime)"><input value={device.ip ?? ''} onChange={e => onChange({ ip: e.target.value })} className={`${inp} w-full font-mono text-xs`} placeholder="192.168.1.10" /></Field>
      )}
      <Field label={kind === 'host' ? 'Flash port (serial path)' : 'Address (serial path or rfc2217)'}>
        {ports.length > 0 && <datalist id={listId}>{ports.map(p => <option key={p} value={p} />)}</datalist>}
        <input value={device.addr} onChange={e => onChange({ addr: e.target.value })} list={ports.length ? listId : undefined}
          className={`${inp} w-full font-mono text-xs`} placeholder="/dev/serial/by-id/… or rfc2217://…" />
      </Field>
      {typeFields.map(f => (
        <Field key={f.name} label={f.name}>
          {f.kind === 'combo'
            ? <select value={device.fields?.[f.name] ?? ''} onChange={e => setF(f.name, e.target.value)} className={`${inp} w-full`}>
                <option value="" className="bg-slate-800">—</option>
                {f.options.map(o => <option key={o} value={o} className="bg-slate-800">{o}</option>)}
              </select>
            : <input value={device.fields?.[f.name] ?? ''} onChange={e => setF(f.name, e.target.value)} className={`${inp} w-full`} />}
        </Field>
      ))}
    </Modal>
  )
}

function LinkEditor({ link, hosts, cps, members, deviceTypes, linkTypes, colors, onChange, onAddMember, onDelete, onClose, onConfirm }: {
  link: Link; hosts: LabDevice[]; cps: LabDevice[]; members: LabDevice[]; deviceTypes: string[]; linkTypes: LinkType[]; colors: Record<string, string>
  onChange: (p: Partial<Link>) => void; onAddMember: (type: string, name: string) => void; onDelete: () => void; onClose: () => void; onConfirm: () => void
}) {
  return (
    <Modal title="Edit set-up" onClose={onClose} onConfirm={onConfirm} footer={<>
      <DeleteControl onDelete={onDelete} />
      <button onClick={onConfirm} className="btn btn-primary !px-3 !py-1 text-xs ml-auto">Done</button>
    </>}>
      <Field label="Name"><input value={link.name} onChange={e => onChange({ name: e.target.value })} className={`${inp} w-full`} placeholder="set-up name" /></Field>
      <Field label="Host"><select value={link.hostId ?? ''} onChange={e => onChange({ hostId: e.target.value || undefined })} className={`${inp} w-full`}>
        <option value="" className="bg-slate-800">host…</option>
        {hosts.map(h => <option key={h.id} value={h.id} className="bg-slate-800">{h.name}</option>)}</select></Field>
      <Field label="Co-processor"><select value={link.cpId ?? ''} onChange={e => onChange({ cpId: e.target.value || undefined })} className={`${inp} w-full`}>
        <option value="" className="bg-slate-800">co-processor…</option>
        {cps.map(c => <option key={c.id} value={c.id} className="bg-slate-800">{c.name}</option>)}</select></Field>
      <Field label="Transport / link"><select value={link.transport} onChange={e => onChange({ transport: e.target.value })} className={`${inp} w-full`}>
        <optgroup label="wired" className="bg-slate-800">{linkTypes.filter(l => !l.wireless).map(l => <option key={l.name} value={l.name} className="bg-slate-800">{l.name.toUpperCase()}</option>)}</optgroup>
        <optgroup label="wireless" className="bg-slate-800">{linkTypes.filter(l => l.wireless).map(l => <option key={l.name} value={l.name} className="bg-slate-800">{l.name.toUpperCase()}</option>)}</optgroup>
      </select></Field>
      <Field label="Attached devices (any type, multiple allowed)">
        <LinkMembers link={link} devices={members} deviceTypes={deviceTypes} colors={colors} onChange={onChange} onAddNew={onAddMember} />
      </Field>
    </Modal>
  )
}

function DevicePane({ title, hint, devices, selId, relatedIds, tileColorOf, onTap, onAdd, onClear }: {
  title: string; hint: string; devices: LabDevice[]; selId?: string; relatedIds?: Set<string>
  tileColorOf?: (d: LabDevice) => string | undefined; onTap: (id: string) => void; onAdd: () => void; onClear: () => void
}) {
  return (
    <div className="glass h-full w-full p-3 flex flex-col gap-2 overflow-hidden" onClick={e => { if (e.target === e.currentTarget) onClear() }}>
      <div className="flex items-center gap-2">
        <span className="font-semibold">{title}</span>
        <span className="chip">{devices.length}</span>
        <button onClick={onAdd} className="btn !px-2 !py-0.5 text-xs ml-auto" title={`add ${title}`}>＋ new</button>
      </div>
      <p className="text-xs text-slate-500 -mt-1">{hint}</p>
      <div className={tileWrap} onClick={e => { if (e.target === e.currentTarget) onClear() }}>
        {devices.map(d => (
          <DeviceTile key={d.id} d={d} selected={d.id === selId} related={relatedIds?.has(d.id) ?? false}
            tileColor={tileColorOf?.(d)} onTap={() => onTap(d.id)} />
        ))}
        {!devices.length && <p className="w-full text-xs text-slate-500 py-4 text-center">none — ＋ new or ⟳ Probe</p>}
      </div>
    </div>
  )
}

export default function LabPage({ lab, setLab, col }: { lab: LabState; setLab: (f: (l: LabState) => LabState) => void; col: LabCol }) {
  const [sel, setSel] = useState<Sel | null>(null)
  const [edit, setEdit] = useState<Sel | null>(null)
  const [probedPorts, setProbedPorts] = useState<string[]>([])
  const [creatingId, setCreatingId] = useState<string | null>(null)
  const isSel = (k: SelKind, id: string) => sel?.kind === k && sel.id === id
  const colors = colorMap(lab)
  const names = lab.deviceTypes.map(t => t.name)
  const allDevices = [...lab.hosts, ...lab.cps, ...lab.helpers]

  const key = (k: DevKind) => (k === 'host' ? 'hosts' : k === 'cp' ? 'cps' : 'helpers') as 'hosts' | 'cps' | 'helpers'
  const paneForType = (t: string): 'hosts' | 'cps' | 'helpers' => t === 'host' ? 'hosts' : t === 'cp' ? 'cps' : 'helpers'
  const setMode = (mode: LabState['mode']) => setLab(l => {
    const c = catalog(mode)
    const link: Link = { id: newId(), name: `${mode} bench`, hostId: c.hosts[0]?.id, cpId: c.cps[0]?.id, transport: 'sdio', memberIds: [] }
    return { ...l, mode, hosts: c.hosts, cps: c.cps, helpers: c.helpers, links: [link], activeLinkId: undefined, hub: { ...l.hub, connected: mode === 'donut' } }
  })
  const probe = () => {
    if (lab.mode === 'local') {
      fetch('/api/v1/devices')
        .then(r => r.ok ? r.json() : [])
        .then((data: { path: string }[]) => setProbedPorts(data.map(d => d.path)))
        .catch(() => {})
    } else {
      // donut mode: merge catalog devices (mock until donut integration)
      setLab(l => {
        const c = catalog(l.mode)
        const merge = (cur: LabDevice[], f: LabDevice[]) => [...cur, ...f.filter(x => !cur.some(d => d.id === x.id))]
        return { ...l, hosts: merge(l.hosts, c.hosts), cps: merge(l.cps, c.cps), helpers: merge(l.helpers, c.helpers) }
      })
    }
  }
  const onDev = (k: DevKind, id: string, p: Partial<LabDevice>) =>
    setLab(l => ({ ...l, [key(k)]: l[key(k)].map(d => d.id === id ? { ...d, ...p } : d) }))
  const addDev = (k: DevKind) => {
    const id = newId()
    setLab(l => {
      const tn = l.deviceTypes.map(t => t.name)
      const chip = k === 'host' ? 'host' : k === 'cp' ? 'cp' : (tn.find(t => !['host', 'cp', 'setup'].includes(t)) ?? 'sniffer')
      return { ...l, [key(k)]: [...l[key(k)], { id, name: `${prefixFor(chip)}-`, chip, tags: [], addr: '', status: 'offline' } as LabDevice] }
    })
    setSel({ kind: k, id }); setEdit({ kind: k, id }); setCreatingId(id)
  }
  const delDev = (k: DevKind, id: string) => { setLab(l => ({ ...l, [key(k)]: l[key(k)].filter(d => d.id !== id) })); setSel(null); setEdit(null) }
  const tapDev = (k: DevKind, id: string) => { if (isSel(k, id)) setEdit({ kind: k, id }); else setSel({ kind: k, id }) }

  // device types (schemas)
  const addType = () => {
    let n = 1; let name = `type-${n}`
    while (lab.deviceTypes.some(t => t.name === name)) { n++; name = `type-${n}` }
    setLab(l => ({ ...l, deviceTypes: [...l.deviceTypes, { name, fields: [] }] }))
    setSel({ kind: 'type', id: name }); setEdit({ kind: 'type', id: name }); setCreatingId(name)
  }
  const updateType = (name: string, p: Partial<DeviceType>) => setLab(l => ({ ...l, deviceTypes: l.deviceTypes.map(t => t.name === name ? { ...t, ...p } : t) }))
  const renameType = (old: string, nu: string) => setLab(l => {
    if (l.deviceTypes.some(t => t.name === nu)) return l
    const remap = (arr: LabDevice[]) => arr.map(d => d.chip === old ? { ...d, chip: nu } : d)
    return { ...l, deviceTypes: l.deviceTypes.map(t => t.name === old ? { ...t, name: nu } : t), hosts: remap(l.hosts), cps: remap(l.cps), helpers: remap(l.helpers) }
  })
  const removeType = (name: string) => { setLab(l => allDevices.some(d => d.chip === name) ? l : { ...l, deviceTypes: l.deviceTypes.filter(t => t.name !== name) }); setSel(null); setEdit(null) }
  const typeCount = (name: string) => allDevices.filter(d => d.chip === name).length
  const tapType = (name: string) => { if (isSel('type', name)) setEdit({ kind: 'type', id: name }); else setSel({ kind: 'type', id: name }) }

  // set-up links
  const onLink = (id: string, p: Partial<Link>) => setLab(l => ({ ...l, links: l.links.map(k => k.id === id ? { ...k, ...p } : k) }))
  const tapLink = (id: string) => { setLab(l => ({ ...l, activeLinkId: id })); if (isSel('link', id)) setEdit({ kind: 'link', id }); else setSel({ kind: 'link', id }) }
  const addLink = () => {
    const id = newId()
    setLab(l => ({ ...l, links: [...l.links, { id, name: 'new set-up', hostId: l.hosts[0]?.id, cpId: l.cps[0]?.id, transport: 'sdio', memberIds: [] }], activeLinkId: id }))
    setSel({ kind: 'link', id }); setEdit({ kind: 'link', id }); setCreatingId(id)
  }
  const delLink = (id: string) => { setLab(l => { const links = l.links.filter(k => k.id !== id); return { ...l, links, activeLinkId: l.activeLinkId === id ? links[0]?.id : l.activeLinkId } }); setSel(null); setEdit(null) }
  const addMember = (linkId: string, type: string, name: string) => {
    const id = newId()
    setLab(l => ({
      ...l,
      [paneForType(type)]: [...l[paneForType(type)], { id, name: name || `${prefixFor(type)}-`, chip: type, tags: [], addr: '', status: 'offline' } as LabDevice],
      links: l.links.map(k => k.id === linkId ? { ...k, memberIds: [...(k.memberIds ?? []), id] } : k),
    }))
  }
  // link types (wired / wireless)
  const addLinkType = () => {
    let n = 1; let name = `link-${n}`
    while (lab.linkTypes.some(l => l.name === name)) { n++; name = `link-${n}` }
    setLab(l => ({ ...l, linkTypes: [...l.linkTypes, { name, wireless: false }] }))
  }
  const renameLinkType = (old: string, nu: string) => setLab(l => l.linkTypes.some(x => x.name === nu) ? l
    : { ...l, linkTypes: l.linkTypes.map(x => x.name === old ? { ...x, name: nu } : x), links: l.links.map(k => k.transport === old ? { ...k, transport: nu } : k) })
  const toggleLinkType = (name: string) => setLab(l => ({ ...l, linkTypes: l.linkTypes.map(x => x.name === name ? { ...x, wireless: !x.wireless } : x) }))
  const removeLinkType = (name: string) => setLab(l => ({ ...l, linkTypes: l.linkTypes.filter(x => x.name !== name) }))
  const setOpt = <K extends keyof LabState['opts']>(o: K, v: LabState['opts'][K]) => setLab(l => ({ ...l, opts: { ...l.opts, [o]: v } }))

  // keyboard: ←/→/↑/↓ move selection across visible tiles, Enter edits, Esc deselects
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const t = e.target as HTMLElement | null
      if (t && /^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName)) return
      if (edit) return  // editor open → its own Esc handles close
      const items: Sel[] = []
      if (!col.cp) lab.cps.forEach(d => items.push({ kind: 'cp', id: d.id }))
      if (!col.host) lab.hosts.forEach(d => items.push({ kind: 'host', id: d.id }))
      if (!col.help) lab.helpers.forEach(d => items.push({ kind: 'helper', id: d.id }))
      if (!col.types) lab.deviceTypes.forEach(x => items.push({ kind: 'type', id: x.name }))
      if (!col.setup) lab.links.forEach(k => items.push({ kind: 'link', id: k.id }))
      const idx = sel ? items.findIndex(i => i.kind === sel.kind && i.id === sel.id) : -1
      const move = (dir: number) => { if (!items.length) return; const i = idx < 0 ? (dir > 0 ? 0 : items.length - 1) : (idx + dir + items.length) % items.length; setSel(items[i]) }
      if (e.key === 'ArrowDown' || e.key === 'ArrowRight') { e.preventDefault(); move(1) }
      else if (e.key === 'ArrowUp' || e.key === 'ArrowLeft') { e.preventDefault(); move(-1) }
      else if (e.key === 'Enter' && sel) { e.preventDefault(); if (sel.kind === 'link') setLab(l => ({ ...l, activeLinkId: sel.id })); setEdit(sel) }
      else if (e.key === 'Escape') setSel(null)
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [sel, edit, col, lab])

  type ColKey = 'cp' | 'host' | 'help' | 'types'
  const tileColorOf = (d: LabDevice) => paletteOf(d.chip, colors).tile

  // any-to-any relation highlight, driven purely by the current selection
  const relDev = new Set<string>(); const relLink = new Set<string>(); let relType: string | undefined
  if (sel?.kind === 'link') {
    const L = lab.links.find(k => k.id === sel.id)
    if (L) [L.hostId, L.cpId, ...(L.memberIds ?? [])].forEach(id => { if (id) relDev.add(id) })
  } else if (sel?.kind === 'type') {
    allDevices.forEach(d => { if (d.chip === sel.id) relDev.add(d.id) })
  } else if (sel) {
    lab.links.forEach(k => { if (k.hostId === sel.id || k.cpId === sel.id || (k.memberIds ?? []).includes(sel.id)) relLink.add(k.id) })
    relType = allDevices.find(d => d.id === sel.id)?.chip
  }
  const devSel = (k: DevKind) => sel?.kind === k ? sel.id : undefined

  const clear = () => setSel(null)
  const PANE: Record<ColKey, JSX.Element> = {
    cp: <DevicePane title="Co-processors" hint="coloured by device type" devices={lab.cps}
      selId={devSel('cp')} relatedIds={relDev} tileColorOf={tileColorOf} onTap={id => tapDev('cp', id)} onAdd={() => addDev('cp')} onClear={clear} />,
    host: <DevicePane title="Hosts" hint="IP + flash port, coloured by type" devices={lab.hosts}
      selId={devSel('host')} relatedIds={relDev} tileColorOf={tileColorOf} onTap={id => tapDev('host', id)} onAdd={() => addDev('host')} onClear={clear} />,
    help: <DevicePane title="Helpers" hint="other devices, coloured by type" devices={lab.helpers}
      selId={devSel('helper')} relatedIds={relDev} tileColorOf={tileColorOf} onTap={id => tapDev('helper', id)} onAdd={() => addDev('helper')} onClear={clear} />,
    types: <TypesPane types={lab.deviceTypes} colors={colors} selId={sel?.kind === 'type' ? sel.id : undefined} relName={relType} linkTypes={lab.linkTypes}
      onAdd={addType} onTap={tapType} onClear={clear} onAddLink={addLinkType} onRenameLink={renameLinkType} onToggleLink={toggleLinkType} onRemoveLink={removeLinkType} />,
  }
  const openCols = (['cp', 'host', 'help', 'types'] as const).filter(k => !col[k])
  const showTop = openCols.length > 0
  const showSetup = !col.setup
  const setupTile = paletteOf('setup', colors).tile

  return (
    <div className="h-full flex flex-col gap-2">
      <section className="glass p-2 flex items-center gap-3 flex-wrap">
        <div className="flex rounded-lg overflow-hidden border border-white/10">
          {(['donut', 'local'] as const).map(m => (
            <button key={m} onClick={() => setMode(m)}
              className={`px-3 py-1 text-sm ${lab.mode === m ? 'bg-emerald-500/30 text-emerald-100' : 'hover:bg-white/5'}`}>
              {m === 'donut' ? 'donut (remote)' : 'Local ports'}</button>
          ))}
        </div>
        {lab.mode === 'donut' && (
          <span className="flex items-center gap-2 text-sm text-slate-300">
            <span className={`h-2 w-2 rounded-full ${lab.hub.connected ? 'bg-emerald-400' : 'bg-slate-500'}`} />
            <code className="text-xs text-slate-400">{lab.hub.url}</code>
          </span>
        )}
        <button onClick={probe} className="btn !px-2 !py-1 text-xs" title="re-scan / donut ls">⟳ Probe</button>
        <span className="opacity-20">|</span>
        <Toggle on={lab.opts.flashBeforeRun} onClick={() => setOpt('flashBeforeRun', !lab.opts.flashBeforeRun)} label="Flash before run" />
        <Toggle on={lab.opts.chipGuard} onClick={() => setOpt('chipGuard', !lab.opts.chipGuard)} label="Pre-flash chip guard" />
        <label className="flex items-center gap-2 text-sm">Baud
          <select value={lab.opts.baud} onChange={e => setOpt('baud', Number(e.target.value))} className={inp}>
            {BAUDS.map(b => <option key={b} value={b} className="bg-slate-800">{b}</option>)}
          </select>
        </label>
        <button onClick={() => { setLab(() => DEFAULT_LAB); setSel(null); setEdit(null) }} className="btn !px-2 !py-1 text-xs ml-auto">reset defaults</button>
      </section>

      <div className="flex-1 min-h-0">
        {!showTop && !showSetup ? (
          <div className="glass h-full grid place-items-center text-sm text-slate-500">all panes hidden — pick one from the bar below</div>
        ) : (
          <PanelGroup key={`${showTop ? 'T' : ''}${showSetup ? 'S' : ''}`} direction="vertical" className="h-full">
            {showTop && (
              <Panel order={1} minSize={20} defaultSize={showSetup ? 68 : 100}>
                <PanelGroup key={openCols.join('-')} direction="horizontal" className="h-full">
                  {openCols.map((k, i) => (
                    <Fragment key={k}>
                      {i > 0 && <PanelResizeHandle className={handleV} />}
                      <Panel order={i + 1} minSize={12} defaultSize={Math.floor(100 / openCols.length)}>
                        <div className="h-full px-0.5">{PANE[k]}</div>
                      </Panel>
                    </Fragment>
                  ))}
                </PanelGroup>
              </Panel>
            )}
            {showTop && showSetup && <PanelResizeHandle className={handleH} />}
            {showSetup && (
              <Panel order={2} minSize={12} defaultSize={showTop ? 32 : 100}>
                <div className="h-full pt-1">
                  <div className="glass h-full p-3 flex flex-col gap-2 overflow-hidden">
                    <div className="flex items-center gap-2">
                      <span className="font-semibold">Set-up</span>
                      <span className="chip">{lab.links.length}</span>
                      <span className="text-xs text-slate-500">click to select + activate · again to edit</span>
                      <button onClick={addLink} className="btn !px-2 !py-0.5 text-xs ml-auto" title="add set-up">＋ new</button>
                    </div>
                    <div className={tileWrap} onClick={e => { if (e.target === e.currentTarget) clear() }}>
                      {lab.links.map(k => (
                        <LinkTile key={k.id} link={k} active={k.id === lab.activeLinkId} selected={sel?.kind === 'link' && sel.id === k.id}
                          related={relLink.has(k.id)} tileColor={setupTile} onTap={() => tapLink(k.id)} />
                      ))}
                      {!lab.links.length && <p className="w-full text-xs text-slate-500 py-4 text-center">no set-ups — ＋ new</p>}
                    </div>
                  </div>
                </div>
              </Panel>
            )}
          </PanelGroup>
        )}
      </div>

      {edit?.kind === 'type' && lab.deviceTypes.find(t => t.name === edit.id) && (
        <TypeEditor type={lab.deviceTypes.find(t => t.name === edit.id)!} inUse={typeCount(edit.id)} colors={colors}
          onRename={nu => { if (creatingId === edit.id) setCreatingId(nu); renameType(edit.id, nu); setEdit({ kind: 'type', id: nu }); setSel({ kind: 'type', id: nu }) }}
          onChange={p => updateType(edit.id, p)} onDelete={() => { setCreatingId(null); removeType(edit.id) }}
          onClose={() => { if (creatingId === edit.id) removeType(edit.id); setCreatingId(null); setEdit(null) }}
          onConfirm={() => { setCreatingId(null); setEdit(null) }} />
      )}
      {edit?.kind === 'link' && lab.links.find(k => k.id === edit.id) && (
        <LinkEditor link={lab.links.find(k => k.id === edit.id)!} hosts={lab.hosts} cps={lab.cps} members={allDevices}
          deviceTypes={names} linkTypes={lab.linkTypes} colors={colors} onChange={p => onLink(edit.id, p)} onAddMember={(t, n) => addMember(edit.id, t, n)}
          onDelete={() => { setCreatingId(null); delLink(edit.id) }}
          onClose={() => { if (creatingId === edit.id) delLink(edit.id); setCreatingId(null); setEdit(null) }}
          onConfirm={() => { setCreatingId(null); setEdit(null) }} />
      )}
      {edit && edit.kind !== 'link' && edit.kind !== 'type' && lab[key(edit.kind as DevKind)].find(d => d.id === edit.id) && (
        <DeviceEditor kind={edit.kind} device={lab[key(edit.kind as DevKind)].find(d => d.id === edit.id)!}
          typeFields={lab.deviceTypes.find(t => t.name === lab[key(edit.kind as DevKind)].find(d => d.id === edit.id)!.chip)?.fields ?? []}
          types={names} ports={probedPorts}
          onChange={p => onDev(edit.kind as DevKind, edit.id, p)}
          onDelete={() => { setCreatingId(null); delDev(edit.kind as DevKind, edit.id) }}
          onClose={() => { if (creatingId === edit.id) delDev(edit.kind as DevKind, edit.id); setCreatingId(null); setEdit(null) }}
          onConfirm={() => { setCreatingId(null); setEdit(null) }} />
      )}
    </div>
  )
}
