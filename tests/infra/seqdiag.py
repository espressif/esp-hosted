"""Text sequence diagram from a test's archived logs — an at-a-glance overview.

Reads the host log (the host drives the link and observes every response/event, so
one file yields an accurate host-centric interaction) and turns high-signal lines
into a NET/HOST/CP sequence: commands and their result codes, RPC handshake, Wi-Fi
events, and the sleep/wake dance. Emits BOTH a mermaid `sequenceDiagram` block
(renders in Markdown viewers) and a plain ASCII lane diagram (readable in a
terminal) — so the flow is graspable without wading through the raw logs.

Used by `eh.py test … --sequence-diagram`.
"""
import re
from pathlib import Path

# Ordered rules: first match on a line wins. Each yields one or more steps
# (frm, to, kind, label). kind: 'req' solid →, 'rsp' dashed ⇠, 'note' over actor.
_RULES = [
    (re.compile(r'host connected to '),                      lambda m: [("HOST", "CP", "req", "UART/SDIO bridge up")]),
    (re.compile(r'Slave reset issued'),                      lambda m: [("HOST", "CP", "req", "slave reset")]),
    (re.compile(r'slave chip id: (0x[0-9a-fA-F]+) \(([^)]+)\)'), lambda m: [("CP", "HOST", "rsp", f"chip {m.group(2)} ({m.group(1)})")]),
    (re.compile(r'slave fw version: (0x[0-9a-fA-F]+)'),       lambda m: [("CP", "HOST", "rsp", f"fw {m.group(1)}")]),
    (re.compile(r'RPC version negotiated: (\w+)'),           lambda m: [("CP", "HOST", "rsp", f"RPC {m.group(1)} negotiated")]),
    (re.compile(r'EH rc=(-?\d+) cmd=(\w+)'),                 lambda m: [("HOST", "CP", "req", f"cmd {m.group(2)}"),
                                                                        ("CP", "HOST", "rsp", f"rc={m.group(1)}")]),
    (re.compile(r'Connecting to (\S+?)\.'),                  lambda m: [("HOST", "CP", "req", f"connect {m.group(1)}")]),
    (re.compile(r'rx RPC WifiEventNoArgs id=(\d+)'),         lambda m: [("CP", "HOST", "rsp", f"WifiEvent id={m.group(1)}")]),
    (re.compile(r'WIFI_EVENT_STA_START'),                    lambda m: [("CP", "HOST", "rsp", "STA_START")]),
    (re.compile(r'WIFI_EVENT_STA_CONNECTED'),                lambda m: [("CP", "HOST", "rsp", "STA_CONNECTED")]),
    (re.compile(r'WIFI_EVENT_STA_DISCONNECTED'),             lambda m: [("CP", "HOST", "rsp", "STA_DISCONNECTED")]),
    (re.compile(r'IP_EVENT_STA_GOT_IP:.*?address: ([0-9.]+)'), lambda m: [("CP", "HOST", "rsp", f"GOT_IP {m.group(1)}")]),
    (re.compile(r'PMU deep-sleep entry'),                    lambda m: [("HOST", "HOST", "note", "enter deep sleep")]),
    (re.compile(r'deep-sleep wakeup armed|Armed wakeup GPIO'), lambda m: [("HOST", "HOST", "note", "arm wake source")]),
    (re.compile(r'RX OP_WAKE'),                              lambda m: [("NET", "HOST", "req", "wake packet")]),
    (re.compile(r'Restarting emulator \(deep-sleep|leaving deep sleep'), lambda m: [("HOST", "HOST", "note", "resume from sleep")]),
    (re.compile(r'Host wakeup triggered.*len: (\d+)'),       lambda m: [("CP", "HOST", "rsp", f"wake RX ({m.group(1)} B)")]),
]

_STEP_CAP = 200  # keep the overview an overview
_ANSI = re.compile(r'\x1b\[[0-9;]*m')


def _parse(text):
    steps = []
    for raw in text.splitlines():
        line = _ANSI.sub("", raw)
        for rx, fn in _RULES:
            m = rx.search(line)
            if m:
                steps.extend(fn(m))
                break
    # Collapse consecutive identical steps into one with a ×N count.
    out = []
    for s in steps:
        if out and out[-1][:4] == s[:4] and len(out[-1]) == 5:
            out[-1] = (*s, out[-1][4] + 1)
        elif out and out[-1] == s:
            out[-1] = (*s, 2)
        else:
            out.append(s)
    return out


def _host_log(logdir: Path):
    for name in ("host.log", "host_uart.log", "host_sdio.log"):
        p = logdir / name
        if p.is_file():
            return p
    hits = sorted(logdir.glob("host*.log"))
    return hits[0] if hits else None


def _mermaid(steps):
    out = ["```mermaid", "sequenceDiagram", "  participant NET",
           "  participant HOST", "  participant CP"]
    for s in steps:
        frm, to, kind, label = s[0], s[1], s[2], s[3]
        n = s[4] if len(s) == 5 else 1
        tag = f"{label} ×{n}" if n > 1 else label
        if kind == "note":
            out.append(f"  Note over HOST: {tag}")
        else:
            arrow = "->>" if kind == "req" else "-->>"
            out.append(f"  {frm}{arrow}{to}: {tag}")
    out.append("```")
    return out


# ASCII lane diagram: three fixed lifelines.
_COL = {"NET": 2, "HOST": 16, "CP": 30}
_WIDTH = 33


def _ascii(steps):
    header = list(" " * _WIDTH)
    for a, c in _COL.items():
        header[c - (len(a) // 2):c - (len(a) // 2) + len(a)] = a
    rows = ["".join(header).rstrip(),
            "".join("|" if i in _COL.values() else " " for i in range(_WIDTH))]
    for s in steps:
        frm, to, kind, label = s[0], s[1], s[2], s[3]
        n = s[4] if len(s) == 5 else 1
        tag = f"{label} (x{n})" if n > 1 else label
        row = ["|" if i in _COL.values() else " " for i in range(_WIDTH)]
        if kind == "note":
            box = f"[ {tag} ]"
            start = max(0, _COL["HOST"] - len(box) // 2)
            row[start:start + len(box)] = list(box)[:_WIDTH - start]
            rows.append("".join(row).rstrip())
            continue
        a, b = _COL[frm], _COL[to]
        lo, hi = min(a, b), max(a, b)
        dash = "-" if kind == "req" else "."
        for i in range(lo + 1, hi):
            row[i] = dash
        row[b] = ">" if b > a else "<"
        row[a] = "|"
        rows.append("".join(row).rstrip().ljust(_WIDTH) + "  " + tag)
    return rows


def render_dir(logdir: Path):
    hp = _host_log(logdir)
    if not hp:
        return None
    steps = _parse(hp.read_text(errors="replace"))
    if not steps:
        return None
    capped = steps[:_STEP_CAP]
    md = [f"# Sequence — {logdir.name}", "",
          f"Host-observed interaction (parsed from `{hp.name}`; NET = external "
          f"packet injector). {len(steps)} step(s)"
          + (f", showing first {_STEP_CAP}." if len(steps) > _STEP_CAP else "."),
          "", "## Overview (ASCII)", "", "```text"]
    md += _ascii(capped)
    md += ["```", "", "## Mermaid"]
    md += _mermaid(capped)
    md += ["", f"_Raw logs: `{logdir.name}/` (host + CP)._"]
    return "\n".join(md) + "\n"
