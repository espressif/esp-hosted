#!/usr/bin/env python3
"""esplab — ESP-Hosted test lab launcher and HTTP server.

Every UI control maps 1:1 to a CLI switch.

Usage:
  ./esplab.py --serve                 # serve frontend/dist/ + /api/v1/
  ./esplab.py --serve --port 8321
  ./esplab.py --serve --no-open       # server only, no browser
  ./esplab.py probe                   # list connected serial/USB devices
  ./esplab.py --scenario recovery     # dev mode: open Vite dev server URL
  ./esplab.py --headless              # print URL only

Dev mode: run Vite separately (npm run dev in frontend/).  vite.config.ts
proxies /api → :8321 so you can get both HMR and a real API simultaneously.

API (--serve mode):
  GET  /api/v1/devices  — enumerate connected serial/USB ports
  GET  /api/v1/lab      — load persisted lab state (lab.json)
  PUT  /api/v1/lab      — persist lab state
"""
import argparse
import glob
import http.server
import json
import pathlib
import socketserver
import sys
import urllib.parse
import webbrowser

import runner  # sibling module — build/emulate/assert engine (usable standalone too)

HERE = pathlib.Path(__file__).parent
DIST = HERE / 'frontend' / 'dist'
LAB_JSON = HERE / 'lab.json'
DEFAULT_PORT = 8321

_MIME = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'application/javascript',
    '.mjs': 'application/javascript',
    '.css': 'text/css',
    '.json': 'application/json',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
    '.ico': 'image/x-icon',
    '.woff2': 'font/woff2',
    '.woff': 'font/woff',
    '.txt': 'text/plain',
}

AXES = {
    'env':       ['esp_sim', 'hw'],
    'host':      ['mcu_current', 'mcu_old', 'linux_current', 'linux_old'],
    'cp':        ['cp_current', 'mcu_slave_old', 'linux_cp_legacy'],
    'transport': ['sdio', 'spi', 'uart'],
    'rpc':       ['v1', 'v2'],
}


# ── Device probe ──────────────────────────────────────────────────────────────

def probe_serial() -> list:
    if sys.platform == 'darwin':
        paths = sorted(set(glob.glob('/dev/cu.usb*') + glob.glob('/dev/cu.SLAB*')))
    else:
        by_id = sorted(glob.glob('/dev/serial/by-id/*'))
        paths = by_id if by_id else sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    return [{'path': p, 'name': p.split('/')[-1]} for p in paths]


# ── HTTP handler ──────────────────────────────────────────────────────────────

class _Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_OPTIONS(self):
        self._head(200)
        self.end_headers()

    def do_GET(self):
        path = self.path.split('?')[0]
        if path.startswith('/api/'):
            self._api_get(path)
        else:
            self._static(path)

    def do_PUT(self):
        path = self.path.split('?')[0]
        if path == '/api/v1/lab':
            try:
                data = self._body()
                LAB_JSON.write_text(json.dumps(data, indent=2) + '\n')
                self._json(200, {'ok': True})
            except Exception as exc:
                self._json(400, {'error': str(exc)})
        else:
            self._json(404, {'error': 'not found'})

    def do_POST(self):
        path = self.path.split('?')[0]
        if path == '/api/v1/run':
            try:
                body = self._body() or {}
                res = runner.start_run(
                    ids=body.get('ids') or None,
                    rebuild=body.get('rebuild', False),
                    include_unsupported=body.get('includeUnsupported', False),
                    sandbox=body.get('sandbox', 'none'))
                self._json(400 if 'error' in res else 200, res)
            except Exception as exc:
                self._json(500, {'error': str(exc)})
        elif path.startswith('/api/v1/run/') and path.endswith('/stop'):
            run_id = path[len('/api/v1/run/'):-len('/stop')]
            self._json(200, {'ok': runner.stop_run(run_id)})
        else:
            self._json(404, {'error': 'not found'})

    def _api_get(self, path):
        if path == '/api/v1/devices':
            self._json(200, probe_serial())
        elif path == '/api/v1/lab':
            self._json(200, json.loads(LAB_JSON.read_text()) if LAB_JSON.exists() else None)
        elif path == '/api/v1/tests':
            cat = runner.load_catalog()
            self._json(200, {'suites': cat['suites'], 'setups': cat['setups'],
                             'targets': cat['targets'], 'preflight': runner.preflight()})
        elif path.startswith('/api/v1/run/'):
            data = runner.get_run(path[len('/api/v1/run/'):])
            self._json(200 if data else 404, data or {'error': 'no such run'})
        else:
            self._json(404, {'error': f'unknown: {path}'})

    def _body(self):
        n = int(self.headers.get('Content-Length', 0))
        return json.loads(self.rfile.read(n)) if n else None

    def _static(self, path):
        if path == '/':
            path = '/index.html'
        candidate = DIST / path.lstrip('/')
        if candidate.is_file():
            data = candidate.read_bytes()
            self._head(200)
            self.send_header('Content-Type', _MIME.get(candidate.suffix, 'application/octet-stream'))
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        # SPA fallback — React Router handles unknown paths
        index = DIST / 'index.html'
        if index.is_file():
            data = index.read_bytes()
            self._head(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            self._json(404, {'error': f'not found: {path} — run: cd frontend && npm run build'})

    def _json(self, code, data):
        body = json.dumps(data).encode()
        self._head(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _head(self, code):
        self.send_response(code)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, PUT, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')


# ── CLI ───────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='esplab', description='ESP-Hosted test lab')
    p.add_argument('subcommand', nargs='?', choices=['probe'], help='probe: list serial devices')
    for axis, choices in AXES.items():
        p.add_argument(f'--{axis}', choices=choices)
    p.add_argument('--scenario', help='named scenario (quick, recovery, rpc_v1, linux_legacy, wifi_sta)')
    p.add_argument('--matrix', help='path to matrix.yaml (reserved)')
    p.add_argument('--flash', choices=['full', 'off'], default='full')
    p.add_argument('--no-reset', action='store_true')
    p.add_argument('--theme', choices=['dark', 'light'], default='dark')
    p.add_argument('--no-mock', action='store_true')
    p.add_argument('--serve', action='store_true', help='start HTTP server, serve built frontend')
    p.add_argument('--port', type=int, default=DEFAULT_PORT)
    p.add_argument('--base-url', default=None)
    p.add_argument('--open', dest='open', action='store_true', default=True)
    p.add_argument('--no-open', dest='open', action='store_false')
    p.add_argument('--headless', action='store_true')
    return p


def to_query(args) -> str:
    q: dict = {}
    for axis in AXES:
        v = getattr(args, axis)
        if v:
            q[axis] = v
    if args.scenario:
        q['scenario'] = args.scenario
    if args.flash != 'full':
        q['flash'] = args.flash
    if args.no_reset:
        q['no-reset'] = '1'
    if args.theme != 'dark':
        q['theme'] = args.theme
    if args.no_mock:
        q['mock'] = '0'
    return urllib.parse.urlencode(q)


def main() -> None:
    args = build_parser().parse_args()

    if args.subcommand == 'probe':
        devices = probe_serial()
        if not devices:
            print('(no serial devices found)')
        for d in devices:
            print(d['path'])
        return

    if args.serve:
        socketserver.TCPServer.allow_reuse_address = True
        url = f'http://localhost:{args.port}'
        query = to_query(args)
        if query:
            url += f'/?{query}'
        print(f'esplab  {url}')
        if args.open and not args.headless:
            webbrowser.open(url)
        with socketserver.ThreadingTCPServer(('', args.port), _Handler) as srv:
            try:
                srv.serve_forever()
            except KeyboardInterrupt:
                pass
        return

    # Dev mode: point at Vite dev server (no local server started)
    base = args.base_url or 'http://localhost:5173'
    query = to_query(args)
    url = f'{base}/?{query}' if query else base
    print(f'esplab  {url}')
    print('Dev mode: cd frontend && npm run dev')
    if args.open and not args.headless:
        webbrowser.open(url)


if __name__ == '__main__':
    main()
