# esplab

ESP-Hosted test lab — configure devices, plan and run test scenarios.
The UI is the human interface; every action has a CLI equivalent.

**Current state**:
- Frontend: a real **Tests** console (tab `4`, default landing) that drives actual
  emulator runs, plus a Set-up page (device/link CRUD) and a mock Run/Reports
  sequencer kept as a sequence visualiser.
- `esplab.py` serves the built bundle and exposes `/api/v1/` (device probe, lab
  persist, the entity tree, and the run API).
- **Real execution** lands via `runner.py`: it clean-builds each scenario's C6
  coprocessor + P4 host, runs them through the [esp-emulator](https://) SDIO
  bridge (P4↔C6, reset GPIO 54, wake GPIO 2), applies any UART stimulus, and
  asserts each case (expected log lines present, crash markers absent).
  `runner.py` is standalone — usable with or without the UI.

## The entity model (`tests.json`)

Everything is an entity differentiated only by its **properties** — the same row
renders a positive functional test, a negative robustness test, or an unsupported
one. The tree is **Suites ▸ Scenarios ▸ Cases**:

- a **scenario** is one emulator run (one C6+host build pair + optional stimulus),
  with props `kind` (positive | negative), `category` (functional | memory |
  performance | recovery), and `support` (`supported` | `unsupported:<reason>` |
  `tbd:<reason>`);
- a **case** is one assertion over that run's logs: `expect` substrings must
  appear and `absent` crash markers must not, within `timeout_s`.

Scenarios reflect **what *should* be tested** (the broad vision), not only what is
doable today — anything not yet runnable carries `support: unsupported`/`tbd` with
a reason, so the gap is visible instead of missing. `support` gates execution.

## Real runner (C6 ↔ P4 on the emulator)

`runner.py` builds + emulates + asserts. Prereqs: `esp-emu` on disk and ESP-IDF.
```sh
export ESP_EMU_BIN=<esp-emulator>/target/release/esp-emu   # or: eh.py set-esp-emu <dir>
export IDF_PATH=<esp-idf>                                  # or: eh.py set-idf-path <dir>

python3 runner.py list                       # show the Suites▸Scenarios▸Cases tree
python3 runner.py run wifi_sta               # run a suite (all its scenarios)
python3 runner.py run sta_connect            # run one scenario (positive)
python3 runner.py run sta_wrong_password     # run one negative scenario
python3 runner.py run sta_connect --no-clean # reuse build, fast iterate
python3 runner.py run --all                  # whole sweep -> writes coverage.md
python3 runner.py run --all --include-unsupported     # also try unsupported/tbd
python3 runner.py run sta_connect --sandbox bwrap     # run inside a bubblewrap sandbox
```

### Optional sandbox (`--sandbox bwrap`)

Off by default. With `--sandbox bwrap` (or the **sandbox** toggle in the UI) each
build + emulate runs inside a [bubblewrap](https://github.com/containers/bubblewrap)
namespace so the **main system stays untouched**: whole root read-only, private
`/tmp`, the scratch workspace bound read-write, and `--unshare-net` during emulation
(the emulator's `--net user` is in-process, and the SDIO bridge is a Unix socket, so
no host network is needed). Overhead is negligible — bubblewrap is namespaces only,
~5-10 MB and ~ms of startup, no root, no daemon. If bubblewrap is missing or user
namespaces are blocked (e.g. already inside a restricted container), it logs a notice
and **falls back to native** — the run still proceeds. Execution stays sequential, so
peak memory is bounded to one build + one emulator pair regardless of sandboxing.
An id resolves to a **suite**, a **scenario**, or a **case** (the owning scenario
runs). Each run writes per-stream logs under `runs/<timestamp>/` and the sweep
writes `coverage.md`. From the UI, the **Tests** console calls the same engine over
HTTP and shows live CP/Host/Run phases + per-case verdicts + host/cp/build logs.

**Examples are read-only inputs.** Tests never modify the example tree. Each build
copies the example into a scratch workspace (`.work/`, gitignored) and builds there
— the overlay, `build/`, `sdkconfig`, `managed_components` and merged bin all live
in scratch (relative `override_path:` deps are rewritten to absolute on copy so the
copy still points at the in-repo components). What/how/when to test is scoped to the
manifest + runner; an example only ever supplies source. Scenarios may be *inspired*
by an example but never depend on editing one.

## Quick start

**Production mode** (built bundle + real API):
```sh
cd tools/esplab
npm --prefix frontend install
npm --prefix frontend run build
./esplab.py --serve          # starts server on :8321, opens browser
```

**Dev mode** (Vite HMR + API proxy to :8321):
```sh
# terminal 1 — backend (API only, no static serving)
python tools/esplab/esplab.py --serve --no-open

# terminal 2 — frontend (HMR)
cd tools/esplab/frontend && npm run dev   # http://localhost:5173
```

**CLI tools**:
```sh
./esplab.py probe                         # list connected serial/USB devices
./esplab.py --serve --no-open             # server only, no browser
./esplab.py --scenario recovery           # open dev server with preset
```

## CLI ⇄ UI (1:1)

| Switch | UI control |
|---|---|
| `--env {esp_sim,hw}` | env chips |
| `--host`, `--cp`, `--transport`, `--rpc` | config axes |
| `--scenario <id>` | scenario picker preset |
| `--flash {full,off}` / `--no-reset` | flash behaviour |
| `--theme {dark,light}` | theme toggle |
| `--no-mock` | use real backend |
| `--port`, `--serve`, `--open/--no-open`, `--headless` | serve / launch |

## API

| Route | Purpose |
|---|---|
| `GET /api/v1/lab` | load persisted lab state (`lab.json`) |
| `PUT /api/v1/lab` | save lab state |
| `GET /api/v1/devices` | enumerate connected serial/USB ports |
| `GET /api/v1/tests` | entity tree (`suites`, `setups`, `targets`) + `preflight` |
| `POST /api/v1/run` | start a run `{ids?, clean?, includeUnsupported?}` → `{runId}` |
| `GET /api/v1/run/<id>` | live run state: per-scenario phases, case verdicts, logs |
| `POST /api/v1/run/<id>/stop` | stop a run |

## Extend

- **Entity tree** — `tests.json` (suites ▸ scenarios ▸ cases; `kind`/`category`/`support`/`stimulus`/`assert`).
- **Runner engine** — `runner.py` (`build_side`, `emulate`, `run_scenario`, `RunJob`); standalone CLI too.
- **Tests console UI** — `frontend/src/components/ExamplesPage.tsx`.
- **Test matrix (mock sequencer)** — `frontend/src/config/matrix.ts` (`AXES`, `SCENARIOS`, `evalCompat`).
- **Device model** — `frontend/src/config/lab.ts` (`LabState`, `LabDevice`, `Link`).
- **API / CLI** — `esplab.py`; mirror new API routes as subcommands/flags.
