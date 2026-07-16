"""Lab inventory + leasing — the shared-HW allocator (design:
.meta2/changes/eh-testlab-framework/bench-management-and-leasing.md).

emu benches are infinite + private; HW benches are scarce + shared, so they need
an inventory and exclusive, time-boxed leases. This module owns ONLY that:
device/bench inventory and lease acquire/release. It never builds, flashes, or
probes chips (that's build_fw / flasher / hardware). SQLite (stdlib, zero-dep) at
~/.eh/lab.db gives cross-process mutual exclusion via a partial-unique index — no
daemon, no server; leases auto-expire lazily on any access.

A device = one addressable thing (host chip / CP / AP). A bench binds devices by
ROLE into a tuple. A lease grants one owner exclusive use of a bench for a bounded
time. Nothing here assumes which chip plays which role — role is an edge, not a
chip property.
"""
import getpass
import os
import socket
import sqlite3
import time
from pathlib import Path

# Lease durations (seconds). Updated 2026-07-04:
#   MIN_HOLD     = an unspecified `hw acquire` (no --for) holds this — 2h.
#   DEFAULT_HOLD = a test run (`eh.py test hw`) holds this — 4h.
#   MAX_HOLD     = hard cap; renew/want can never exceed — 8h.
MIN_HOLD = 2 * 3600
DEFAULT_HOLD = 4 * 3600
MAX_HOLD = 8 * 3600

_SCHEMA = """
CREATE TABLE IF NOT EXISTS device (
  id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL, chip TEXT, target TEXT,
  rev TEXT, serial_number TEXT, source TEXT NOT NULL DEFAULT 'local',
  port TEXT, attrs TEXT, present INTEGER DEFAULT 1, updated_at INTEGER
);
CREATE TABLE IF NOT EXISTS bench (
  id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL, description TEXT,
  source TEXT NOT NULL DEFAULT 'local', board TEXT, transports TEXT, caps TEXT,
  active INTEGER DEFAULT 1, created_at INTEGER
);
CREATE TABLE IF NOT EXISTS bench_device (
  bench_id INTEGER NOT NULL REFERENCES bench(id) ON DELETE CASCADE,
  device_id INTEGER NOT NULL REFERENCES device(id),
  role TEXT NOT NULL, PRIMARY KEY (bench_id, role)
);
CREATE TABLE IF NOT EXISTS lease (
  id INTEGER PRIMARY KEY, bench_id INTEGER NOT NULL REFERENCES bench(id),
  owner TEXT NOT NULL, reason TEXT, acquired_at INTEGER NOT NULL,
  min_until INTEGER NOT NULL, expires_at INTEGER NOT NULL, max_until INTEGER NOT NULL,
  state TEXT NOT NULL DEFAULT 'active'
);
CREATE UNIQUE INDEX IF NOT EXISTS one_active_lease ON lease(bench_id) WHERE state='active';
"""


def db_path() -> Path:
    """~/.eh/lab.db (per-host: one truth for every checkout on a lab machine).
    Override with EH_LAB_DB (tests point it at a scratch file)."""
    p = os.environ.get("EH_LAB_DB")
    return Path(p) if p else Path.home() / ".eh" / "lab.db"


def connect(path=None) -> sqlite3.Connection:
    p = Path(path) if path else db_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(p), timeout=10)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    conn.executescript(_SCHEMA)
    _expire(conn)
    return conn


def whoami() -> str:
    """Lease owner id — signed off as user@host:pid."""
    try:
        user = getpass.getuser()
    except Exception:
        user = os.environ.get("USER", "unknown")
    return f"{user}@{socket.gethostname()}:{os.getpid()}"


def _expire(conn) -> None:
    """Lazy auto-release: expire any active lease past its deadline. Runs on every
    open, so a crashed run's bench frees itself — no daemon."""
    conn.execute("UPDATE lease SET state='expired' WHERE state='active' AND expires_at < ?",
                 (int(time.time()),))
    conn.commit()


# ── inventory: devices ──────────────────────────────────────────────────
def add_device(conn, name, chip=None, target=None, rev=None, serial_number=None,
               source="local", port=None):
    conn.execute(
        "INSERT INTO device(name,chip,target,rev,serial_number,source,port,updated_at) "
        "VALUES(?,?,?,?,?,?,?,?) ON CONFLICT(name) DO UPDATE SET "
        "chip=excluded.chip,target=excluded.target,rev=excluded.rev,"
        "serial_number=excluded.serial_number,source=excluded.source,"
        "port=excluded.port,present=1,updated_at=excluded.updated_at",
        (name, chip, target, rev, serial_number, source, port, int(time.time())))
    conn.commit()


def list_devices(conn):
    return conn.execute("SELECT * FROM device ORDER BY name").fetchall()


def remove_device(conn, name):
    conn.execute("DELETE FROM device WHERE name=?", (name,))
    conn.commit()


# ── inventory: benches (a bench binds devices by role) ──────────────────
def add_bench(conn, name, roles, description=None, source="local", board=None,
              transports=None, caps=None):
    """roles = {role: device_name}. Upserts the bench and its role→device edges."""
    import json
    cur = conn.execute(
        "INSERT INTO bench(name,description,source,board,transports,caps,created_at) "
        "VALUES(?,?,?,?,?,?,?) ON CONFLICT(name) DO UPDATE SET "
        "description=excluded.description,source=excluded.source,board=excluded.board,"
        "transports=excluded.transports,caps=excluded.caps",
        (name, description, source, board,
         json.dumps(transports or ["sdio"]), json.dumps(caps or []), int(time.time())))
    bid = conn.execute("SELECT id FROM bench WHERE name=?", (name,)).fetchone()["id"]
    conn.execute("DELETE FROM bench_device WHERE bench_id=?", (bid,))
    for role, dev in roles.items():
        row = conn.execute("SELECT id FROM device WHERE name=?", (dev,)).fetchone()
        if not row:
            raise ValueError(f"bench '{name}' role '{role}': no such device '{dev}'")
        conn.execute("INSERT INTO bench_device(bench_id,device_id,role) VALUES(?,?,?)",
                     (bid, row["id"], role))
    conn.commit()
    return bid


def list_benches(conn):
    return conn.execute("SELECT * FROM bench ORDER BY name").fetchall()


def bench_roles(conn, bench_id):
    return conn.execute(
        "SELECT bd.role, d.* FROM bench_device bd JOIN device d ON d.id=bd.device_id "
        "WHERE bd.bench_id=?", (bench_id,)).fetchall()


def get_bench(conn, name):
    return conn.execute("SELECT * FROM bench WHERE name=?", (name,)).fetchone()


def remove_bench(conn, name):
    conn.execute("DELETE FROM bench WHERE name=?", (name,))
    conn.commit()


# ── leasing ─────────────────────────────────────────────────────────────
def _owner_dead(owner):
    """True if the lease owner (user@host:pid) is a process that no longer exists
    ON THIS HOST — i.e. a crashed/killed run whose lease should be reclaimed. A
    lease from another host can't be checked, so it's treated as alive."""
    try:
        host, pid = owner.rsplit("@", 1)[1].rsplit(":", 1)
    except (ValueError, IndexError):
        return False
    if host != socket.gethostname():
        return False
    try:
        os.kill(int(pid), 0)
        return False          # alive
    except ProcessLookupError:
        return True           # gone → stale lease
    except (ValueError, PermissionError, OSError):
        return False


def active_lease(conn, bench_id):
    """The bench's active lease, or None. Reclaims a lease whose owner PID died on
    this host (SIGKILL'd run) so a crash doesn't hold the bench till expiry."""
    l = conn.execute("SELECT * FROM lease WHERE bench_id=? AND state='active'",
                     (bench_id,)).fetchone()
    if l and _owner_dead(l["owner"]):
        conn.execute("UPDATE lease SET state='expired' WHERE id=?", (l["id"],))
        conn.commit()
        return None
    return l


def _device_conflict(conn, bench_id):
    """A device in this bench that is also in another bench holding an active lease
    (device-level exclusion — a shared device can't be double-leased)."""
    return conn.execute(
        "SELECT d.name FROM bench_device bd "
        "JOIN bench_device other ON other.device_id=bd.device_id AND other.bench_id!=bd.bench_id "
        "JOIN lease l ON l.bench_id=other.bench_id AND l.state='active' "
        "JOIN device d ON d.id=bd.device_id WHERE bd.bench_id=? LIMIT 1", (bench_id,)).fetchone()


def pick_free_bench(conn):
    """First active local bench with no active lease — the default when none named."""
    for b in conn.execute("SELECT * FROM bench WHERE active=1 ORDER BY name"):
        if not active_lease(conn, b["id"]) and not _device_conflict(conn, b["id"]):
            return b
    return None


def acquire(conn, bench_name=None, owner=None, want=None, maximum=None, reason=None):
    """Lease a bench exclusively. bench_name=None → first free bench. Returns the
    lease row. Raises if the bench (or a device it binds) is already leased."""
    _expire(conn)
    owner = owner or whoami()
    if bench_name:
        b = get_bench(conn, bench_name)
        if not b:
            raise ValueError(f"no such bench '{bench_name}'")
    else:
        b = pick_free_bench(conn)
        if not b:
            raise RuntimeError("no free bench available")
    if active_lease(conn, b["id"]):
        held = active_lease(conn, b["id"])
        raise RuntimeError(f"bench '{b['name']}' already leased by {held['owner']} "
                           f"until {time.strftime('%H:%M', time.localtime(held['expires_at']))}")
    conflict = _device_conflict(conn, b["id"])
    if conflict:
        raise RuntimeError(f"bench '{b['name']}' shares device '{conflict['name']}' "
                           f"with another active lease")
    now = int(time.time())
    # Unspecified grab → MIN hold (a short default so a forgotten grab frees itself
    # fast); callers that know their budget (e.g. a test run) pass want explicitly.
    hold = int(want) if want else MIN_HOLD
    cap = int(maximum) if maximum else MAX_HOLD
    expires = now + min(hold, cap)
    try:
        conn.execute(
            "INSERT INTO lease(bench_id,owner,reason,acquired_at,min_until,expires_at,"
            "max_until,state) VALUES(?,?,?,?,?,?,?,'active')",
            (b["id"], owner, reason, now, now + MIN_HOLD, expires, now + cap))
        conn.commit()
    except sqlite3.IntegrityError:  # lost the race on the partial-unique index
        raise RuntimeError(f"bench '{b['name']}' was just leased by someone else")
    return conn.execute("SELECT * FROM lease WHERE bench_id=? AND state='active'",
                        (b["id"],)).fetchone()


def release(conn, bench_name=None, owner=None, force=False):
    """Release active lease(s). By default only the caller's own (min_until never
    blocks an owner releasing their own lease — it's a floor on AUTO-expiry, not a
    lock against the holder). force=True releases regardless of owner (admin).
    bench_name=None → all matching."""
    _expire(conn)
    owner = owner or whoami()
    q = "SELECT l.* FROM lease l JOIN bench b ON b.id=l.bench_id WHERE l.state='active'"
    args = []
    if bench_name:
        q += " AND b.name=?"; args.append(bench_name)
    if not force:
        q += " AND l.owner=?"; args.append(owner)
    freed = 0
    for l in conn.execute(q, args).fetchall():
        conn.execute("UPDATE lease SET state='released' WHERE id=?", (l["id"],))
        freed += 1
    conn.commit()
    return freed


def renew(conn, bench_name, owner=None, want=None):
    _expire(conn)
    owner = owner or whoami()
    b = get_bench(conn, bench_name)
    if not b:
        raise ValueError(f"no such bench '{bench_name}'")
    l = active_lease(conn, b["id"])
    if not l or l["owner"] != owner:
        raise RuntimeError(f"you don't hold an active lease on '{bench_name}'")
    new_exp = min(int(time.time()) + (int(want) if want else DEFAULT_HOLD), l["max_until"])
    conn.execute("UPDATE lease SET expires_at=? WHERE id=?", (new_exp, l["id"]))
    conn.commit()
    return new_exp


def status(conn):
    """Every bench with its current lease (owner/expiry) or 'free'."""
    _expire(conn)
    out = []
    for b in list_benches(conn):
        l = active_lease(conn, b["id"])
        out.append({"bench": b["name"], "active": bool(b["active"]),
                    "lease": (dict(l) if l else None)})
    return out
