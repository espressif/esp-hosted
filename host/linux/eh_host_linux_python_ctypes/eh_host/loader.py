"""Locate libeh_host.so for `ctypes.CDLL` to open.

Search order (first hit wins):
  1. `EH_HOST_LIB` env var — explicit override path.
  2. `${TMPDIR:-/tmp}/eh_libeh_host/libeh_host.so` — output of the
     sibling `build_libeh_host.sh` (developer-mode default).
  3. Standard ld.so search path — for a system-installed
     `libeh_host.so` placed in `/usr/lib`, `/usr/local/lib`, etc.

Packaged installs that ship the .so as `package_data` (per
pyproject.toml) can add a fourth path resolved via
`importlib.resources`.  That path is not exercised today because
the build does not copy the .so into the wheel — left as a future
extension once a wheel-publishing flow exists.
"""

import os
import tempfile


def find_libeh_host() -> str:
    """Return a path to libeh_host.so.

    Raises FileNotFoundError if no candidate is on disk.
    """
    explicit = os.environ.get("EH_HOST_LIB")
    if explicit:
        if not os.path.isfile(explicit):
            raise FileNotFoundError(
                f"EH_HOST_LIB set to {explicit!r} but file does not exist"
            )
        return explicit

    tmp = tempfile.gettempdir()
    dev_path = os.path.join(tmp, "eh_libeh_host", "libeh_host.so")
    if os.path.isfile(dev_path):
        return dev_path

    return "libeh_host.so"
