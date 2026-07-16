"""Minimal 'rich' fallback for minimal Linux hosts (RPi / imx / yocto).

The vendored esp-idf-kconfig backend imports `rich` for pretty report output.
`rich` is not part of a stock distro Python, and eh.py promises zero extra pip
packages. This package provides just enough of the rich API that kconfgen and
menuconfig run — rendering plain text instead of styled tables.

It is added to PYTHONPATH by the build ONLY when the real `rich` is absent
(see tools/cmake/hosted_kconfig.cmake), so a machine that has real rich keeps
the pretty output.
"""


def print(*objects, sep=" ", end="\n", file=None, **kwargs):  # noqa: A001
    import builtins
    builtins.print(sep.join(str(o) for o in objects), end=end, file=file)
