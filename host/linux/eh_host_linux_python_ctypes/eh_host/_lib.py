from ctypes import CDLL

from eh_host.loader import find_libeh_host

_LIB = CDLL(find_libeh_host())


def _opt_bind(name, argtypes=None, restype=None):
    fn = getattr(_LIB, name, None)
    if fn is not None:
        if argtypes is not None:
            fn.argtypes = argtypes
        if restype is not None:
            fn.restype = restype
    return fn


def _must(fn, name, *args):
    if fn is None:
        raise RuntimeError(
            f"libeh_host.so: symbol '{name}' not built into this .so "
            "(feature likely not enabled in this example's Kconfig)"
        )
    return fn(*args)
