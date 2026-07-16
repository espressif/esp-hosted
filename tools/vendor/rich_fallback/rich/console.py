"""rich.console fallback — Console(...).print(obj) writes plain text."""


class Console:
    def __init__(self, *args, **kwargs):
        self._stderr = kwargs.get("stderr", False)

    def print(self, *objects, sep=" ", end="\n", **kwargs):
        import sys
        stream = sys.stderr if self._stderr else sys.stdout
        stream.write(sep.join(str(o) for o in objects) + end)
