"""rich.table fallback — collects rows and renders them as plain text.

Covers the API the esp-idf-kconfig report uses: Table(title=..., ...),
.add_column(...), .add_row(*cells, style=...), .box = ..., and str(table)
(tables are nested as rows of an outer table, then printed)."""


class Table:
    def __init__(self, *args, title=None, **kwargs):
        self._title = title
        self._rows = []
        self.box = None

    def add_column(self, *args, **kwargs):
        pass

    def add_row(self, *cells, **kwargs):
        self._rows.append(" ".join(str(c) for c in cells if c != ""))

    def __str__(self):
        out = [str(self._title)] if self._title else []
        out.extend(r for r in self._rows if r)
        return "\n".join(out)
