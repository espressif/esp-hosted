"""Substrate providers (the Target contract, test-architecture.md §4.2).

Each module implements one BenchProvider — emu / linux / serial-HW / (future
wasm) — behind a uniform `make(spec) -> Bench`. A test body only touches
`bench.duts[0]/[1]` and `bench.net`; the provider that produced them is invisible.
A new substrate is one file here, not a fork of the corpus.
"""
