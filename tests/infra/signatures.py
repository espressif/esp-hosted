"""Firmware crash signatures — the one place that knows what a CP/host panic
looks like on the wire. Add a new signature HERE, not in the consumers.

Two lists on purpose — different scan contexts, not an oversight:

* EXPECT_FATAL — checked as `not_matching` inside a BOUNDED expect window
  (infra.expect_helper). Includes bare 'panic': within a short window around a
  specific expectation it is unambiguous.
* SETTLE_FATAL — scanned across the WHOLE post-success buffer
  (infra.emu_dut.settle_check / fatal()). EXCLUDES bare 'panic' (it matches
  benign handler-registration logs when you scan an entire boot buffer) and
  prefers more specific fault names.
"""

# Bounded-window fail patterns (expect_helper default not_matching).
EXPECT_FATAL = [
    'Guru Meditation Error',
    'assert failed',
    'panic',
    'abort()',
    'LoadProhibited',
    'StoreProhibited',
    'Illegal instruction',
    'Stack smashing',
]

# Full-buffer late-crash signatures (settle_check). Bare 'panic' intentionally
# absent — it matches benign logs across a whole buffer; these do not.
SETTLE_FATAL = (
    'Guru Meditation',
    'abort() was called',
    'assert failed',
    'CORRUPT HEAP',
    'LoadProhibited',
    'StoreProhibited',
    'Instruction access fault',
    'Backtrace:',
)
