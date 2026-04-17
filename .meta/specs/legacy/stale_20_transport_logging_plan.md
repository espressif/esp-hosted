# Transport Logging & Checksum Control Plan

Status: OPEN
Owner: TBD
Last updated: 2026-03-18

## Goals
- Provide a uniform, bus-agnostic logging policy for transport debug and bus TX/RX dumps.
- Keep checksum behavior dependent on transport preference (per-bus Kconfig).
- Avoid tying log control solely to ESP_LOG component levels; use explicit Kconfig switches.

## Scope
- CP transport implementations: SDIO, SPI, SPI-HD, UART.
- Kconfig controls in CP core/transport Kconfig (global), applied consistently.
- No behavioral changes to data path or buffer lifetime logic.

## Current Baseline
- Bus TX/RX logs: DEBUG (summary), VERBOSE (hex dumps).
- SDIO checksum: per-transport Kconfig.

## Proposed Controls
1) `ESP_HOSTED_TRANSPORT_DEBUG_LOGS` (bool)
   - Enables extra transport debug logs (enqueue/free/outstanding/drop).
2) `ESP_HOSTED_TRANSPORT_BUS_LOG_LEVEL` (choice)
   - None / Error / Warn / Info / Debug / Verbose
   - Applies to bus_tx / bus_rx summary logs and bus hex dumps.

## Checksum Preference Binding (OPEN)
- Keep the runtime checksum behavior controlled by the transport’s own Kconfig
  (e.g., SDIO/SPI/SPI-HD/UART).
- In Kconfig, default `ESP_HOSTED_COMMON_CHECKSUM_ENABLED` to `y` when any transport
  checksum is enabled, so code can uniformly use the common symbol.

## Implementation Steps
1) Add Kconfig switches in CP core (single source of truth).
2) Add shared logging macros in transport header.
3) Update SDIO/SPI/SPI-HD/UART to use macros.
4) Validate that default settings preserve current log behavior.
5) Document in `DOCS_MASTER.md` and update implementation status if needed.

## Open Questions
- Exact naming for Kconfig switches (prefix consistency).
- Whether to split “bus summary” and “bus hex” into two independent levels.
- Whether to surface these in host builds too.
