#!/bin/bash
# Build and run host_wakeup_demo_using_udp_packet.c
# Sends a wakeup packet to a remote host over TCP or UDP.

# -- Defaults ------------------------------------------------------------------
DEFAULT_PORT="22"
DEFAULT_PROTOCOL="tcp"
IP_ADDRESS=""
PORT="$DEFAULT_PORT"
PROTOCOL="$DEFAULT_PROTOCOL"

# -- Usage ---------------------------------------------------------------------
usage() {
    echo "Usage: $0 --ip <ip_address> [--port <port>] [--protocol <tcp|udp>]"
    echo ""
    echo "Options:"
    echo "  --ip        Target IP address (required)"
    echo "  --port      Target port       (default: $DEFAULT_PORT)"
    echo "  --protocol  tcp | udp         (default: $DEFAULT_PROTOCOL)"
    echo ""
    echo "Examples:"
    echo "  $0 --ip 192.168.1.100"
    echo "  $0 --ip 192.168.1.100 --port 60000 --protocol udp"
    echo "  $0 --ip 192.168.1.100 --port 22 --protocol tcp"
    exit 0
}

# -- Argument parsing ----------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip)
            IP_ADDRESS="$2"; shift 2 ;;
        --port)
            PORT="$2"; shift 2 ;;
        --protocol)
            PROTOCOL="$2"; shift 2 ;;
        --help|-h)
            usage ;;
        *)
            echo "ERROR: Unknown option: $1"
            usage ;;
    esac
done

# -- Validation ----------------------------------------------------------------
if [[ -z "$IP_ADDRESS" ]]; then
    echo "ERROR: --ip is required."
    echo ""
    usage
fi

case "$PROTOCOL" in
    tcp|udp) ;;
    *)
        echo "ERROR: --protocol must be tcp or udp (got: '$PROTOCOL')"
        exit 1 ;;
esac

if ! [[ "$PORT" =~ ^[0-9]+$ ]] || (( PORT < 1 || PORT > 65535 )); then
    echo "ERROR: --port must be a number between 1 and 65535 (got: '$PORT')"
    exit 1
fi

# -- Resolve paths -------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BINARY="$SCRIPT_DIR/host_wakeup_demo_using_udp_packet"

# -- Summary -------------------------------------------------------------------
echo "===== Network Split Host Wakeup Tool ====="
echo "Target:   $IP_ADDRESS:$PORT ($PROTOCOL)"
echo ""

# -- Build ---------------------------------------------------------------------
echo "Building..."
gcc -o "$BINARY" "$SCRIPT_DIR/host_wakeup_demo_using_test_packet.c" -pthread
if [[ $? -ne 0 ]]; then
    echo "FAILED: Compilation failed!"
    exit 1
fi
echo "OK: Build successful!"
echo ""

# -- Run -----------------------------------------------------------------------
echo "Sending wakeup packet..."
echo "-------------------------------------------"
"$BINARY" "$IP_ADDRESS" "$PORT" "$PROTOCOL"
RESULT=$?
echo "-------------------------------------------"

if [[ $RESULT -eq 0 ]]; then
    echo "OK: Wakeup packet sent successfully!"
else
    echo "FAILED: Exit code $RESULT"
fi

exit $RESULT
