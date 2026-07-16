#!/usr/bin/env python3
"""vserial <-> eh_frame shim: bridges a Linux vserial host (TLV over a PTY) to an
emulated CP that speaks the eh_frame UART transport over an esp-emu bridge socket.

  Linux c_app (vserial, EH_ESPS_DEV=<pty>) <-> [this shim] <-> esp-emu C6 (--hosted-uart bridge:slave:<sock>)

vserial wire:  [0x01][eplen:2LE][epname][0x02][datalen:2LE][data]   (data == serial payload)
eh_frame wire: esp_payload_header(12B V1) + payload; if_type SERIAL=3, checksum = 16-bit
               byte-sum over header+payload (the shim plays the kmod's wire-framing role).
"""
import os, pty, select, socket, struct, sys, termios, time, tty

SOCK, PTY_LINK = sys.argv[1], sys.argv[2]
# Native (non-legacy) eh_if_type_t on the wire: INVALID=0 STA=1 AP=2 SERIAL=3 HCI=4 PRIV=5
HDR, SERIAL_IF = 12, 3

def eh_encode(payload, seq):
    frame = bytearray(struct.pack('<BBHHHHBB', SERIAL_IF, 0, len(payload),
                                  HDR, 0, seq & 0xFFFF, 0, 0) + payload)
    # Agnostic-host checksum (mirrors the legacy Linux kmod): 16-bit byte-sum over
    # header+payload with the checksum field zeroed. Always sent — a checksum-enabled
    # CP (UART/SPI default) drops frames without it; a disabled CP ignores the field.
    struct.pack_into('<H', frame, 6, sum(frame) & 0xFFFF)   # checksum at byte offset 6
    return bytes(frame)

def eh_parse(buf):
    if len(buf) < HDR: return 0, None, None
    if_byte, _flags, ln, offset, _cksum, _seq, _thr, _u = struct.unpack('<BBHHHHBB', buf[:HDR])
    total = offset + ln
    if total == 0 or offset < HDR or total > 65535: return -1, None, None
    if len(buf) < total: return 0, None, None
    return total, (if_byte & 0x0F), buf[offset:offset + ln]

# esp-emu framed hosted-UART bridge: [op:u8][len:u32 BE][payload]. OP_UART_DATA
# carries the eh_frame byte stream; OP_RESET/OP_WAKE carry reset/wake GPIO edges
# (out-of-band, alongside the data — which is why the framed protocol is needed).
OP_UART_DATA, OP_RESET, OP_WAKE, OP_HOST_SLEEP = 0x01, 0x06, 0x07, 0x08

def op_frame(op, payload=b''):
    return bytes([op]) + struct.pack('>I', len(payload)) + payload

def op_parse(buf):
    if len(buf) < 5: return 0, None, None
    op = buf[0]
    ln = struct.unpack('>I', buf[1:5])[0]
    if len(buf) < 5 + ln: return 0, None, None
    return 5 + ln, op, buf[5:5 + ln]

def vs_wrap(data, ep=b'RPCRsp'):
    return b'\x01' + struct.pack('<H', len(ep)) + ep + b'\x02' + struct.pack('<H', len(data)) + data

def vs_parse(buf):
    if len(buf) < 3 or buf[0] != 0x01: return (-1 if buf and buf[0] != 0x01 else 0), None
    eplen = struct.unpack('<H', buf[1:3])[0]
    q = 3 + eplen
    if len(buf) < q + 3: return 0, None
    if buf[q] != 0x02: return -1, None
    dlen = struct.unpack('<H', buf[q + 1:q + 3])[0]
    end = q + 3 + dlen
    if len(buf) < end: return 0, None
    return end, buf[q + 3:end]

# PTY (raw mode so binary frames pass untouched)
master, slave = pty.openpty()
for fd in (master, slave):
    tty.setraw(fd)
name = os.ttyname(slave)
os.close(slave)
try: os.unlink(PTY_LINK)
except OSError: pass
os.symlink(name, PTY_LINK)
sys.stderr.write(f"[shim] pty {name} -> {PTY_LINK}\n")

# connect to esp-emu bridge socket (it binds; we connect)
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
for _ in range(300):
    try:
        s.connect(SOCK); break
    except OSError:
        time.sleep(0.1)
else:
    sys.stderr.write("[shim] could not connect to esp-emu socket\n"); sys.exit(1)
sys.stderr.write("[shim] connected to esp-emu\n")
s.setblocking(False); os.set_blocking(master, False)

rawb, sb, pb, seq = b'', b'', b'', 0
while True:
    r, _, _ = select.select([s, master], [], [], 2.0)
    if s in r:
        try: d = s.recv(65536)
        except BlockingIOError: d = b''
        rawb += d
        # Layer 1: unwrap the emu's [op][len][payload] bridge frames.
        while rawb:
            n, op, payload = op_parse(rawb)
            if n == 0: break
            rawb = rawb[n:]
            if op == OP_UART_DATA:
                sb += payload  # the eh_frame byte stream rides here
            elif op == OP_WAKE:
                sys.stderr.write("[shim] CP->host OP_WAKE (ignored: linux host does not deep-sleep)\n")
            else:
                sys.stderr.write(f"[shim] CP->host op=0x{op:02x} len={len(payload)} (ignored)\n")
        # Layer 2: parse eh_frames out of the reassembled UART stream.
        while sb:
            n, if_t, pl = eh_parse(sb)
            if n == 0: break
            if n < 0: sb = sb[1:]; continue
            sb = sb[n:]
            if if_t == SERIAL_IF and pl:
                # payload is already a full EPNAME+DATA TLV frame (CP pserial ==
                # vserial framing); pass it straight through to the vserial host.
                os.write(master, pl)
    if master in r:
        try: d = os.read(master, 65536)
        except (BlockingIOError, OSError): d = b''
        pb += d
        while pb:
            n, data = vs_parse(pb)
            if n == 0: break
            if n < 0: pb = pb[1:]; continue
            frame = pb[:n]     # full EPNAME+DATA TLV frame == the CP pserial payload
            pb = pb[n:]
            if data:
                seq += 1
                # eh_frame, then wrapped as an OP_UART_DATA bridge frame.
                s.sendall(op_frame(OP_UART_DATA, eh_encode(frame, seq)))
