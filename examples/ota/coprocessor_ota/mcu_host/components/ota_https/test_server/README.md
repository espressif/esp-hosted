# Test Server Tools

This directory contains tools for testing HTTPS OTA functionality with self-signed certificates.

## Files

- `create_self_signed_certs.sh` - Generates self-signed certificates for testing
- `create_https_server.py` - Simple HTTPS server for serving OTA files
- `README.md` - This file

## Quick Start

### 1. Generate Test Certificates

```bash
cd test_server
bash create_self_signed_certs.sh
```

### 2. Start HTTPS Server

```bash
cd test_server
python3 create_https_server.py
```

### 3. Configure ESP32

```bash
# From project root
eh.py menuconfig
```

Enable in menuconfig:

- `Component config` → `HTTPS OTA Configuration` → `Use self-signed certificate (Testing Only)` → **YES**
- Update `OTA Server URL` to the URL shown by the server

## Important Notes

- **Testing Only**: Self-signed certificates should only be used for testing
- **Security**: For production, use proper CA-signed certificates
- **IP Address**: The certificate is generated for your current IP address
- **Regenerate**: If your IP changes, regenerate certificates

## Troubleshooting

### Certificate not found error

```bash
# Make sure you're in test_server directory
cd test_server
bash create_self_signed_certs.sh
```

### Server won't start

- Check if port 8443 is available
- Ensure certificates exist in `../certs/`

```
ls ../certs/
server_cert.pem  server_key.pem
```

- Run from `test_server/` directory
