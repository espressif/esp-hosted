#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import http.server
import ssl
import socket
import os
import sys

def get_local_ip():
    """Get the local IP address of this machine"""
    try:
        # Connect to a remote address (doesn't actually send data)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"

# Check if certificates exist (relative to component certs)
cert_path = '../certs/server_cert.pem'
key_path = '../certs/server_key.pem'

if not os.path.exists(cert_path) or not os.path.exists(key_path):
    print("ERROR: Certificates not found!")
    print(f"Please run: bash create_self_signed_certs.sh")
    print(f"Looking for: {cert_path} and {key_path}")
    print(f"Note: Run this server from components/ota_https/test_server/ directory")
    sys.exit(1)

# Get local IP
local_ip = get_local_ip()
port = 8443

# Create HTTPS server
class CORSHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        super().end_headers()

    def log_message(self, format, *args):
        """Enhanced logging"""
        print(f"[{self.date_time_string()}] {format % args}")

server_address = ('0.0.0.0', port)
httpd = http.server.HTTPServer(server_address, CORSHTTPRequestHandler)

# Configure SSL with proper settings for ESP32
ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.minimum_version = ssl.TLSVersion.TLSv1_2
ssl_context.set_ciphers('ECDHE+AESGCM:ECDHE+CHACHA20:DHE+AESGCM:DHE+CHACHA20:!aNULL:!MD5:!DSS')
ssl_context.load_cert_chain(cert_path, key_path)

# Wrap socket with SSL
httpd.socket = ssl_context.wrap_socket(httpd.socket, server_side=True)

print("=" * 60)
print("HTTPS OTA Server Started Successfully!")
print("=" * 60)
print(f"Server running on: https://{local_ip}:{port}")
print(f"Serving files from: {os.getcwd()}")
print(f"Using certificates:")
print(f"   Cert: {cert_path}")
print(f"   Key:  {key_path}")
print("")
print("ESP32 Configuration:")
print(f"   Set OTA_HTTP_URL to: https://{local_ip}:{port}/firmware.bin")
print("")
print("To test, place your firmware.bin file in this directory")
print("Press Ctrl+C to stop the server")
print("=" * 60)

try:
    httpd.serve_forever()
except KeyboardInterrupt:
    print("\n🛑 Server stopped by user")
    httpd.server_close()
