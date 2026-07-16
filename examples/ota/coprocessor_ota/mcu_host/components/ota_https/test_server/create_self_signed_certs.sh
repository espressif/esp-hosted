#!/bin/bash

# Create certificates for HTTPS OTA testing
# This script should be run from the component's test_server directory

# Create a directory for certificates in the component's certs folder
mkdir -p ../certs
cd ../certs

# Get your machine's IP address
IP_ADDRESS=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || echo "192.168.1.100")
echo "Creating certificate for IP: $IP_ADDRESS"
echo "If this IP is incorrect, please update the CN field manually"

# Generate private key
openssl genrsa -out server_key.pem 2048

# Generate self-signed certificate (valid for 365 days)
# Use your actual IP address as Common Name
openssl req -new -x509 -key server_key.pem -out server_cert.pem -days 365 \
    -subj "/C=US/ST=State/L=City/O=ESP-OTA/CN=$IP_ADDRESS" \
    -extensions v3_req \
    -config <(cat <<EOF
[req]
distinguished_name = req_distinguished_name
req_extensions = v3_req
prompt = no

[req_distinguished_name]
C = US
ST = State
L = City
O = ESP-OTA
CN = $IP_ADDRESS

[v3_req]
basicConstraints = CA:FALSE
keyUsage = nonRepudiation, digitalSignature, keyEncipherment
subjectAltName = @alt_names

[alt_names]
IP.1 = $IP_ADDRESS
IP.2 = 127.0.0.1
DNS.1 = localhost
EOF
)

# View certificate details
echo "Certificate created with the following details:"
openssl x509 -in server_cert.pem -text -noout | grep -A 5 "Subject:"
openssl x509 -in server_cert.pem -text -noout | grep -A 10 "Subject Alternative Name"

echo ""
echo "IMPORTANT: Your HTTPS server must run on IP: $IP_ADDRESS"
echo "Update your Kconfig OTA_HTTP_URL to: https://$IP_ADDRESS:8443/firmware.bin"
