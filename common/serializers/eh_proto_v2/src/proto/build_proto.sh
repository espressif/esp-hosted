#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTO_FILE="${SCRIPT_DIR}/rpc_v2.proto"

ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

OUT_H="${ROOT_DIR}/include/gen_v2.h"
OUT_C="${ROOT_DIR}/src/gen_v2.c"

TMP_DIR="$(mktemp -d)"

cleanup() {
    rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

if ! command -v protoc >/dev/null 2>&1; then
    echo "Please install protobuf"
    exit 1
fi

if ! command -v protoc-gen-c >/dev/null 2>&1; then
    echo "Please install protobuf-c"
    exit 1
fi

echo "Generating protobuf sources..."

protoc \
    -I"${SCRIPT_DIR}" \
    --c_out="${TMP_DIR}" \
    "${PROTO_FILE}"

GEN_H="${TMP_DIR}/rpc_v2.pb-c.h"
GEN_C="${TMP_DIR}/rpc_v2.pb-c.c"

if [[ ! -f "${GEN_H}" || ! -f "${GEN_C}" ]]; then
    echo "Generated files missing"
    exit 1
fi

mkdir -p "$(dirname "${OUT_H}")"
mkdir -p "$(dirname "${OUT_C}")"

mv -f "${GEN_H}" "${OUT_H}"
mv -f "${GEN_C}" "${OUT_C}"

# Rewrite include to match the renamed header.
sed -i.bak \
    -e 's|#include "rpc_v2.pb-c.h"|#include "gen_v2.h"|' \
    "${OUT_C}"
rm -f "${OUT_C}.bak"

echo "Generated successfully:"
echo "  ${OUT_H}"
echo "  ${OUT_C}"
