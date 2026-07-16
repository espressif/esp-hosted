#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
repo="$(git -C "${here}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${repo}" ]; then
    repo="$(cd "${here}/../../../../.." && pwd)"
fi

exec "${repo}/host/linux/eh_host_linux_kmod/scripts/load.sh" "$@"
