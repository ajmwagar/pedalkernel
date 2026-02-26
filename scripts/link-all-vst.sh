#!/usr/bin/env bash
# Build all PedalKernel variants (release) and symlink to system plugin folders.
# Uses the same variant definitions as build-all-variants.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[PK]${NC} $1"; }
warn() { echo -e "${YELLOW}[PK]${NC} $1"; }

cd "$ROOT_DIR"

# ── Build + bundle all variants ──────────────────────────────────────────
bash "$SCRIPT_DIR/build-all-variants.sh" --all --link

log "Done! All variants built and linked."
