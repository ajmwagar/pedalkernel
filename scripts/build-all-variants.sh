#!/usr/bin/env bash
# Build all PedalKernel VST/CLAP variants and copy to unique bundle names
#
# Variants:
#   - pedalkernel (free/demo)
#   - pedalkernel-pro (guitar + bass + amps + synth + shared)
#   - pedalkernel-guitar (guitar + shared)
#   - pedalkernel-bass (bass + shared)
#   - pedalkernel-amps (amp models + shared)
#   - pedalkernel-synth (VCO/VCF/VCA synthesizers)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
BUNDLE_DIR="$ROOT_DIR/target/bundled"
DIST_DIR="$ROOT_DIR/target/dist"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[BUILD]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
    exit 1
}

# Create dist directory
mkdir -p "$DIST_DIR"

build_variant() {
    local name="$1"
    local features="$2"
    local display_name="$3"

    log "Building $display_name..."

    # Clean previous build (thorough clean to avoid feature caching issues)
    cargo clean -p pedalkernel-vst 2>/dev/null || true
    rm -rf "$ROOT_DIR/target/release/.fingerprint/pedalkernel-vst-"* 2>/dev/null || true
    rm -rf "$ROOT_DIR/target/release/deps/libpedalkernel_vst"* 2>/dev/null || true
    rm -rf "$ROOT_DIR/target/release/deps/pedalkernel_vst"* 2>/dev/null || true

    # Build
    if [ -n "$features" ]; then
        cargo build --release -p pedalkernel-vst --features "$features"
    else
        cargo build --release -p pedalkernel-vst
    fi

    # Bundle
    if [ -n "$features" ]; then
        cargo xtask bundle pedalkernel-vst --release --features "$features"
    else
        cargo xtask bundle pedalkernel-vst --release
    fi

    # Copy to dist with unique name (remove existing first to ensure full overwrite)
    if [ -d "$BUNDLE_DIR/pedalkernel-vst.vst3" ]; then
        rm -rf "$DIST_DIR/${name}.vst3"
        cp -R "$BUNDLE_DIR/pedalkernel-vst.vst3" "$DIST_DIR/${name}.vst3"
        log "  -> $DIST_DIR/${name}.vst3"
    fi

    if [ -d "$BUNDLE_DIR/pedalkernel-vst.clap" ]; then
        rm -rf "$DIST_DIR/${name}.clap"
        cp -R "$BUNDLE_DIR/pedalkernel-vst.clap" "$DIST_DIR/${name}.clap"
        log "  -> $DIST_DIR/${name}.clap"
    fi

    log "$display_name complete!"
    echo
}

# Parse arguments
VARIANTS=()
LINK_SYSTEM=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --free)
            VARIANTS+=("free")
            shift
            ;;
        --pro)
            VARIANTS+=("pro")
            shift
            ;;
        --guitar)
            VARIANTS+=("guitar")
            shift
            ;;
        --bass)
            VARIANTS+=("bass")
            shift
            ;;
        --amps)
            VARIANTS+=("amps")
            shift
            ;;
        --synth)
            VARIANTS+=("synth")
            shift
            ;;
        --all)
            VARIANTS=("free" "pro" "guitar" "bass" "amps" "synth")
            shift
            ;;
        --link)
            LINK_SYSTEM=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo
            echo "Options:"
            echo "  --free      Build free/demo version"
            echo "  --pro       Build pro version (guitar + bass + amps + synth + shared)"
            echo "  --guitar    Build guitar-only version"
            echo "  --bass      Build bass-only version"
            echo "  --amps      Build amp models version"
            echo "  --synth     Build synth version (VCO/VCF/VCA)"
            echo "  --all       Build all variants"
            echo "  --link      Symlink built plugins to system plugin folders"
            echo "  -h, --help  Show this help"
            echo
            echo "If no variant is specified, builds --pro by default."
            echo
            echo "Output: target/dist/"
            exit 0
            ;;
        *)
            error "Unknown option: $1"
            ;;
    esac
done

# Default to pro if no variant specified
if [ ${#VARIANTS[@]} -eq 0 ]; then
    VARIANTS=("pro")
fi

cd "$ROOT_DIR"

log "Building PedalKernel variants: ${VARIANTS[*]}"
echo

# Build each requested variant
for variant in "${VARIANTS[@]}"; do
    case $variant in
        free)
            build_variant "pedalkernel" "" "PedalKernel (Free)"
            ;;
        pro)
            build_variant "pedalkernel-pro" "pro-pedals" "PedalKernel Pro"
            ;;
        guitar)
            build_variant "pedalkernel-guitar" "pro-guitar" "PedalKernel Guitar"
            ;;
        bass)
            build_variant "pedalkernel-bass" "pro-bass" "PedalKernel Bass"
            ;;
        amps)
            build_variant "pedalkernel-amps" "pro-amps" "PedalKernel Amps"
            ;;
        synth)
            build_variant "pedalkernel-synth" "pro-synth" "PedalKernel Synth"
            ;;
    esac
done

# Optionally link to system plugin folders
if [ "$LINK_SYSTEM" = true ]; then
    log "Linking plugins to system folders..."

    VST3_DIR="$HOME/Library/Audio/Plug-Ins/VST3"
    CLAP_DIR="$HOME/Library/Audio/Plug-Ins/CLAP"

    mkdir -p "$VST3_DIR" "$CLAP_DIR"

    for vst3 in "$DIST_DIR"/*.vst3; do
        if [ -d "$vst3" ]; then
            name=$(basename "$vst3")
            rm -rf "$VST3_DIR/$name"
            ln -sf "$vst3" "$VST3_DIR/$name"
            log "  VST3: $name"
        fi
    done

    for clap in "$DIST_DIR"/*.clap; do
        if [ -d "$clap" ]; then
            name=$(basename "$clap")
            rm -rf "$CLAP_DIR/$name"
            ln -sf "$clap" "$CLAP_DIR/$name"
            log "  CLAP: $name"
        fi
    done

    echo
fi

log "All builds complete!"
echo
log "Output directory: $DIST_DIR"
ls -la "$DIST_DIR"
