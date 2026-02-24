#!/usr/bin/env bash
# Build all PedalKernel VST/CLAP variants and copy to unique bundle names
#
# Single-Pedal Selector variants (WGPU single-pedal editor):
#   - pedalkernel       (free/demo — embedded pedals only, no editor)
#   - pedalkernel-pro   (guitar + bass + amps + single-pedal editor)
#   - pedalkernel-synth (VCO/VCF/VCA synthesizers with MIDI + single-pedal editor)
#
# Pedalboard variant (6-slot chain with WGPU pedalboard editor):
#   - pedalkernel-pedalboard (guitar + bass + amps + pedalboard editor)

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
        # Rename the .so to match the bundle name (required by JUCE-based hosts like Carla)
        for so in "$DIST_DIR/${name}.vst3/Contents/"*-linux/pedalkernel-vst.so; do
            if [ -f "$so" ]; then
                mv "$so" "$(dirname "$so")/${name}.so"
            fi
        done
        for dylib in "$DIST_DIR/${name}.vst3/Contents/MacOS/pedalkernel-vst"; do
            if [ -f "$dylib" ]; then
                mv "$dylib" "$(dirname "$dylib")/${name}"
            fi
        done
        # Fix Info.plist CFBundleExecutable to match renamed binary (required by JUCE-based hosts like Carla)
        plist="$DIST_DIR/${name}.vst3/Contents/Info.plist"
        if [ -f "$plist" ]; then
            sed -i.bak "s|<string>pedalkernel-vst</string>|<string>${name}</string>|g" "$plist"
            rm -f "$plist.bak"
        fi
        # Ensure binary has execute permission (required by some VST3 hosts)
        chmod +x "$DIST_DIR/${name}.vst3/Contents/"*-linux/*.so 2>/dev/null || true
        chmod +x "$DIST_DIR/${name}.vst3/Contents/MacOS/"* 2>/dev/null || true
        log "  -> $DIST_DIR/${name}.vst3"
    fi

    if [ -d "$BUNDLE_DIR/pedalkernel-vst.clap" ]; then
        rm -rf "$DIST_DIR/${name}.clap"
        cp -R "$BUNDLE_DIR/pedalkernel-vst.clap" "$DIST_DIR/${name}.clap"
        chmod +x "$DIST_DIR/${name}.clap/"*.so 2>/dev/null || true
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
        --synth)
            VARIANTS+=("synth")
            shift
            ;;
        --pedalboard)
            VARIANTS+=("pedalboard")
            shift
            ;;
        --all)
            VARIANTS=("free" "pro" "synth" "pedalboard")
            shift
            ;;
        --link)
            LINK_SYSTEM=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo
            echo "Single-Pedal variants:"
            echo "  --free         Build free/demo version (no editor)"
            echo "  --pro          Build pro version (guitar + bass + amps + single-pedal editor)"
            echo "  --synth        Build synth version (VCO/VCF/VCA with MIDI + single-pedal editor)"
            echo
            echo "Pedalboard variant:"
            echo "  --pedalboard   Build 6-slot pedalboard with pedalboard editor"
            echo
            echo "Other:"
            echo "  --all          Build all variants (4 total)"
            echo "  --link         Symlink built plugins to system plugin folders"
            echo "  -h, --help     Show this help"
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
            build_variant "pedalkernel-pro" "pro-guitar,pro-bass,pro-amps,pro-ui" "PedalKernel Pro"
            ;;
        synth)
            build_variant "pedalkernel-synth" "pro-synth,pro-ui" "PedalKernel Synth"
            ;;
        pedalboard)
            build_variant "pedalkernel-pedalboard" "pro-guitar,pro-bass,pro-amps,pro-pedalboard" "PedalKernel Pedalboard"
            ;;
    esac
done

# Optionally install to system plugin folders
if [ "$LINK_SYSTEM" = true ]; then
    log "Copying plugins to system folders..."

    case "$(uname)" in
        Darwin)
            VST3_DIR="$HOME/Library/Audio/Plug-Ins/VST3"
            CLAP_DIR="$HOME/Library/Audio/Plug-Ins/CLAP"
            ;;
        *)
            VST3_DIR="$HOME/.vst3"
            CLAP_DIR="$HOME/.clap"
            ;;
    esac

    mkdir -p "$VST3_DIR" "$CLAP_DIR"

    for vst3 in "$DIST_DIR"/*.vst3; do
        if [ -d "$vst3" ]; then
            name=$(basename "$vst3")
            rm -rf "$VST3_DIR/$name"
            cp -R "$vst3" "$VST3_DIR/$name"
            log "  VST3: $name"
        fi
    done

    for clap in "$DIST_DIR"/*.clap; do
        if [ -d "$clap" ]; then
            name=$(basename "$clap")
            rm -rf "$CLAP_DIR/$name"
            cp -R "$clap" "$CLAP_DIR/$name"
            log "  CLAP: $name"
        fi
    done

    echo
fi

log "All builds complete!"
echo
log "Output directory: $DIST_DIR"
ls -la "$DIST_DIR"
