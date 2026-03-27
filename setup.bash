#!/usr/bin/env bash
# Clone thirdparty dependencies needed before pixi install can resolve.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="$SCRIPT_DIR/thirdparty"

info() { printf '\033[1;34m[INFO]\033[0m  %s\n' "$*"; }

# ── Clone tbai_sdk if needed ──────────────────────────────────────────────
SDK_DIR="$DEPS_DIR/tbai_sdk"
if [ -d "$SDK_DIR" ]; then
    info "tbai_sdk found at $SDK_DIR"
elif [ -n "${TBAI_SDK_DIR:-}" ] && [ -d "$TBAI_SDK_DIR" ]; then
    info "Symlinking tbai_sdk from $TBAI_SDK_DIR"
    mkdir -p "$DEPS_DIR"
    ln -sf "$TBAI_SDK_DIR" "$SDK_DIR"
else
    info "Cloning tbai_sdk..."
    mkdir -p "$DEPS_DIR"
    git clone --depth 1 git@github.com:tbai-lab/tbai_sdk.git "$SDK_DIR"
fi

# ── Clone tbai_mujoco_descriptions if needed ──────────────────────────────
DESC_DIR="$DEPS_DIR/tbai_mujoco_descriptions"
if [ -d "$DESC_DIR" ]; then
    info "tbai_mujoco_descriptions found at $DESC_DIR"
else
    info "Cloning tbai_mujoco_descriptions..."
    mkdir -p "$DEPS_DIR"
    git clone --depth 1 git@github.com:tbai-lab/tbai_mujoco_descriptions.git "$DESC_DIR"
fi

info "Setup complete. Run 'pixi run build' to build."
