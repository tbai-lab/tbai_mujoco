#!/usr/bin/env bash
# Set up tbai_mujoco: find or clone tbai_sdk, build its dependencies, install into pixi env.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="$SCRIPT_DIR/thirdparty"
SDK_DIR="$DEPS_DIR/tbai_sdk"
DESCRIPTIONS_DIR="$DEPS_DIR/tbai_mujoco_descriptions"

info() { printf '\033[1;34m[INFO]\033[0m  %s\n' "$*"; }

# ── Find or clone tbai_sdk ────────────────────────────────────────────────
if [ -d "$SDK_DIR" ]; then
    info "tbai_sdk found at $SDK_DIR"
elif [ -n "${TBAI_SDK_DIR:-}" ] && [ -d "$TBAI_SDK_DIR" ]; then
    info "Symlinking tbai_sdk from $TBAI_SDK_DIR"
    mkdir -p "$DEPS_DIR"
    ln -sf "$TBAI_SDK_DIR" "$SDK_DIR"
else
    info "Cloning tbai_sdk..."
    mkdir -p "$DEPS_DIR"
    git clone git@github.com:tbai-lab/tbai_sdk.git "$SDK_DIR"
fi

# ── Find or clone tbai_mujoco_descriptions ────────────────────────────
if [ -d "$DESCRIPTIONS_DIR" ]; then
    info "tbai_mujoco_descriptions found at $DESCRIPTIONS_DIR"
else
    info "Cloning tbai_mujoco_descriptions..."
    mkdir -p "$DEPS_DIR"
    git clone git@github.com:tbai-lab/tbai_mujoco_descriptions.git "$DESCRIPTIONS_DIR"
fi

# ── Build Zenoh (tbai_sdk dependency) ─────────────────────────────────────
info "Setting up Zenoh..."
cd "$SDK_DIR" && bash scripts/setup_zenoh.bash

# ── Install pixi environment ─────────────────────────────────────────────
cd "$SCRIPT_DIR"
info "Installing pixi environment..."
pixi install

info "Setup complete. Run 'pixi run build' to build."
