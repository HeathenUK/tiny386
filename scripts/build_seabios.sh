#!/bin/bash
# Build SeaBIOS and VGA BIOS for tiny386
# Source: https://github.com/coreboot/seabios
# Patched per seabios/patch from https://github.com/hchunhui/tiny386
#
# Output: out/bios.bin, out/vgabios.bin

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

mkdir -p "$TOP_DIR/out"
mkdir -p "$TOP_DIR/build"
cd "$TOP_DIR/build"

if [ ! -d seabios ]; then
    git clone https://github.com/coreboot/seabios.git
fi

cd seabios
git checkout -- . 2>/dev/null || true
patch -p1 < "$TOP_DIR/seabios/patch"
cp "$TOP_DIR/seabios/config" .config
make oldconfig
make -j$(nproc)
cp out/bios.bin out/vgabios.bin "$TOP_DIR/out/"
cd "$TOP_DIR"

echo "Built: out/bios.bin out/vgabios.bin"
