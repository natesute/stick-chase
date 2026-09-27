#!/bin/zsh
# Builds "Stick Chase.app" next to this script.
set -euo pipefail
cd "$(dirname "$0")"

APP="Stick Chase.app"
mkdir -p build
swiftc -O -swift-version 5 -o build/StickChase main.swift -framework Cocoa

rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS"
cp build/StickChase "$APP/Contents/MacOS/StickChase"
cat > "$APP/Contents/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key><string>Stick Chase</string>
    <key>CFBundleDisplayName</key><string>Stick Chase</string>
    <key>CFBundleIdentifier</key><string>com.natesute.stickchase</string>
    <key>CFBundleExecutable</key><string>StickChase</string>
    <key>CFBundlePackageType</key><string>APPL</string>
    <key>CFBundleShortVersionString</key><string>1.0</string>
    <key>CFBundleVersion</key><string>1</string>
    <key>LSMinimumSystemVersion</key><string>13.0</string>
    <key>LSUIElement</key><true/>
    <key>NSHighResolutionCapable</key><true/>
    <key>NSScreenCaptureUsageDescription</key><string>Stick Chase looks at the screen to find ledges and walls to climb.</string>
</dict>
</plist>
PLIST
# A stable signing identity keeps the Screen Recording permission across rebuilds.
IDENTITY=$(security find-identity -v -p codesigning | grep -o '"Apple Development: hi@nathansuttie.com[^"]*"' | head -1 | tr -d '"')
codesign --force --timestamp=none --sign "${IDENTITY:--}" "$APP" >/dev/null
echo "Signed with: ${IDENTITY:-ad-hoc}"
echo "Built $PWD/$APP"
