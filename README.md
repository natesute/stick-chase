# Stick Chase

A stick figure that lives on your Mac's screen and parkours across windows, buttons, icons and
panels trying to catch your mouse cursor. Once he has it he hangs off it or rides on top; shake the
mouse to fling him off.

## Build and run

```bash
./build.sh
open "Stick Chase.app"
```

Requires macOS 13+ and the Xcode command line tools. The build signs with an Apple Development
identity if one is installed (so the Screen Recording permission survives rebuilds), otherwise ad-hoc.

## Screen vision

With Screen Recording permission he treats edges found in the screen image as ledges and walls.
Without it he only uses window frames. Grant it from the menu bar icon ("Enable Screen Vision…"),
then choose "Restart Stick Chase". Captured frames are only used for edge detection and never saved.

## Dev tools

```bash
build/StickChase --selftest                 # planner timing + headless 8-minute chase
build/StickChase --snapshot poses.png       # pose sheet
build/StickChase --film film.png [--alt]    # contact sheet of a simulated chase
build/StickChase --vision-test in.png out.png   # edge detection on a screenshot
STICKCHASE_LOG=1 "Stick Chase.app/Contents/MacOS/StickChase"   # log state twice a second
```
