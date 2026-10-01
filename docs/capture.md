# Capture protocol and device matrix

## LiDAR — iPhone Pro, Stray Scanner

Install Stray Scanner. Select depth, confidence, RGB, poses and intrinsics. Start at the entrance, hold the phone at chest height, walk each wall clockwise, look into every opening, include floor and ceiling in every room, revisit the entrance to close the loop, then export the original folder without transcoding. Avoid fast turns; pause at mirrors and glossy walls. Typical multi-room capture: 2–4 minutes.

## Video — iPhone 15 or newer

Record one continuous landscape clip at 4K/30. Hold near 1.45 m, walk each perimeter, show every floor-wall and wall-ceiling junction, pause at doors, and revisit the start. Do not zoom or switch lenses.

## Photos — iPhone 15 or newer

Create one folder per room. Take 2–8 landscape images with adjacent views overlapping by roughly half. Stand in multiple corners; include floor and ceiling junctions. Add a connector folder for a hall. Do not use portrait mode or digital zoom.

| Tier | Capture device | Required sensors | Current accuracy statement |
|---|---|---|---|
| Photo | iPhone 15+ | RGB only | experimental; wide intervals, not tape-validated |
| Video | iPhone 15+ | RGB video only | experimental; wide intervals, not tape-validated |
| LiDAR | iPhone Pro with LiDAR | RGB, depth, confidence, poses, intrinsics | metric pipeline runs; gate accuracy not tape-validated |

Processing is local Linux with Python 3.11+, ffmpeg, and optional CUDA for SAM. No hosted service is required.
