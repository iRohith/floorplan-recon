"""Single command surface for photo, video, and LiDAR captures."""
from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path


def main() -> None:
    p = argparse.ArgumentParser(prog="floorplan")
    p.add_argument("input", type=Path)
    p.add_argument("--tier", choices=("photo", "video", "lidar"), required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--camera-height", type=float, default=1.45,
                   help="declared capture height in metres; the metric scale prior for RGB tiers")
    p.add_argument("--sam-checkpoint", type=Path, default=Path("models/sam_vit_b_01ec64.pth"))
    p.add_argument("--no-cache", action="store_true")
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.tier == "lidar":
        from .lidar import run
        run(argparse.Namespace(input=str(args.input), out=str(args.out), state=None,
            no_cache=args.no_cache, target_frames=1250, target_points=4_500_000,
            sam_checkpoint=str(args.sam_checkpoint)))
        return

    from .image_tier import run_image_tier
    if args.tier == "photo":
        run_image_tier(args.input, args.out, "photo", args.camera_height)
        return

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise SystemExit("ffmpeg is required for the video tier")
    with tempfile.TemporaryDirectory(prefix="floorplan-video-") as td:
        frames = Path(td) / "room"
        frames.mkdir(parents=True)
        # Deterministic decode: fixed rate and width, no timestamps in the path.
        subprocess.run([ffmpeg, "-loglevel", "error", "-i", str(args.input),
                        "-vf", "fps=2,scale=1280:-2", str(frames / "%06d.jpg")], check=True)
        run_image_tier(Path(td), args.out, "video", args.camera_height)


if __name__ == "__main__":
    main()
