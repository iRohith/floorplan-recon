"""Single entry point for photo, video, and LiDAR captures."""
from __future__ import annotations
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> None:
    p = argparse.ArgumentParser(prog="floorplan-recon")
    p.add_argument("input", type=Path)
    p.add_argument("--tier", choices=("photo", "video", "lidar"), required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--camera-height", type=float, default=1.45,
                   help="Image-tier metric scale prior in metres; widens intervals")
    p.add_argument("--sam-checkpoint", type=Path, default=Path("tmp/models/sam_vit_b_01ec64.pth"))
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.tier == "lidar":
        from .lidar import run
        run(argparse.Namespace(input=str(args.input), out=str(args.out), state=None,
            no_cache=False, target_frames=1250, target_points=4_500_000,
            sam_checkpoint=str(args.sam_checkpoint)))
        return

    # The RGB tiers deliberately go through the same reconstruction backend.
    # Video is deterministically decoded to stills; per-room photo folders are
    # accepted directly. Geometry is produced by the image adapter and marked
    # with wider uncertainty than LiDAR.
    from .image_tier import run_image_tier
    if args.tier == "photo":
        run_image_tier(args.input, args.out, "photo", args.camera_height)
        return
    with tempfile.TemporaryDirectory(prefix="floorplan-video-") as td:
        frame_dir = Path(td)
        subprocess.run(["ffmpeg", "-loglevel", "error", "-i", str(args.input),
                        "-vf", "fps=2,scale=1280:-2", str(frame_dir / "%06d.jpg")], check=True)
        run_image_tier(frame_dir, args.out, "video", args.camera_height)


if __name__ == "__main__":
    main()
