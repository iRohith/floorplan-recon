"""Single command surface for photo, video, and LiDAR captures."""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def _write_unavailable(out: Path, tier: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    status = {
        "tier": tier,
        "status": "not_implemented",
        "error": "RGB-only metric reconstruction is not implemented in this release.",
        "reason": "Camera-height priors cannot satisfy whole-property scale, adjacency, or calibration gates.",
    }
    (out / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    raise SystemExit(2)


def main() -> None:
    p = argparse.ArgumentParser(prog="floorplan-recon")
    p.add_argument("input", type=Path)
    p.add_argument("--tier", choices=("photo", "video", "lidar"), required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--sam-checkpoint", type=Path, default=Path("models/sam_vit_b_01ec64.pth"))
    p.add_argument("--no-cache", action="store_true")
    args = p.parse_args()
    if args.tier != "lidar":
        _write_unavailable(args.out, args.tier)
    from .lidar import run
    run(argparse.Namespace(input=str(args.input), out=str(args.out), state=None,
        no_cache=args.no_cache, target_frames=1250, target_points=4_500_000,
        sam_checkpoint=str(args.sam_checkpoint)))


if __name__ == "__main__":
    main()
