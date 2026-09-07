#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import cv2


def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument('output_dir', type=Path)
    ap.add_argument('--size-px', type=int, default=800)
    args=ap.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id, name in ((10,'rear'),(11,'front')):
        image=cv2.aruco.generateImageMarker(d, marker_id, args.size_px)
        path=args.output_dir/f'aruco-{marker_id}-{name}.png'
        cv2.imwrite(str(path), image)
        print(path)
    return 0
if __name__=='__main__':
    raise SystemExit(main())
