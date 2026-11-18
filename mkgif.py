"""Assemble the PNG frames in logs/frames into one GIF."""
import os
from pathlib import Path
from time import localtime, strftime, time

import imageio.v2 as imageio

import attributes


def frame_number(path):
    digits = "".join(ch for ch in path.stem if ch.isdigit())
    return int(digits) if digits else 0


def main():
    directory = Path("logs/frames")
    paths = sorted(directory.glob("frame*.png"), key=frame_number)
    print("total frames:", len(paths))
    if not paths:
        print("no frames found; enable SHOW_MID_PROCESS first")
        return

    frames = [imageio.imread(path) for path in paths]
    Path("logs/gif").mkdir(parents=True, exist_ok=True)
    stamp = strftime("%Y%m%d-%H%M%S", localtime(time()))
    output = f"logs/gif/traffic-{stamp}.gif"
    if os.path.exists(output):
        base, ext = os.path.splitext(output)
        counter = 1
        while os.path.exists(f"{base}-{counter}{ext}"):
            counter += 1
        output = f"{base}-{counter}{ext}"
    imageio.mimsave(output, frames, fps=attributes.GIF_FPS)
    print("wrote", output)


if __name__ == "__main__":
    main()
