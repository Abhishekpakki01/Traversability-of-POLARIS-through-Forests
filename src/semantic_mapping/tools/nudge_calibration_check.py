#!/usr/bin/env python3
"""Move the green LiDAR outline by eye until it sits on the real hedge and bush tops.

  LEFT/RIGHT/UP/DOWN arrows (or A/D/W/S) move it 1 px; hold SHIFT with A/D/W/S to move 5 px.
  ENTER accepts this frame. K skips a frame if you cannot tell. Q finishes.

Each frame starts at a random offset, so the start does not suggest an answer.
Ignore thin bare branches and sun glare: match the solid hedge and bush shapes.
"""
import argparse
import json
from pathlib import Path
import random

import cv2
import numpy as np

from semantic_mapping.silhouette_alignment import lidar_outline

WINDOW, SCALE = 'Nudge the green outline', 1.6
KEYS = {ord('a'): (-1, 0), ord('d'): (1, 0), ord('w'): (0, -1), ord('s'): (0, 1), ord('A'): (-5, 0), ord('D'): (5, 0),
        ord('W'): (0, -5), ord('S'): (0, 5), 65361: (-1, 0), 65363: (1, 0), 65362: (0, -1), 65364: (0, 1)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', type=Path, required=True, help='folder from prepare_calibration_check.py')
    args = parser.parse_args()
    meta = json.loads((args.input/'frames.json').read_text())
    saved = args.input/'nudges.json'
    results = json.loads(saved.read_text()) if saved.exists() else {}
    names = [f['frame'] for f in meta['frames'] if f['frame'] not in results]
    random.shuffle(names)
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
    stop = False
    for count, name in enumerate(names, 1):
        camera = cv2.imread(str(args.input/name/'camera.jpg'))
        rows = lidar_outline(np.load(args.input/name/'depth.npy'))
        start = [random.randint(-40, 40), random.randint(-20, 20)]
        move = list(start)
        while True:
            picture = cv2.resize(camera, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_LINEAR)
            for x, row in enumerate(rows):
                if np.isfinite(row):
                    point = (int((x+move[0])*SCALE), int((row+move[1])*SCALE))
                    cv2.circle(picture, point, 1, (0, 255, 0), -1, cv2.LINE_AA)
            bar = np.full((40, picture.shape[1], 3), 30, np.uint8)
            cv2.putText(bar, f'Frame {count}/{len(names)}   Arrows or A D W S move   ENTER accept   K skip   Q finish',
                        (10, 27), cv2.FONT_HERSHEY_SIMPLEX, .62, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(WINDOW, np.vstack([picture, bar]))
            key = cv2.waitKeyEx(30)
            if key in KEYS:
                move = [move[0]+KEYS[key][0], move[1]+KEYS[key][1]]
            elif key in (13, 10):
                results[name] = dict(start=start, moved_outline_px=move)
                break
            elif key == ord('k'):
                results[name] = dict(start=start, skipped=True)
                break
            elif key == ord('q') or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                stop = True
                break
        saved.write_text(json.dumps(results, indent=1))
        if stop:
            break
    cv2.destroyAllWindows()
    # The outline was moved by +m to fit the camera, so LiDAR minus camera is -m.
    done = np.array([r['moved_outline_px'] for r in results.values() if 'moved_outline_px' in r], float)
    if len(done) == 0:
        print('No frames accepted yet.')
        return
    offset = -np.median(done, axis=0)
    report = dict(frames=len(done), median_dx_px=float(offset[0]), median_dy_px=float(offset[1]),
                  dx_range_px=[float(-done[:, 0].max()), float(-done[:, 0].min())], by_frame=results)
    (args.input/'nudge_report.json').write_text(json.dumps(report, indent=2))
    print(f'\n{len(done)} frames accepted. Saved {args.input/"nudge_report.json"}')
    print('Tell Claude you have finished; the result is hidden here so that it stays independent.')


if __name__ == '__main__':
    main()
