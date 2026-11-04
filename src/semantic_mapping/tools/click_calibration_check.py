#!/usr/bin/env python3
"""Click matching points in the camera and LiDAR views to measure their alignment.

  Click a sharp point in the camera view (a corner of a bush or hedge outline against the sky).
  The view flips to LiDAR: click the same point there. Aim for about 3 points per frame.

  SPACE flip view   U undo   N next frame   P previous frame   Q finish and write the report

Clicks save automatically, so you can stop and continue later.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from semantic_mapping.calibration_check import ClickSession, depth_at, summarise_pairs

WINDOW, SCALE, ZOOM, LENS = 'Calibration check', 1.6, 5, 90


def report(folder, session, focal):
    if not session.pairs:
        print('No pairs clicked yet.')
        return
    result = summarise_pairs(session.pairs, focal)
    result['by_frame'] = {f: summarise_pairs([p for p in session.pairs if p['frame'] == f], focal)['median_error_px']
                          for f in sorted({p['frame'] for p in session.pairs})}
    (folder/'report.json').write_text(json.dumps(result, indent=2))
    x, y = result['mean_offset_px']
    print(f"\n{result['pairs']} pairs. Median error {result['median_error_px']:.1f} px, "
          f"90% {result['p90_error_px']:.1f} px.")
    print(f'Mean offset, LiDAR minus camera: {x:+.1f} px across, {y:+.1f} px down.')
    print(f"Saved {folder/'report.json'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', type=Path, required=True, help='folder from prepare_calibration_check.py')
    args = parser.parse_args()
    meta = json.loads((args.input/'frames.json').read_text())
    names = [f['frame'] for f in meta['frames']]
    images = {n: dict(camera=cv2.imread(str(args.input/n/'camera.jpg')), lidar=cv2.imread(str(args.input/n/'lidar.png')),
                      depth=np.load(args.input/n/'depth.npy')) for n in names}
    saved = args.input/'clicks.json'
    session = ClickSession(json.loads(saved.read_text()) if saved.exists() else None)
    state = dict(index=0, view='camera', cursor=(0, 0), message='Click a sharp point on an outline against the sky')

    def on_mouse(event, x, y, flags, _):
        pixel = (x/SCALE, y/SCALE)
        state['cursor'] = pixel
        if event != cv2.EVENT_LBUTTONDOWN or y >= int(544*SCALE):
            return
        name = names[state['index']]
        depth = depth_at(images[name]['depth'], pixel) if state['view'] == 'lidar' else None
        state['message'] = session.click(state['view'], name, pixel, depth)
        if state['view'] == 'camera':
            state['view'] = 'lidar'
        elif session.pending is None and state['message'] == 'Saved':
            saved.write_text(json.dumps(session.pairs, indent=1))
            state['view'] = 'camera'

    def draw():
        name = names[state['index']]
        base = images[name][state['view']]
        frame = cv2.resize(base, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_LINEAR)
        for pair in session.pairs:
            if pair['frame'] == name:
                u, v = (int(c*SCALE) for c in pair[state['view']])
                cv2.drawMarker(frame, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)
        if session.pending and session.pending['frame'] == name:
            u, v = (int(c*SCALE) for c in session.pending['camera'])
            if state['view'] == 'camera':
                cv2.drawMarker(frame, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)
        x, y = state['cursor']
        padded = cv2.copyMakeBorder(base, LENS, LENS, LENS, LENS, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        half = LENS//2
        cx, cy = int(x)+LENS, int(y)+LENS
        lens = cv2.resize(padded[cy-half:cy+half, cx-half:cx+half], None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_NEAREST)
        size = lens.shape[0]
        cv2.line(lens, (size//2, 0), (size//2, size), (255, 255, 255), 1)
        cv2.line(lens, (0, size//2), (size, size//2), (255, 255, 255), 1)
        left = 8 if x*SCALE > frame.shape[1]/2 else frame.shape[1]-size-8
        frame[8:8+size, left:left+size] = lens
        cv2.rectangle(frame, (left, 8), (left+size, 8+size), (255, 255, 255), 1)
        bar = np.full((62, frame.shape[1], 3), 30, np.uint8)
        here = sum(p['frame'] == name for p in session.pairs)
        cv2.putText(bar, f"Frame {state['index']+1}/{len(names)}   View: {state['view'].upper()}   "
                         f"Points: {len(session.pairs)} ({here} here)   {state['message']}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .62, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bar, 'SPACE flip view    U undo    N next frame    P previous frame    Q finish',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .55, (170, 200, 255), 1, cv2.LINE_AA)
        return np.vstack([frame, bar])

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
    cv2.setMouseCallback(WINDOW, on_mouse)
    while True:
        cv2.imshow(WINDOW, draw())
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord(' '):
            state['view'] = 'lidar' if state['view'] == 'camera' else 'camera'
        elif key == ord('u'):
            session.undo()
            saved.write_text(json.dumps(session.pairs, indent=1))
            state['message'] = 'Undone'
        elif key in (ord('n'), ord('p')):
            step = 1 if key == ord('n') else -1
            state.update(index=(state['index']+step) % len(names), view='camera', message='Click a sharp point')
            session.pending = None
    cv2.destroyAllWindows()
    report(args.input, session, meta['focal_px'])


if __name__ == '__main__':
    main()
