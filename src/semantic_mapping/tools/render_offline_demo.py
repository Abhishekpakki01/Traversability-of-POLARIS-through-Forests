#!/usr/bin/env python3
"""Render actual saved OHM cubes and synchronized camera results to a demo video.

The video is explicitly offline playback, not a claim of live inference speed.
VTK draws the measured voxel geometry; OpenCV writes the panels and video.
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy

from semantic_mapping.semantic_fusion import CLASS_NAMES, PALETTE
from semantic_mapping.offline_demo import ELEVATION_RANGE


def text(canvas, value, xy, size=.65, color=(235, 235, 235)):
    cv2.putText(canvas, value, xy, cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)


class Scene:
    def __init__(self, width, height, resolution):
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(.13, .16, .20)
        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.SetSize(width, height)
        self.window.AddRenderer(self.renderer)
        self.window.SetMultiSamples(0)
        self.points = vtk.vtkPoints()
        self.data = vtk.vtkPolyData(); self.data.SetPoints(self.points)
        cube = vtk.vtkCubeSource()
        cube.SetXLength(resolution); cube.SetYLength(resolution); cube.SetZLength(resolution)
        self.mapper = vtk.vtkGlyph3DMapper()
        self.mapper.SetInputData(self.data); self.mapper.SetSourceConnection(cube.GetOutputPort())
        self.mapper.ScalingOff(); self.mapper.SetColorModeToDirectScalars()
        actor = vtk.vtkActor(); actor.SetMapper(self.mapper)
        actor.GetProperty().SetAmbient(.4); actor.GetProperty().SetDiffuse(.6)
        self.renderer.AddActor(actor)
        self.path_data = vtk.vtkPolyData()
        path_mapper = vtk.vtkPolyDataMapper(); path_mapper.SetInputData(self.path_data)
        path_actor = vtk.vtkActor(); path_actor.SetMapper(path_mapper)
        path_actor.GetProperty().SetColor(1., .25, .15); path_actor.GetProperty().SetLineWidth(3.)
        self.renderer.AddActor(path_actor)
        arrow = vtk.vtkArrowSource(); arrow.SetShaftRadius(.045); arrow.SetTipRadius(.13)
        arrow_mapper = vtk.vtkPolyDataMapper(); arrow_mapper.SetInputConnection(arrow.GetOutputPort())
        self.arrow_actor = vtk.vtkActor(); self.arrow_actor.SetMapper(arrow_mapper)
        self.arrow_actor.GetProperty().SetColor(1., .15, .08)
        self.renderer.AddActor(self.arrow_actor)
        self.capture = vtk.vtkWindowToImageFilter()
        self.capture.SetInput(self.window); self.capture.ReadFrontBufferOff()
        self.capture.SetInputBufferTypeToRGB()
        self.width, self.height = width, height

    def update(self, saved, color_mode):
        centres, labels = saved['centres'], saved['labels']
        self.points.SetData(numpy_to_vtk(np.ascontiguousarray(centres), deep=True))
        colors = PALETTE[labels]
        if color_mode == 'elevation':
            low, high = ELEVATION_RANGE
            values = np.rint(np.clip((saved['relative_z']-low)/(high-low), 0, 1)*255).astype(np.uint8)
            colors = cv2.applyColorMap(values[:, None], cv2.COLORMAP_TURBO)[:, 0, ::-1]
        self.data.GetPointData().SetScalars(numpy_to_vtk(np.ascontiguousarray(colors), deep=True))
        self.data.Modified()
        path = saved['trajectory']
        points = vtk.vtkPoints(); points.SetData(numpy_to_vtk(np.ascontiguousarray(path), deep=True))
        line = vtk.vtkPolyLine(); line.GetPointIds().SetNumberOfIds(len(path))
        for i in range(len(path)):
            line.GetPointIds().SetId(i, i)
        cells = vtk.vtkCellArray(); cells.InsertNextCell(line)
        self.path_data.SetPoints(points); self.path_data.SetLines(cells); self.path_data.Modified()
        self.pose = saved['world_vehicle']
        arrow_transform = vtk.vtkMatrix4x4()
        matrix = self.pose.copy(); matrix[:3, :3] *= 2.
        for row in range(4):
            for col in range(4):
                arrow_transform.SetElement(row, col, matrix[row, col])
        self.arrow_actor.SetUserMatrix(arrow_transform)

    def render(self, orbit=None):
        camera = self.renderer.GetActiveCamera()
        position = self.pose[:3, 3]
        if orbit is None:
            offset = self.pose[:3, :3] @ np.array([-11., -11., 9.])
            target = position+self.pose[:3, :3] @ np.array([2., 0., 0.])
        else:
            offset = np.array([18*math.cos(orbit), 18*math.sin(orbit), 12.])
            target = position
        camera.SetPosition(*(position+offset)); camera.SetFocalPoint(*target)
        camera.SetViewUp(0., 0., 1.); camera.SetViewAngle(55.)
        self.renderer.ResetCameraClippingRange()
        self.window.Render(); self.capture.Modified(); self.capture.Update()
        pixels = vtk_to_numpy(self.capture.GetOutput().GetPointData().GetScalars())
        return pixels.reshape(self.height, self.width, 3)[::-1, :, ::-1].copy()


def compose(scene_image, folder, entry, report, mode, final_view=False):
    canvas = np.full((960, 1600, 3), 25, np.uint8)
    text(canvas, 'POLARIS THESIS | TartanDrive: down_meadows | 32 s excerpt', (18, 29), .8)
    title = 'SEMANTIC OCCUPIED VOXELS' if mode == 'semantic' else 'ELEVATION RELATIVE TO STARTING VEHICLE Z'
    run_type = 'OFFLINE PLAYBACK' if report['full_recording_processed'] else 'PARTIAL TEST | OFFLINE PLAYBACK'
    text(canvas, title+' | '+run_type, (18, 57), .63, (130, 220, 255))
    canvas[64:832, 480:1600] = scene_image
    text(canvas, f"Source scan accounting: {report['mapped_scans']} mapped, "
         f"{report['missing_lidar_pose']} without pose", (500, 92), .56)
    if entry.get('image_stamp_ns') is not None:
        text(canvas, f"Camera - LiDAR time: {entry['image_minus_lidar_ms']:+.1f} ms", (500, 116), .53)
    for i, (name, label) in enumerate([('camera', 'Camera exposure'),
            ('segmentation', 'Predicted terrain classes'), ('projection', 'LiDAR projection: depth colors')]):
        top = 64+i*256
        text(canvas, label, (12, top+22), .60)
        path = folder/(name+'.jpg')
        if path.exists():
            pixels = cv2.imread(str(path))
            # Fit original aspect ratio into the panel without stretching.
            scale = min(456/pixels.shape[1], 224/pixels.shape[0])
            pixels = cv2.resize(pixels, (round(pixels.shape[1]*scale), round(pixels.shape[0]*scale)), interpolation=cv2.INTER_AREA)
            h, w = pixels.shape[:2]
            canvas[top+29:top+29+h, 12:12+w] = pixels
        else:
            text(canvas, 'No paired exposure / no inferred labels', (12, top+120), .48)
    suffix = ' | FINAL MAP ORBIT (time held)' if final_view else ''
    text(canvas, f"Recorded t={entry['elapsed_recording_seconds']:.2f}s | scan {entry['scans_seen']}/{report['source_scans']}"
         f" | fused {entry['fused_frames']}"+suffix, (18, 856), .65)
    text(canvas, f"Occupied {entry['occupied_voxels']} | classified {entry['classified_voxels']}"
         f" | with camera evidence {entry['voxels_with_evidence']} | voxel 0.35 m", (18, 883), .60)
    if mode == 'semantic':
        for i, (name, rgb) in enumerate(zip(CLASS_NAMES, PALETTE)):
            x = 18+i*180
            cv2.rectangle(canvas, (x, 897), (x+18, 915), tuple(map(int, rgb[::-1])), -1)
            text(canvas, name, (x+27, 912), .6)
    else:
        low, high = ELEVATION_RANGE
        text(canvas, f'Elevation scale: {low:g} m to {high:+g} m, clipped. This is not height above local ground.', (18, 912), .60)
        gradient = np.tile(np.arange(256, dtype=np.uint8), (12, 1))
        gradient = cv2.applyColorMap(gradient, cv2.COLORMAP_TURBO)
        canvas[900:912, 1225:1481] = gradient
        text(canvas, f'{low:g}', (1190, 913), .45)
        text(canvas, f'{high:+g} m', (1490, 913), .45)
    text(canvas, 'Calibration accuracy unverified | terrain predictions unvalidated | traversability not computed',
         (18, 943), .58, (150, 200, 255))
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--color-mode', choices=['semantic', 'elevation'], default='semantic')
    parser.add_argument('--fps', type=int, default=20)
    parser.add_argument('--orbit-seconds', type=float, default=5.)
    args = parser.parse_args()
    if args.fps <= 0 or args.orbit_seconds < 0:
        parser.error('Invalid frame rate or orbit duration')
    report = json.loads((args.input/'report.json').read_text())
    if not report['completed_requested_input']:
        raise ValueError('Cannot present an interrupted processing run')
    entries = [json.loads(line) for line in (args.input/'frames.jsonl').read_text().splitlines()]
    if not entries:
        raise ValueError('No saved map frames')
    output = args.input/(args.color_mode+'_demo.mp4')
    if output.exists():
        raise FileExistsError(output)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (1600, 960))
    if not writer.isOpened():
        raise RuntimeError('MP4 writer unavailable')
    scene = Scene(1120, 768, report['resolution_m'])
    written, first = 0, entries[0]['lidar_stamp_ns']
    try:
        for i, entry in enumerate(entries):
            folder = args.input/'frames'/entry['frame']
            with np.load(folder/'voxels.npz') as saved:
                scene.update(saved, args.color_mode)
            canvas = compose(scene.render(), folder, entry, report, args.color_mode)
            next_stamp = entries[i+1]['lidar_stamp_ns'] if i+1 < len(entries) else entry['lidar_stamp_ns']+100_000_000
            target = round((next_stamp-first)/1e9*args.fps)
            for _ in range(max(0, target-written)):
                writer.write(canvas); written += 1
            if i in (0, len(entries)//2, len(entries)-1):
                cv2.imwrite(str(args.input/f'{args.color_mode}_preview_{i:04d}.jpg'), canvas)
            if (i+1) % 25 == 0:
                print(f'Rendered {i+1}/{len(entries)} saved maps', flush=True)
        for i in range(round(args.orbit_seconds*args.fps)):
            angle = -2.35+2*math.pi*i/max(1, round(args.orbit_seconds*args.fps))
            canvas = compose(scene.render(angle), folder, entry, report, args.color_mode, True)
            writer.write(canvas); written += 1
    finally:
        writer.release(); scene.window.Finalize()
    cap = cv2.VideoCapture(str(output))
    decoded, count = cap.read(), int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    if not decoded[0] or count != written:
        raise RuntimeError('Written video failed decoding/count verification')
    metadata = dict(file=output.name, mode=args.color_mode, rendered_frames=written,
        fps=args.fps, duration_seconds=written/args.fps, source_map_frames=len(entries),
        offline_playback=True, playback_rate=1., final_orbit_seconds=args.orbit_seconds,
        full_recording_processed=report['full_recording_processed'], renderer='VTK cubes from saved OHM occupied centres',
        physical_calibration_validated=False)
    if args.color_mode == 'elevation':
        metadata['elevation_range_m'] = list(ELEVATION_RANGE)
    (args.input/(args.color_mode+'_video.json')).write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata, indent=2), flush=True)


if __name__ == '__main__':
    main()
