#!/usr/bin/env python3
"""Make pictures and videos from the real RViz view, with the camera images beside the map.

The v2 replay is stepped one scan at a time. For each scan this publishes the map, waits for RViz
to draw it, saves the RViz window (capture_rviz_window.ps1) and places the camera image and
predicted classes of that same scan beside it. Map and camera therefore always match, no scan is
skipped, and videos play at the recording's real speed of 10 scans per second.

Colour keys are computed from the layer shown. Saved RViz layouts and results are not changed.
Mouse and keyboard input to the RViz window is blocked while it records.
"""
import argparse
import copy
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc

FULL, FUSED = 'Full accumulated map', 'Traversability fused'
LIDAR, WATER = 'LiDAR-only classes, full map', 'Water ('
TOPICS = {FUSED: '/terrain/traversability_fused', LIDAR: '/lidar/features', WATER: '/water/cells', FULL: '/semantic_voxel_map'}
ALWAYS = ['/tf', '/vehicle_pose', '/offline_demo/vehicle_path']
PANELS = ['/offline_demo/camera', '/offline_demo/segmentation', '/offline_demo/status_image']
LAYERS = {  # displays, title, legend source
    'traversability': ([FUSED], 'Traversability (camera + LiDAR)', 'fused'),
    'lidar': ([LIDAR], 'LiDAR-only classes', '/lidar/legend'),
    'water': ([LIDAR, WATER], 'Water on the LiDAR-class map', '/water/legend'),
    'camera': ([FULL], 'Camera classes on the voxel map', None),
}
OVERVIEW = dict(kind='Orbit', frame='sensor_init', focal=(78., -90., -20.), distance=152., pitch=1.05, yaw=3.14, label='overview')
CHASE = dict(kind='ThirdPersonFollower', frame='vehicle', focal=(0., 0., 0.), distance=24., pitch=.85, yaw=3.14, label='chase')
LAST = 318
RUNS = [  # layer, view, scans (None = all), video name, {scan: picture name}
    ('traversability', CHASE, None, 'video_1_follow_vehicle_traversability',
     {80: 'picture_1_follow_traversability_scan080', 160: 'picture_2_follow_traversability_scan160',
      240: 'picture_3_follow_traversability_scan240'}),
    ('lidar', CHASE, None, 'video_2_follow_vehicle_lidar_classes', {160: 'picture_4_follow_lidar_classes_scan160'}),
    ('traversability', OVERVIEW, None, 'video_3_whole_map_traversability', {LAST: 'picture_5_whole_map_traversability'}),
    ('lidar', OVERVIEW, [LAST], None, {LAST: 'picture_6_whole_map_lidar_classes'}),
    ('camera', OVERVIEW, [LAST], None, {LAST: 'picture_7_whole_map_camera_classes'}),
    ('water', CHASE, [232], None, {232: 'picture_8_water_scan232'}),
]
STATE_COLOURS = {'traversable': (10, 165, 10), 'caution': (20, 175, 250), 'non-traversable': (60, 55, 210)}
STATE_VALUES = {'traversable': 2, 'caution': 3, 'non-traversable': 4}
WIDTH, HEIGHT, MAP_WIDTH = 1920, 1080, 1230
PANEL_WIDTH = WIDTH - MAP_WIDTH - 20


def rviz_layout(base, layer, view):
    config = copy.deepcopy(base)
    names = LAYERS[layer][0]
    for display in config['Visualization Manager']['Displays']:
        name, kind = display['Name'], display['Class']
        if 'Enabled' in display:
            display['Enabled'] = name.startswith(('Vehicle pose', 'Vehicle trail')) or any(name.startswith(n) for n in names)
        if kind.endswith('/Path'):
            display.update({'Color': '255; 255; 255', 'Line Style': 'Billboards', 'Line Width': .3 if view is OVERVIEW else .12})
        if kind.endswith('/Pose'):
            display.update({'Color': '255; 0; 255', 'Shaft Length': 2., 'Shaft Radius': .14, 'Head Length': 1., 'Head Radius': .4})
    displays = config['Visualization Manager']['Displays']
    config['Visualization Manager']['Displays'] = [d for d in displays if not d['Class'].endswith('/Image')]
    current = config['Visualization Manager']['Views']['Current']
    plugin = f"rviz_default_plugins/{view['kind']}"
    current.update({'Class': plugin, 'Value': f"{view['kind']} (rviz_default_plugins)", 'Target Frame': view['frame'],
                    'Distance': view['distance'], 'Pitch': view['pitch'], 'Yaw': view['yaw'],
                    'Focal Point': dict(zip('XYZ', view['focal']))})
    window = config['Window Geometry']
    window.update({'Hide Left Dock': True, 'Hide Right Dock': True, 'Width': 1230, 'Height': 1000, 'X': 0, 'Y': 0})
    window.pop('QMainWindow State', None)  # the saved dock arrangement would bring the panels back
    for name in [k for k, v in window.items() if isinstance(v, dict)]:
        window.pop(name)
    config['Panels'] = []
    return config


class Capture:
    def __init__(self):
        run = dict(cwd='/mnt/c', capture_output=True, text=True)
        self.temp = subprocess.run(['cmd.exe', '/c', 'echo %TEMP%'], **run).stdout.strip()
        self.local = Path(subprocess.check_output(['wslpath', self.temp], text=True).strip())
        shutil.copy(Path(__file__).with_name('capture_rviz_window.ps1'), self.local/'capture_rviz_window.ps1')
        self.process = subprocess.Popen(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                                         self.temp+'\\capture_rviz_window.ps1'], cwd='/mnt/c', stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)

    def frame(self):
        self.process.stdin.write(self.temp+'\\rviz_media_frame.jpg\n')
        self.process.stdin.flush()
        if self.process.stdout.readline().strip() != 'ok':
            raise RuntimeError('RViz window capture failed')
        return cv2.imread(str(self.local/'rviz_media_frame.jpg'))

    def close(self):
        self.process.stdin.close()
        self.process.wait(10)
        (self.local/'rviz_media_frame.jpg').unlink(missing_ok=True)


def scans(bag, topics):
    """Yield (scan number, {topic: serialized message}); every scan shares one bag timestamp."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    group, stamp, number = {}, None, 0
    while reader.has_next():
        topic, data, time_ns = reader.read_next()
        if stamp is not None and time_ns != stamp:
            number += 1
            yield number, stamp, group
            group = {}
        stamp = time_ns
        group[topic] = data
    yield number+1, stamp, group


def image(data):
    message = deserialize_message(data, get_message('sensor_msgs/msg/Image'))
    return np.frombuffer(message.data, np.uint8).reshape(message.height, message.width, -1)


def put(canvas, value, xy, size=.62, colour=(235, 235, 235), weight=1):
    cv2.putText(canvas, value, xy, cv2.FONT_HERSHEY_SIMPLEX, size, colour, weight, cv2.LINE_AA)


def fit(frame, width):
    return cv2.resize(frame, (width, round(frame.shape[0]*width/frame.shape[1])), interpolation=cv2.INTER_AREA)


def fused_key(data):
    cloud = deserialize_message(data, PointCloud2)
    state = pc.read_points(cloud, field_names=('state',))['state']
    key = np.full((96, PANEL_WIDTH, 3), 28, np.uint8)
    x = 12
    for name, value in STATE_VALUES.items():
        share = 100*np.count_nonzero(state == value)/max(len(state), 1)
        cv2.rectangle(key, (x, 16), (x+22, 38), STATE_COLOURS[name], -1)
        put(key, f'{name} {share:.0f}%', (x+30, 34), .55)
        x += 190 if name != 'caution' else 150
    put(key, 'Limits are PLACEHOLDERS, not POLARIS specifications', (12, 74), .55, (60, 180, 250))
    return key


def view_bounds(frame):
    """Rows and columns of the 3D view: the window frame around it is light grey or black."""
    brightness = frame.mean(2)
    inside = (brightness > 25) & (brightness < 70)  # the empty view's background
    rows, columns = np.flatnonzero(inside.mean(1) > .6), np.flatnonzero(inside.mean(0) > .6)
    return rows[0]+2, rows[-1]-1, columns[0]+2, columns[-1]-1


def compose(view, group, layer, label, number, total):
    canvas = np.full((HEIGHT, WIDTH, 3), 28, np.uint8)
    keep = round(view.shape[0]*MAP_WIDTH/HEIGHT)  # the window is wider than this panel: keep its centre
    start = max((view.shape[1]-keep)//2, 0)
    canvas[:, :MAP_WIDTH] = cv2.resize(view[:, start:start+keep], (MAP_WIDTH, HEIGHT), interpolation=cv2.INTER_CUBIC)
    x, y = MAP_WIDTH+10, 34
    _, title, key_source = LAYERS[layer]
    put(canvas, title, (x, y), .78, (255, 255, 255), 2)
    y += 28
    put(canvas, f'RViz view: {label}  |  scan {number}/{total}  |  t = {(number-1)/10:.1f} s', (x, y), .52, (190, 190, 190))
    y += 14
    for caption, topic in (('Camera reference (same scan)', PANELS[0]), ('Predicted terrain classes (GOOSE pretrained model)', PANELS[1])):
        y += 22
        put(canvas, caption, (x, y), .56)
        y += 8
        panel = fit(image(group[topic]), PANEL_WIDTH)
        canvas[y:y+panel.shape[0], x:x+PANEL_WIDTH] = panel
        y += panel.shape[0]
    strip = fit(image(group[PANELS[2]])[184:214], PANEL_WIDTH)
    y += 4
    canvas[y:y+strip.shape[0], x:x+PANEL_WIDTH] = strip
    y += strip.shape[0]+6
    if key_source:
        key = fused_key(group[TOPICS[FUSED]]) if key_source == 'fused' else fit(image(group[key_source]), PANEL_WIDTH)
        key = key[:HEIGHT-y-30]
        canvas[y:y+key.shape[0], x:x+PANEL_WIDTH] = key
    put(canvas, 'Offline replay of a TartanDrive 2.0 recording, not live', (x, HEIGHT-12), .5, (170, 170, 170))
    return canvas


def ffmpeg_path():
    found = shutil.which('ffmpeg') or next(iter(Path.home().glob(
        'thesis_env/lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg-*')), None)
    if not found:
        raise SystemExit('Need ffmpeg: run  ~/thesis_env/bin/pip install imageio-ffmpeg')
    return str(found)


def encoder(path, fps):
    return subprocess.Popen([ffmpeg_path(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                             '-s', f'{WIDTH}x{HEIGHT}', '-r', str(fps), '-i', '-', '-c:v', 'libx264', '-preset', 'medium',
                             '-crf', '25', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)


def start_rviz(path, log_path):
    environment = {k: v for k, v in os.environ.items() if not k.startswith('QT_QPA')}  # cv2 sets these for its own Qt
    return subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', str(path), '--ros-args', '-p', 'use_sim_time:=true'],
                            stdout=open(log_path, 'w'), stderr=subprocess.STDOUT, start_new_session=True, env=environment)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


def run(args, base, node, layer, view, wanted, video, pictures):
    names = LAYERS[layer][0]
    map_topics = [TOPICS[n] for n in names]
    key_source = LAYERS[layer][2]
    extra = [key_source] if key_source and key_source.startswith('/') else []
    label = 'whole map' if view is OVERVIEW else 'following the vehicle'
    stem = video or next(iter(pictures.values()))
    layout = args.output/'rviz_layouts'/f'{stem}.rviz'
    layout.write_text(yaml.safe_dump(rviz_layout(base, layer, view), sort_keys=False))
    rviz = start_rviz(layout, layout.with_suffix('.log'))
    publishers = {}
    types = {'/tf': 'tf2_msgs/msg/TFMessage', '/vehicle_pose': 'geometry_msgs/msg/PoseStamped',
             '/offline_demo/vehicle_path': 'nav_msgs/msg/Path'}
    for topic in ALWAYS+map_topics:
        publishers[topic] = node.create_publisher(get_message(types.get(topic, 'sensor_msgs/msg/PointCloud2')), topic, 10)
    clock = node.create_publisher(Clock, '/clock', 10)
    limit = time.time()+40
    while any(p.get_subscription_count() == 0 for p in publishers.values()) and time.time() < limit:
        time.sleep(.2)
    time.sleep(args.rviz_start_s)
    capture, writer = Capture(), encoder(args.output/f'{video}.mp4', args.fps) if video else None
    total, started = LAST, time.time()
    capture.frame()
    time.sleep(1)
    blank = capture.frame()
    bounds = view_bounds(blank)
    print('window', blank.shape[:2], 'view', bounds, flush=True)
    try:
        for number, stamp, group in scans(args.bag, list(dict.fromkeys(ALWAYS+map_topics+PANELS+extra))):
            if wanted and number not in wanted:
                continue
            message = Clock()
            message.clock.sec, message.clock.nanosec = divmod(stamp, 1_000_000_000)
            clock.publish(message)
            for topic, publisher in publishers.items():
                publisher.publish(group[topic])
            time.sleep(args.settle_s if not wanted or len(wanted) > 1 else 4.)
            frame = capture.frame()
            if args.save_raw:
                cv2.imwrite(str(args.output/f'raw_{number}.jpg'), frame)
            view_image = frame[bounds[0]:bounds[1], bounds[2]:bounds[3]]
            canvas = compose(view_image, group, layer, label, number, total)
            if writer:
                writer.stdin.write(canvas.tobytes())
            if number in pictures:
                cv2.imwrite(str(args.output/f'{pictures[number]}.jpg'), canvas, [cv2.IMWRITE_JPEG_QUALITY, 93])
            if rviz.poll() is not None:
                raise RuntimeError(f'RViz exited; see {layout.with_suffix(".log")}')
    finally:
        capture.close()
        if writer:
            writer.stdin.close()
            writer.wait()
        stop(rviz)
        for publisher in publishers.values():
            node.destroy_publisher(publisher)
        node.destroy_publisher(clock)
    print(f'{stem}: done in {time.time()-started:.0f} s', flush=True)


def main():
    repo = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bag', type=Path, default=repo/'artifacts/water_demo_v2_2026_11_03/replay')
    parser.add_argument('--base-layout', type=Path, default=repo/'src/semantic_mapping/config/demo_v2.rviz')
    parser.add_argument('--only', nargs='+', type=int, help='numbers of the runs to make (1-6), default all')
    parser.add_argument('--scans', nargs='+', type=int, help='test: step only these scans of each run')
    parser.add_argument('--fps', type=float, default=10., help='10 is the real speed of the recording')
    parser.add_argument('--settle-s', type=float, default=.45, help='time RViz gets to draw each scan')
    parser.add_argument('--save-raw', action='store_true', help='test: also save the uncropped window')
    parser.add_argument('--rviz-start-s', type=float, default=3.)
    args = parser.parse_args()
    (args.output/'rviz_layouts').mkdir(parents=True, exist_ok=True)
    base = yaml.safe_load(args.base_layout.read_text())
    rclpy.init()
    node = rclpy.create_node('rviz_media_capture')
    for index, (layer, view, wanted, video, pictures) in enumerate(RUNS, 1):
        if not args.only or index in args.only:
            if args.scans:
                wanted, video, pictures = args.scans, None, {s: f'test_{index}_scan{s}' for s in args.scans}
            run(args, base, node, layer, view, wanted, video, pictures)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
