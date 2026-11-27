#!/usr/bin/env python3
"""
Generates the Foxglove Studio layouts in foxglove/layouts/*.json (single source of truth) and can
check them.

  python3 scripts/generate_layouts.py            # (re)write the JSON files
  python3 scripts/generate_layouts.py --check    # verify every message path against the ROS
                                                 # message definitions (needs a sourced ROS env)
  python3 scripts/generate_layouts.py --check --live   # also compare topics with the running graph

Import in Foxglove: Layouts (left bar) -> Import from file...
"""
import argparse
import hashlib
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'foxglove', 'layouts')

# Every topic the layouts touch, with its message type (used by --check).
TOPIC_TYPES = {
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/odometry/filtered': 'nav_msgs/msg/Odometry',
    '/agrobot_base_controller/odom': 'nav_msgs/msg/Odometry',
    '/imu/data_game': 'sensor_msgs/msg/Imu',
    '/imu/data_mag': 'sensor_msgs/msg/Imu',
    '/gps/fix': 'sensor_msgs/msg/NavSatFix',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/scan_raw': 'sensor_msgs/msg/LaserScan',
    '/camera/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
    '/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/arm_controller/controller_state': 'control_msgs/msg/JointTrajectoryControllerState',
    '/telemetry/wheel_velocity': 'std_msgs/msg/Float64MultiArray',
    '/telemetry/arm_position': 'std_msgs/msg/Float64MultiArray',
    '/telemetry/heading_deg': 'std_msgs/msg/Float64',
    '/telemetry/cpu_percent': 'std_msgs/msg/Float64',
    '/telemetry/mem_percent': 'std_msgs/msg/Float64',
    '/telemetry/cpu_temp_c': 'std_msgs/msg/Float64',
    '/diagnostics': 'diagnostic_msgs/msg/DiagnosticArray',
    '/rosout': 'rcl_interfaces/msg/Log',
    '/navigate_to_pose/_action/status': 'action_msgs/msg/GoalStatusArray',
    '/map': 'nav_msgs/msg/OccupancyGrid',
    '/local_costmap/costmap': 'nav_msgs/msg/OccupancyGrid',
    '/global_costmap/costmap': 'nav_msgs/msg/OccupancyGrid',
    '/plan': 'nav_msgs/msg/Path',
    '/local_plan': 'nav_msgs/msg/Path',
    '/local_costmap/published_footprint': 'geometry_msgs/msg/PolygonStamped',
    '/robot_description': 'std_msgs/msg/String',
    '/lidar_manager/state': 'std_msgs/msg/String',
    '/lidar_manager/status': 'std_msgs/msg/String',
    '/lidar_manager/log': 'rcl_interfaces/msg/Log',
}
PALETTE = ['#4e98e2', '#f5774d', '#f7df71', '#5cd6a9', '#cb7de8', '#e86a8c']


def pid(kind, name):
    return f'{kind}!{hashlib.md5(name.encode()).hexdigest()[:7]}'


def split(direction, first, second, pct):
    return {'direction': direction, 'first': first, 'second': second, 'splitPercentage': pct}


def plot(title, series, y_min=None, y_max=None, window=30):
    """series: [(label, '/topic.field.path'), ...]"""
    cfg = {
        'paths': [{'value': path, 'enabled': True, 'timestampMethod': 'receiveTime', 'label': label,
                   'color': PALETTE[i % len(PALETTE)], 'lineSize': 2, 'showLine': True}
                  for i, (label, path) in enumerate(series)],
        'showXAxisLabels': True, 'showYAxisLabels': True, 'showLegend': True,
        'legendDisplay': 'floating', 'showPlotValuesInLegend': False, 'isSynced': True,
        'xAxisVal': 'timestamp', 'sidebarDimension': 240, 'followingViewWidth': window,
        'foxglovePanelTitle': title,
    }
    if y_min is not None:
        cfg['minYValue'] = y_min
    if y_max is not None:
        cfg['maxYValue'] = y_max
    return cfg


def gauge(title, path, lo, hi, colors=('#2ecc71', '#f1c40f', '#e74c3c')):
    return {'path': path, 'minValue': lo, 'maxValue': hi, 'colorMode': 'gradient',
            'gradient': list(colors[:1] + colors[-1:]), 'reverse': False, 'foxglovePanelTitle': title}


def panel_3d():
    def uuid(n):
        h = hashlib.md5(n.encode()).hexdigest()
        return f'{h[:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}'
    return {
        'layers': {
            uuid('grid'): {'layerId': 'foxglove.Grid', 'instanceId': uuid('grid'), 'visible': True,
                           'label': 'Grid', 'frameLocked': True, 'size': 30, 'divisions': 30,
                           'lineWidth': 1, 'color': '#3a4a5c', 'position': [0, 0, 0],
                           'rotation': [0, 0, 0], 'order': 1},
            uuid('urdf'): {'layerId': 'foxglove.Urdf', 'instanceId': uuid('urdf'), 'visible': True,
                           'label': 'AgroBot', 'sourceType': 'topic', 'topic': '/robot_description',
                           'url': '', 'filePath': '', 'parameter': '', 'framePrefix': '',
                           'displayMode': 'auto', 'fallbackColor': '#ffffff', 'order': 2},
        },
        'cameraState': {'perspective': True, 'distance': 4.5, 'phi': 60, 'thetaOffset': 35,
                        'targetOffset': [0, 0, 0], 'target': [0, 0, 0],
                        'targetOrientation': [0, 0, 0, 1], 'fovy': 45, 'near': 0.01, 'far': 2000},
        'followMode': 'follow-position', 'followTf': 'base_footprint',
        'scene': {'backgroundColor': '#12161c', 'enableStats': False, 'labelScaleFactor': 1,
                  'transforms': {'showLabel': False, 'editable': False, 'axisScale': 0.2,
                                 'lineWidth': 0.02}},
        'topics': {
            '/scan': {'visible': True, 'colorMode': 'flat', 'flatColor': '#22d3eeff',
                      'pointSize': 3, 'pointShape': 'circle', 'decayTime': 0.1},
            '/map': {'visible': True, 'colorMode': 'map', 'alpha': 0.85, 'frameLocked': False},
            '/local_costmap/costmap': {'visible': True, 'colorMode': 'costmap', 'alpha': 0.45,
                                       'frameLocked': True},
            '/global_costmap/costmap': {'visible': False, 'colorMode': 'costmap', 'alpha': 0.35},
            '/plan': {'visible': True, 'type': 'line', 'lineWidth': 0.06,
                      'gradient': ['#00e676ff', '#00e676ff']},
            '/local_plan': {'visible': True, 'type': 'line', 'lineWidth': 0.05,
                            'gradient': ['#ff4081ff', '#ff4081ff']},
            '/local_costmap/published_footprint': {'visible': True, 'color': '#00e676ff'},
            '/odometry/filtered': {'visible': True},
        },
        'publish': {'type': 'pose', 'poseTopic': '/goal_pose', 'pointTopic': '/clicked_point',
                    'poseEstimateTopic': '/initialpose', 'poseEstimateXDeviation': 0.5,
                    'poseEstimateYDeviation': 0.5, 'poseEstimateThetaDeviation': 0.26},
        'foxglovePanelTitle': 'Robot / Map / Plans',
    }


ARM_PLANNER = 'AgroBotArmPlanner.Arm Planner'   # local extensions are named <displayName>.<panel> (foxglove/extension)


def mission_control():
    ids = {k: pid(*k.split(':')) for k in (
        '3D:main', 'Image:camera', 'Plot:speed', 'Plot:yaw', 'Teleop:teleop', 'DiagnosticSummary:diag')}
    ids['arm'] = pid(ARM_PLANNER, 'armplanner')
    cfg = {
        ids['3D:main']: panel_3d(),
        ids['Image:camera']: {'imageMode': {'imageTopic': '/camera/image_raw/compressed',
                                            'calibrationTopic': '/camera/camera_info'},
                              'foxglovePanelTitle': 'Arm camera'},
        ids['Plot:speed']: plot('Linear speed (m/s)', [
            ('cmd_vel', '/cmd_vel.linear.x'),
            ('EKF', '/odometry/filtered.twist.twist.linear.x'),
            ('wheel odom', '/agrobot_base_controller/odom.twist.twist.linear.x')],
            y_min=-0.8, y_max=0.8, window=20),
        ids['Plot:yaw']: plot('Yaw rate (rad/s)', [
            ('cmd_vel', '/cmd_vel.angular.z'),
            ('IMU gyro', '/imu/data_game.angular_velocity.z'),
            ('EKF', '/odometry/filtered.twist.twist.angular.z')],
            y_min=-2.5, y_max=2.5, window=20),
        ids['Teleop:teleop']: {
            'topic': '/cmd_vel', 'publishRate': 10,
            'upButton': {'field': 'linear-x', 'value': 0.3},
            'downButton': {'field': 'linear-x', 'value': -0.3},
            'leftButton': {'field': 'angular-z', 'value': 0.8},
            'rightButton': {'field': 'angular-z', 'value': -0.8},
            'foxglovePanelTitle': 'Drive'},
        ids['DiagnosticSummary:diag']: {'minLevel': 0, 'pinnedIds': [], 'hardwareIdFilter': '',
                                        'topicToRender': '/diagnostics', 'sortByLevel': True,
                                        'foxglovePanelTitle': 'Health'},
        ids['arm']: {'foxglovePanelTitle': 'Arm planner'},
    }
    # left: 3D on top; below it Drive | speed plot | yaw plot | Health.  right: arm camera over the planner
    bottom = split('row', ids['Teleop:teleop'],
                   split('row', ids['Plot:speed'],
                         split('row', ids['Plot:yaw'], ids['DiagnosticSummary:diag'], 55), 50), 16)
    left = split('column', ids['3D:main'], bottom, 68)
    right = split('column', ids['Image:camera'], ids['arm'], 30)
    layout = split('row', left, right, 64)
    return {'configById': cfg, 'globalVariables': {}, 'userNodes': {},
            'playbackConfig': {'speed': 1}, 'layout': layout}


def panel_3d_robot():
    """Robot model only: grid + URDF; the joints follow /joint_states through robot_state_publisher."""
    cfg = panel_3d()
    cfg['topics'] = {}
    cfg.pop('publish', None)
    cfg['cameraState'].update({'distance': 1.4, 'phi': 65, 'thetaOffset': 140})
    cfg['foxglovePanelTitle'] = 'Robot joints (3D)'
    return cfg


def call_service(title, service, color):
    return {'serviceName': service, 'requestPayload': '{}', 'layout': 'vertical', 'buttonText': title,
            'buttonTooltip': f'Call {service}', 'buttonColor': color, 'editingMode': False,
            'foxglovePanelTitle': title}


def engineering():
    ids = {k: pid(*k.split(':')) for k in (
        'Plot:wheels', 'Plot:armpos', 'Plot:armerr', 'Plot:odom', 'TransformTree:tf', '3D:robot',
        'DiagnosticStatusPanel:lidar', 'Indicator:lidar', 'CallService:start', 'CallService:restart',
        'CallService:stop', 'RosOut:lidar')}
    cfg = {
        ids['Plot:wheels']: plot('Wheel velocity (rad/s)', [
            (n, f'/telemetry/wheel_velocity.data[{i}]') for i, n in enumerate(['FL', 'FR', 'RL', 'RR'])],
            y_min=-12, y_max=12),
        ids['Plot:armpos']: plot('Arm joints: feedback vs reference (rad)', [
            ('yaw', '/arm_controller/controller_state.feedback.positions[0]'),
            ('shoulder', '/arm_controller/controller_state.feedback.positions[1]'),
            ('wrist', '/arm_controller/controller_state.feedback.positions[2]'),
            ('yaw ref', '/arm_controller/controller_state.reference.positions[0]'),
            ('shoulder ref', '/arm_controller/controller_state.reference.positions[1]'),
            ('wrist ref', '/arm_controller/controller_state.reference.positions[2]')],
            y_min=-1.7, y_max=1.7),
        ids['Plot:armerr']: plot('Arm tracking error (rad)', [
            ('yaw', '/arm_controller/controller_state.error.positions[0]'),
            ('shoulder', '/arm_controller/controller_state.error.positions[1]'),
            ('wrist', '/arm_controller/controller_state.error.positions[2]')],
            y_min=-0.2, y_max=0.2),
        ids['Plot:odom']: plot('EKF position (m)', [
            ('x', '/odometry/filtered.pose.pose.position.x'),
            ('y', '/odometry/filtered.pose.pose.position.y')], window=120),
        ids['TransformTree:tf']: {'foxglovePanelTitle': 'TF tree'},
        ids['3D:robot']: panel_3d_robot(),
        ids['DiagnosticStatusPanel:lidar']: {
            'topicToRender': '/diagnostics', 'selectedHardwareId': 'lidar', 'selectedName': 'LiDAR driver',
            'splitFraction': 0.4, 'numericPrecision': 0, 'sortByLevel': True,
            'foxglovePanelTitle': 'LiDAR driver (sllidar_node)'},
        ids['Indicator:lidar']: {
            'path': '/lidar_manager/state.data', 'style': 'background', 'fallbackColor': '#6b7280',
            'fallbackLabel': 'LiDAR: no manager', 'rules': [
                {'operator': '=', 'rawValue': 'running', 'color': '#16a34a', 'label': 'LiDAR running'},
                {'operator': '=', 'rawValue': 'starting', 'color': '#d97706', 'label': 'LiDAR starting...'},
                {'operator': '=', 'rawValue': 'stopped', 'color': '#6b7280', 'label': 'LiDAR stopped'},
                {'operator': '=', 'rawValue': 'crashed', 'color': '#dc2626', 'label': 'LiDAR CRASHED'}],
            'foxglovePanelTitle': 'LiDAR state'},
        ids['CallService:start']: call_service('Start', '/lidar_manager/start', '#16a34a'),
        ids['CallService:restart']: call_service('Relaunch', '/lidar_manager/restart', '#2563eb'),
        ids['CallService:stop']: call_service('Stop', '/lidar_manager/stop', '#b91c1c'),
        ids['RosOut:lidar']: {'topicToRender': '/lidar_manager/log', 'searchTerms': [], 'minLogLevel': 1,
                              'nameFilter': {}, 'foxglovePanelTitle': 'sllidar_node log'},
    }
    row1 = split('row', ids['Plot:wheels'], split('row', ids['Plot:armpos'], ids['Plot:armerr'], 50), 33)
    # TF tree spans two cells (where the two IMU plots were); EKF position stays in the third
    row2 = split('row', ids['TransformTree:tf'], ids['Plot:odom'], 66)
    buttons = split('row', ids['Indicator:lidar'],
                    split('row', ids['CallService:start'],
                          split('row', ids['CallService:restart'], ids['CallService:stop'], 50), 33), 34)
    lidar = split('column', buttons, ids['RosOut:lidar'], 30)
    row3 = split('row', ids['3D:robot'], split('row', ids['DiagnosticStatusPanel:lidar'], lidar, 32), 28)
    layout = split('column', row1, split('column', row2, row3, 50), 33)
    return {'configById': cfg, 'globalVariables': {}, 'userNodes': {},
            'playbackConfig': {'speed': 1}, 'layout': layout}


LAYOUTS = {'agrobot_mission_control.json': mission_control, 'agrobot_engineering.json': engineering}


# ----------------------------------------------------------------------------- checking
def collect_paths(layout):
    """Yield (panel, kind, value) for every topic / message path in a layout."""
    for pid_, cfg in layout['configById'].items():
        kind = pid_.split('!')[0]
        for p in cfg.get('paths', []):
            yield pid_, 'path', p['value']
        if 'path' in cfg:
            yield pid_, 'path', cfg['path']
        for key in ('topic', 'topicToRender', 'followTopic'):
            if key in cfg:
                yield pid_, 'topic', cfg[key]
        im = cfg.get('imageMode', {})
        for key in ('imageTopic', 'calibrationTopic'):
            if key in im:
                yield pid_, 'topic', im[key]
        for t in cfg.get('topics', {}):
            yield pid_, 'topic', t
        if kind == 'RosOut':
            yield pid_, 'topic', cfg.get('topicToRender', '/rosout')
        if kind == '3D':
            for key in ('poseTopic', 'pointTopic', 'poseEstimateTopic'):
                pass


def check(live):
    from rosidl_runtime_py.utilities import get_message
    problems = []
    seen_topics = set()

    def field_type(msg_cls, token):
        name = re.match(r'[A-Za-z_]\w*', token).group(0)
        fields = msg_cls.get_fields_and_field_types()
        if name not in fields:
            return None, f'no field "{name}" in {msg_cls.__module__.split(".")[0]}/{msg_cls.__name__}'
        t = fields[name]
        m = re.match(r'sequence<(.+?)(?:, \d+)?>$', t) or re.match(r'(.+?)\[\d*\]$', t)
        if m:
            t = m.group(1)
        if '/' in t:
            return get_message(t), None
        return None, None  # primitive leaf

    for fname, builder in LAYOUTS.items():
        layout = builder()
        for panel, kind, value in collect_paths(layout):
            topic = re.match(r'(/[^.\s]*)', value).group(1) if value.startswith('/') else value
            seen_topics.add(topic)
            if topic not in TOPIC_TYPES:
                if kind == 'path' or topic.startswith('/'):
                    problems.append(f'{fname}: {panel}: topic {topic} not in TOPIC_TYPES')
                continue
            if kind != 'path' or value == topic:
                continue
            cls = get_message(TOPIC_TYPES[topic])
            for tok in value[len(topic) + 1:].split('.'):
                if cls is None:
                    problems.append(f'{fname}: {panel}: {value}: field "{tok}" after a primitive')
                    break
                cls, err = field_type(cls, tok)
                if err:
                    problems.append(f'{fname}: {panel}: {value}: {err}')
                    break
    if live:
        import subprocess
        out = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=30).stdout.split()
        missing = sorted(t for t in seen_topics if t in TOPIC_TYPES and t not in out)
        print('Topics in layouts but not currently published:', ', '.join(missing) or 'none')
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--live', action='store_true')
    args = ap.parse_args()
    if args.check:
        problems = check(args.live)
        for p in problems:
            print('PROBLEM:', p)
        print('layouts OK' if not problems else f'{len(problems)} problem(s)')
        sys.exit(1 if problems else 0)
    os.makedirs(OUT, exist_ok=True)
    for fname, builder in LAYOUTS.items():
        path = os.path.join(OUT, fname)
        with open(path, 'w') as f:
            json.dump(builder(), f, indent=2)
            f.write('\n')
        print('wrote', os.path.normpath(path))


if __name__ == '__main__':
    main()
