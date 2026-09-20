"""Independent maximum-height audit of URDF collision surfaces (metres).

Uses the same calibrated URDF as MoveIt, but reports link surface heights
directly in the measured table frame. Unknown geometry fails closed.
This is a geometric model check, not a safety-rated monitor.
"""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation


def numbers(text, default):
    return np.asarray([float(x) for x in text.split()] if text else default)


def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = numbers(element.get('xyz'), [0, 0, 0])
        result[:3, :3] = Rotation.from_euler(
            'xyz', numbers(element.get('rpy'), [0, 0, 0])
        ).as_matrix()
    return result


def mesh_vertices(filename):
    if filename.startswith('package://'):
        package, relative = filename[len('package://'):].split('/', 1)
        filename = str(Path(get_package_share_directory(package)) / relative)
    if filename.startswith('file://'):
        filename = filename[7:]
    data = Path(filename).read_bytes()
    if len(data) < 84:
        raise ValueError(f'Invalid binary STL: {filename}')
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f'Only binary STL collision meshes supported: {filename}')
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, offset=84, count=count)
    return np.unique(triangles['vertices'].reshape(-1, 3).astype(float), axis=0)


class CeilingGeometry:
    def __init__(self, xml, base_from_world):
        root = ET.fromstring(xml)
        self.world_from_base = np.linalg.inv(np.asarray(base_from_world, dtype=float))
        self.links = [link.get('name') for link in root.findall('link')]
        self.joints = []
        children = set()
        for joint in root.findall('joint'):
            child = joint.find('child').get('link')
            children.add(child)
            axis = joint.find('axis')
            self.joints.append((
                joint.get('name'), joint.get('type'), joint.find('parent').get('link'),
                child, origin(joint.find('origin')),
                numbers(axis.get('xyz') if axis is not None else None, [1, 0, 0]),
            ))
        roots = set(self.links) - children
        if len(roots) != 1 or 'base' not in self.links:
            raise ValueError('Expected a connected URDF with controller base frame')
        self.root = roots.pop()
        self.surfaces = []
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                kind = list(geometry)[0]
                if kind.tag == 'mesh':
                    shape = mesh_vertices(kind.get('filename')) * numbers(kind.get('scale'), [1, 1, 1])
                elif kind.tag == 'sphere':
                    shape = float(kind.get('radius'))
                elif kind.tag == 'cylinder':
                    shape = (float(kind.get('radius')), float(kind.get('length')))
                elif kind.tag == 'box':
                    size = numbers(kind.get('size'), []) / 2
                    shape = np.asarray([[x, y, z] for x in [-size[0], size[0]]
                                        for y in [-size[1], size[1]] for z in [-size[2], size[2]]])
                else:
                    raise ValueError(f'Unsupported collision geometry {kind.tag}')
                self.surfaces.append((link.get('name'), origin(collision.find('origin')), kind.tag, shape))
        if len(self.surfaces) < 7:
            raise ValueError('Incomplete UR3 collision geometry')

    def transforms(self, joints):
        transforms = {self.root: np.eye(4)}
        pending = list(self.joints)
        while pending:
            next_pending = []
            for name, kind, parent, child, fixed, axis in pending:
                if parent not in transforms:
                    next_pending.append((name, kind, parent, child, fixed, axis))
                    continue
                motion = np.eye(4)
                if kind in ('revolute', 'continuous'):
                    value = float(joints[name])
                    if not np.isfinite(value):
                        raise ValueError(f'Nonfinite joint {name}')
                    motion[:3, :3] = Rotation.from_rotvec(axis / np.linalg.norm(axis) * value).as_matrix()
                elif kind != 'fixed':
                    raise ValueError(f'Unsupported joint type {kind}')
                transforms[child] = transforms[parent] @ fixed @ motion
            if len(next_pending) == len(pending):
                raise ValueError('Disconnected URDF')
            pending = next_pending
        base_from_root = np.linalg.inv(transforms['base'])
        return {name: base_from_root @ value for name, value in transforms.items()}

    def heights(self, joints, sphere_xyz, sphere_radius):
        transforms = self.transforms(joints)
        result = {}
        for link, offset, kind, shape in self.surfaces:
            pose = self.world_from_base @ transforms[link] @ offset
            if kind in ('mesh', 'box'):
                top = float(np.max(shape @ pose[2, :3]) + pose[2, 3])
            elif kind == 'sphere':
                top = float(pose[2, 3] + shape)
            else:
                radius, length = shape
                top = float(pose[2, 3] + radius * np.linalg.norm(pose[2, :2])
                            + abs(pose[2, 2]) * length / 2)
            result[link] = max(top, result.get(link, -np.inf))
        sphere = self.world_from_base @ transforms['tool0'] @ np.r_[sphere_xyz, 1.0]
        result['magnet_sphere_guard'] = float(sphere[2] + sphere_radius)
        return result


def sample_trajectory(trajectory, interval_s=0.01):
    """Evaluate the controller's linear/cubic/quintic waypoint interpolation."""
    def seconds(value):
        return value.sec + value.nanosec * 1e-9
    points = trajectory.points
    if not points:
        raise ValueError('Empty trajectory')
    yield seconds(points[0].time_from_start), np.asarray(points[0].positions)
    for left, right in zip(points, points[1:]):
        start = seconds(left.time_from_start)
        duration = seconds(right.time_from_start) - start
        if duration <= 0:
            raise ValueError('Trajectory times must strictly increase')
        q0, q1 = np.asarray(left.positions), np.asarray(right.positions)
        c0, c1 = q0, q1 - q0
        c2 = c3 = c4 = c5 = np.zeros_like(q0)
        if left.velocities and right.velocities:
            c1 = np.asarray(left.velocities) * duration
            v1 = np.asarray(right.velocities) * duration
            if left.accelerations and right.accelerations:
                c2 = np.asarray(left.accelerations) * duration**2 / 2
                d = q1 - c0 - c1 - c2
                v = v1 - c1 - 2*c2
                a = np.asarray(right.accelerations)*duration**2 - 2*c2
                c3, c4, c5 = 10*d - 4*v + a/2, -15*d + 7*v - a, 6*d - 3*v + a/2
            else:
                c2, c3 = 3*(q1-q0)-2*c1-v1, 2*(q0-q1)+c1+v1
        for u in np.linspace(0, 1, max(2, int(np.ceil(duration/interval_s))+1))[1:]:
            yield start + u*duration, c0 + u*(c1 + u*(c2 + u*(c3 + u*(c4 + u*c5))))
