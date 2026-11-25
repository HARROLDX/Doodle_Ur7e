"""URDF forward kinematics and bounded tip speed for straight joint segments."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


class TipKinematics:
    def __init__(self, xml, base, link, names, tip):
        joints = {j.find('child').get('link'): j for j in ET.fromstring(xml).findall('joint')}
        chain = []
        visited = set()
        while link != base:
            if link in visited or link not in joints:
                raise ValueError('URDF has no supported base-to-tool chain')
            visited.add(link)
            j = joints[link]
            chain.append(j)
            link = j.find('parent').get('link')
        self.chain = []
        self.tip = np.asarray(tip, dtype=float)
        self.reach = float(np.linalg.norm(self.tip))
        active = []
        for j in reversed(chain):
            kind = j.get('type')
            if kind not in ('fixed', 'revolute', 'continuous'):
                raise ValueError('tip timing supports fixed/revolute URDF joints only')
            origin = j.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            T[:3, 3] = xyz
            self.reach += float(np.linalg.norm(xyz))
            index, axis = None, None
            if kind != 'fixed':
                index = names.index(j.get('name'))
                active.append(j.get('name'))
                a = j.find('axis')
                axis = np.fromstring(a.get('xyz', '1 0 0') if a is not None else '1 0 0', sep=' ')
                if len(axis) != 3 or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-9:
                    raise ValueError('invalid URDF joint axis')
                axis /= np.linalg.norm(axis)
            self.chain.append((T, index, axis))
        if set(active) != set(names) or len(active) != len(names):
            raise ValueError('URDF timing chain differs from trajectory joints')

    def wrist(self, q):
        T = np.eye(4)
        for origin, index, axis in self.chain:
            T = T @ origin
            if index is not None:
                R = np.eye(4)
                R[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                T = T @ R
        return T

    def point(self, q):
        T = self.wrist(q)
        return T[:3, 3] + T[:3, :3] @ self.tip

    def segment_bounds(self, positions, subdivisions=32):
        """Upper bound |dp/ds|, s in [0,1], for q=q0+s*(q1-q0).

        The URDF translation-length sum bounds every downstream lever arm.
        Product-rule differentiation bounds |d²p/ds²| by reach*sum(|dq|)².
        A derivative differs from its interval-average chord by <= M*h/2.
        Thus samples plus this analytic remainder bound the entire segment,
        rather than assuming endpoint chords equal traveled tip distance.
        """
        if not isinstance(subdivisions, int) or subdivisions < 1:
            raise ValueError('positive subdivisions required')
        result = []
        for a, b in zip(positions[:-1], positions[1:]):
            dq = np.asarray(b) - a
            points = np.array([self.point(np.asarray(a) + s*dq)
                               for s in np.linspace(0, 1, subdivisions+1)])
            chord = np.linalg.norm(np.diff(points, axis=0), axis=1).max()*subdivisions
            curvature = self.reach * np.abs(dq).sum()**2
            result.append(chord + curvature/(2*subdivisions))
        return np.asarray(result)
