"""Planar extensible cable, gravity, material transport and an inertial winch.

The uniform computational grid follows the shortening unstretched cable domain.
Node velocities are material velocities, not grid velocities (ALE formulation).
Positive drum speed reels rope in; positive cable x points towards the glider.
"""
from dataclasses import dataclass
import numpy as np

G = 9.80665


@dataclass
class Parameters:
    drum_mass_kg: float = 15.0
    radius_m: float = 0.125
    inertia_factor: float = 1.0  # Ring: J = m*r^2; solid cylinder: factor = 0.5.
    airborne_length_m: float = 1500.0  # Initial STRETCHED arc length.
    angle_deg: float = 15.0  # Chord from winch to glider, not the local exit angle.
    glider_speed_mps: float = 7.0  # Towards winch, along the initial chord.
    initial_drum_speed_mps: float = 7.0
    mass_per_m: float = 0.00170  # Armare Dynext PU SK75, diameter 1.7 mm.
    diameter_m: float = 0.0017
    # Armare Dynext PU: 1.95% at 30% BL, 3.15% at BL; BL = 287 daN.
    breaking_load_n: float = 2870.0
    strain_at_30_percent: float = 0.0195
    strain_at_break: float = 0.0315
    stiffness_scale: float = 1.0  # Sensitivity factor, 1 = catalogue curve.
    axial_damping_ns: float = 200.0  # BA: assumed Kelvin-Voigt axial viscosity.
    segments: int = 120
    initial_pull_kgf: float = 140.0
    brake_regularization_mps: float = 0.02
    gravity: float = G

    @property
    def inertia(self):
        return self.inertia_factor * self.drum_mass_kg * self.radius_m**2

    @property
    def drum_effective_mass(self):
        return self.inertia / self.radius_m**2

    @property
    def low_ea(self):
        return self.stiffness_scale * 0.30 * self.breaking_load_n / self.strain_at_30_percent

    @property
    def high_ea(self):
        return self.stiffness_scale * 0.70 * self.breaking_load_n / (self.strain_at_break - self.strain_at_30_percent)

    def strain(self, tension):
        knee = self.stiffness_scale * 0.30 * self.breaking_load_n
        return np.where(tension <= knee, tension / self.low_ea,
                        self.strain_at_30_percent + (tension-knee) / self.high_ea)

    def compliance(self, tension):
        return np.where(tension <= self.stiffness_scale * 0.30 * self.breaking_load_n,
                        1/self.low_ea, 1/self.high_ea)

    def tension(self, strain):
        return np.where(strain <= self.strain_at_30_percent, self.low_ea * np.maximum(strain, 0),
                        self.stiffness_scale * 0.30 * self.breaking_load_n +
                        self.high_ea * (strain-self.strain_at_30_percent))


def equilibrium_from_force(p, length, horizontal, vertical):
    """Exact discrete equilibrium: adjacent segment vertical forces differ by mg."""
    ds = length / p.segments
    v = vertical + np.arange(p.segments) * p.mass_per_m * p.gravity * ds
    f = np.column_stack((np.full(p.segments, horizontal), v))
    tension = np.linalg.norm(f, axis=1)
    edges = ds * (1.0 + p.strain(tension))[:, None] * f / tension[:, None]
    positions = np.vstack((np.zeros(2), np.cumsum(edges, axis=0)))
    return positions, tension


def initial_equilibrium(p):
    """Given drum tension, airborne arc length and chord angle, solve rest length."""
    tension0 = p.initial_pull_kgf * G
    angle = np.deg2rad(p.angle_deg)
    length = p.airborne_length_m / (1.0 + p.strain(tension0))
    for _ in range(15):
        lo, hi = -np.pi / 2 + 1e-5, np.pi / 2 - 1e-5
        for _ in range(65):
            phi = (lo + hi) / 2
            h, v = tension0 * np.cos(phi), tension0 * np.sin(phi)
            r, tensions = equilibrium_from_force(p, length, h, v)
            if np.arctan2(r[-1, 1], r[-1, 0]) < angle:
                lo = phi
            else:
                hi = phi
        arc = np.linalg.norm(np.diff(r, axis=0), axis=1).sum()
        if abs(arc - p.airborne_length_m) < 1e-8:
            return r, length, (h, v), tensions
        length *= p.airborne_length_m / arc
    raise RuntimeError('Initial catenary solve failed')


def equilibrium_at_endpoint(p, length, endpoint, guess):
    h, v = guess
    ds = length / p.segments
    for _ in range(35):
        r, tensions = equilibrium_from_force(p, length, h, v)
        error = r[-1] - endpoint
        if np.linalg.norm(error) < 1e-9:
            return r
        vertical = v + np.arange(p.segments) * p.mass_per_m * p.gravity * ds
        transverse = (1 + p.strain(tensions)) / tensions
        longitudinal = p.compliance(tensions)
        th, tv = h / tensions, vertical / tensions
        cross = np.sum(ds * (longitudinal-transverse) * th * tv)
        jacobian = np.array([
            [np.sum(ds * (transverse * tv**2 + longitudinal * th**2)), cross],
            [cross, np.sum(ds * (transverse * th**2 + longitudinal * tv**2))],
        ])
        step = np.linalg.solve(jacobian, error)
        scale = min(1.0, 0.5 * h / step[0]) if step[0] > 0 else 1.0
        h, v = np.array([h, v]) - scale * step
    raise RuntimeError('Endpoint catenary solve failed')


class CableDrum:
    def __init__(self, p):
        self.p = p
        self.r_initial, self.length_initial, guess, self.t_initial = initial_equilibrium(p)
        self.endpoint_initial = self.r_initial[-1].copy()
        self.chord_unit = self.endpoint_initial / np.linalg.norm(self.endpoint_initial)
        self.xi = np.linspace(0, 1, p.segments + 1)
        v0 = p.initial_drum_speed_mps
        feed_rate = v0 / (1 + p.strain(self.t_initial[0]))
        # Differentiate neighbouring static equilibria to avoid exciting the rope
        # merely by choosing inconsistent initial material velocities.
        epsilon = 0.001
        plus = equilibrium_at_endpoint(p, self.length_initial - feed_rate * epsilon,
            self.endpoint_initial - p.glider_speed_mps * epsilon * self.chord_unit, guess)
        minus = equilibrium_at_endpoint(p, self.length_initial + feed_rate * epsilon,
            self.endpoint_initial + p.glider_speed_mps * epsilon * self.chord_unit, guess)
        grid_velocity = (plus - minus) / (2 * epsilon)
        ds = self.length_initial / p.segments
        r_s = (self.r_initial[2:] - self.r_initial[:-2]) / (2 * ds)
        b = feed_rate * (1 - self.xi[1:-1])
        material_velocity = grid_velocity[1:-1] - b[:, None] * r_s
        # State: interior positions, interior material velocities, rest length, drum v.
        self.y0 = np.concatenate((self.r_initial[1:-1].ravel(), material_velocity.ravel(),
                                  [self.length_initial, v0]))

    def fields(self, time, y):
        p = self.p
        count = 2 * (p.segments - 1)
        r = np.empty((p.segments + 1, 2))
        u = np.empty_like(r)
        r[0] = 0
        r[-1] = self.endpoint_initial - p.glider_speed_mps * time * self.chord_unit
        r[1:-1] = y[:count].reshape(-1, 2)
        u[1:-1] = y[count:2*count].reshape(-1, 2)
        length, drum_v = y[-2:]
        ds = length / p.segments
        edges = np.diff(r, axis=0)
        edge_length = np.linalg.norm(edges, axis=1)
        tangent = edges / edge_length[:, None]
        stretch = edge_length / ds
        u[0] = -drum_v * tangent[0]
        u[-1] = -p.glider_speed_mps * self.chord_unit
        strain_rate = np.sum(np.diff(u, axis=0) * tangent, axis=1) / ds
        elastic = p.tension(stretch - 1)
        tension = np.maximum(elastic + p.axial_damping_ns * strain_rate, 0)
        tension[stretch <= 1] = 0  # Cable cannot push, including when slack.
        feed_rate = drum_v / stretch[0]
        return r, u, tension, tangent, stretch, ds, feed_rate

    def motor_force(self, requested_kgf, braking, drum_v):
        if braking:
            # Passive brake: torque opposes rotation, with a narrow smooth zero-speed zone.
            return -abs(requested_kgf) * G * np.tanh(drum_v / self.p.brake_regularization_mps)
        return requested_kgf * G

    def rhs(self, time, y, requested_kgf, braking):
        p = self.p
        r, u, tension, tangent, stretch, ds, feed = self.fields(time, y)
        f = tension[:, None] * tangent
        acceleration = (f[1:] - f[:-1]) / (p.mass_per_m * ds)
        acceleration[:, 1] -= p.gravity
        b = feed * (1 - self.xi[1:-1])
        grid_velocity = u[1:-1] + b[:, None] * (r[2:] - r[:-2]) / (2 * ds)
        material_velocity_rate = acceleration + b[:, None] * (u[2:] - u[:-2]) / (2 * ds)
        drive = self.motor_force(requested_kgf, braking, y[-1])
        drum_a = (drive - tension[0]) / p.drum_effective_mass
        return np.concatenate((grid_velocity.ravel(), material_velocity_rate.ravel(), [-feed, drum_a]))

    def observe(self, time, y, requested_kgf, braking):
        r, u, tension, tangent, stretch, ds, feed = self.fields(time, y)
        drive = self.motor_force(requested_kgf, braking, y[-1])
        return dict(time_s=time, command_kgf=requested_kgf, braking=int(braking),
                    motor_force_kgf=drive/G, rope_force_kgf=tension[0]/G,
                    pilot_force_kgf=tension[-1]/G, speed_mps=y[-1],
                    acceleration_mps2=(drive-tension[0])/self.p.drum_effective_mass,
                    rpm=y[-1]/self.p.radius_m*60/(2*np.pi), rest_length_m=y[-2],
                    min_height_m=float(r[1:-1, 1].min()), min_strain=float(stretch.min()-1),
                    max_strain=float(stretch.max()-1))


def integrate(model, command, end_time=6.0, dt=0.00025, sample_dt=0.005,
              stop_at_ground=True, stop_at_strain_limit=False):
    """RK4, aligned with every 1 ms command sample and every output sample."""
    substeps = round(0.001 / dt)
    assert substeps >= 1 and abs(substeps * dt - 0.001) < 1e-12
    stride = round(sample_dt / dt)
    steps = round(end_time / dt)
    y = model.y0.copy()
    rows, snapshots = [], {}
    for step in range(steps + 1):
        time = step * dt
        if callable(command):
            if step % substeps == 0:
                requested, braking = command(time, model, y)
        else:
            command_index = min(step // substeps, len(command) - 1)
            requested, braking = command[command_index]
        if step % stride == 0:
            rows.append(model.observe(time, y, requested, bool(braking)))
            if callable(command):
                rows[-1].update(command.observe())
            # Do not extrapolate the intact-rope law beyond its catalogue range.
            if stop_at_strain_limit and rows[-1]['max_strain'] >= model.p.strain_at_break:
                snapshots[f'{time:g}'] = model.fields(time, y)[0].copy()
                break
            # Ground contact is outside this airborne-cable model. Do not continue
            # with a cable passing through the ground, or silently clamp its shape.
            if stop_at_ground and rows[-1]['min_height_m'] <= 0:
                snapshots[f'{time:g}'] = model.fields(time, y)[0].copy()
                break
        if step % round(1.0 / dt) == 0:
            snapshots[f'{time:g}'] = model.fields(time, y)[0].copy()
        if step == steps:
            break
        k1 = model.rhs(time, y, requested, bool(braking))
        k2 = model.rhs(time + dt/2, y + dt*k1/2, requested, bool(braking))
        k3 = model.rhs(time + dt/2, y + dt*k2/2, requested, bool(braking))
        k4 = model.rhs(time + dt, y + dt*k3, requested, bool(braking))
        y += dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        if step % 1000 == 0 and not np.isfinite(y).all():
            raise RuntimeError(f'Non-finite state at {time}s')
    return rows, snapshots
