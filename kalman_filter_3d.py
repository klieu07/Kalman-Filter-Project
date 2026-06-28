import numpy as np
 
G = 9.80665                  # gravity, m/s^2
_M_PER_DEG_LAT = 110540.0
 
 
def make_projector3d(lat0, lon0):
    """Local-tangent projection around the first fix -> East/North meters (Up = altitude)."""
    m_per_deg_lon = 111320.0 * np.cos(np.radians(lat0))
 
    def project(lat, lon, alt):
        e = (lon - lon0) * m_per_deg_lon
        n = (lat - lat0) * _M_PER_DEG_LAT
        return e, n, alt
    return project
 
 
def body_to_nav_accel(ax_mg, ay_mg, az_mg, roll_deg, pitch_deg, yaw_deg):
    """
    Rotate body-frame accelerometer reading (in mg) into the navigation frame,
    remove gravity, and return linear acceleration [ae, an, au] in m/s^2.
    """
    # mg -> m/s^2
    a_body = np.array([ax_mg, ay_mg, az_mg], dtype=float) / 1000.0 * G
 
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
 
    # ZYX rotation: body -> nav (matches firmware's convention)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])
 
    a_nav = R @ a_body
    a_nav[2] -= G                      # remove gravity from the Up axis
    return a_nav                       # [ae, an, au] m/s^2
 
 
class KalmanFilter3D:
    def __init__(self, sigma_a=1.5, sigma_gps_horiz=2.5, sigma_gps_vert=5.0):
        self.sigma_a = sigma_a                 # process accel noise (m/s^2)
        self.x = np.zeros(6)
        self.P = np.eye(6) * 100.0
        # measurement model: GPS observes position (E,N,U)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0
        self.R = np.diag([sigma_gps_horiz**2, sigma_gps_horiz**2, sigma_gps_vert**2])
        self.initialized = False
 
    def initialize(self, pe, pn, pu):
        self.x = np.array([pe, pn, pu, 0.0, 0.0, 0.0])
        self.P = np.diag([4, 4, 25, 4, 4, 4]).astype(float)
        self.initialized = True
 
    def predict(self, dt, a_nav=None):
        if not self.initialized or dt <= 0:
            return
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        # control input B maps acceleration -> state
        B = np.zeros((6, 3))
        B[0, 0] = B[1, 1] = B[2, 2] = 0.5 * dt * dt
        B[3, 0] = B[4, 1] = B[5, 2] = dt
 
        u = np.zeros(3) if a_nav is None else np.asarray(a_nav, dtype=float)
        self.x = F @ self.x + B @ u
 
        # process noise from random-acceleration model
        q = self.sigma_a ** 2
        dt2, dt3, dt4 = dt*dt, dt**3, dt**4
        Qpp = dt4/4 * q
        Qpv = dt3/2 * q
        Qvv = dt2 * q
        Q = np.zeros((6, 6))
        for i in range(3):
            Q[i, i]       = Qpp
            Q[i, i+3]     = Qpv
            Q[i+3, i]     = Qpv
            Q[i+3, i+3]   = Qvv
        self.P = F @ self.P @ F.T + Q
 
    def update(self, ze, zn, zu):
        if not self.initialized:
            self.initialize(ze, zn, zu)
            return
        z = np.array([ze, zn, zu])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
 
    @property
    def position(self):
        return self.x[0], self.x[1], self.x[2]    # E, N, U
 
    @property
    def velocity(self):
        return self.x[3], self.x[4], self.x[5]