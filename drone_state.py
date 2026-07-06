
class DroneState:
    def __init__(self, x, y, z, vx, vy, vz):
        # store each parameter as self.something
        self.x=x
        self.y=y
        self.z=z
        self.vx=vx
        self.vy=vy
        self.vz=vz


    def update(self, ax, ay, az, dt):
        # 1. update velocity using acceleration and dt
        # 2. update position using the (new) velocity and dt
        self.vx=self.vx + ax * dt
        self.x=self.x + self.vx * dt

        self.vy=self.vy + ay * dt
        self.y=self.y + self.vy * dt

        self.vz=self.vz + az * dt
        self.z=self.z + self.vz * dt
