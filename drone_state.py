
class DroneState:
    def __init__(self, x, y, z, vx, vy, vz,name):
        # store each parameter as self.something
        self.x=x
        self.y=y
        self.z=z
        self.vx=vx
        self.vy=vy
        self.vz=vz
        self.name=name


    def update(self, ax, ay, az, dt):
        # 1. update velocity using acceleration and dt
        # 2. update position using the (new) velocity and dt
        self.vx=self.vx + ax * dt
        self.x=self.x + self.vx * dt

        self.vy=self.vy + ay * dt
        self.y=self.y + self.vy * dt

        self.vz=self.vz + az * dt
        self.z=self.z + self.vz * dt

    def speed(self):
        speed = (self.vx**2 + self.vy**2 + self.vz**2) ** 0.5 
        '''cannot be self.speed,
            cuz then it would store the value on 
            the vairable and is no longer a local variable and 
            dron.speed would also deem the function call useless after one run of code'''
        return speed
        