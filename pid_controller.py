class PIDController:
    def __init__(self, Kp,Ki,Kd):
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.integral=0
        self.previous_error=0
    
    def compute(self, error, dt):
        self.integral = self.integral + error * dt
        derivative = (error - self.previous_error) / dt
        command = self.Kp * error +  self.Ki * self.integral + self.Kd * derivative

        self.previous_error=error

        return command