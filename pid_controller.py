class PIDController:
    def __init__(self, Kp,Ki,Kd, integral_limit):
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.integral=0
        self.previous_error=0
        self.integral_limit=integral_limit
        
    
    def compute(self, error, dt):
        self.integral = self.integral + error * dt
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        derivative = (error - self.previous_error) / dt
        command = self.Kp * error +  self.Ki * self.integral + self.Kd * derivative

        self.previous_error=error

        return command