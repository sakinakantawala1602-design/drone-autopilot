class KalmanFilter:
    def __init__(self, estimate, variance, process_noise):
        self.estimate=estimate
        self.variance=variance
        self.process_noise= process_noise
    
    def predict(self, velocity, dt):
        self.estimate = self.estimate + velocity * dt
        self.variance = self.variance + self.process_noise
    
    def update(self, measurement, measurement_variance):
        self.estimate = (self.estimate/self.variance + measurement/measurement_variance) / (1/self.variance + 1/measurement_variance)
        self.variance= 1/ (1/self.variance + 1/measurement_variance)