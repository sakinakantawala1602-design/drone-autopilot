from drone_state import DroneState
from pid_controller import PIDController
from kalman_filter import KalmanFilter
import random

'''if __name__ == "__main__":    #checks if the file is being excecuted directly or is imported and excecuted in another file
    drone1 = DroneState(0, 0, 0, 0, 0, 0,'Drone A')
    drone2 = DroneState(1, 0, 0, 0, 0, 1,'Drone B')
    drone3 = DroneState(0, 1, 0, 1, 0, 0,'Drone C')
    drones_list=[drone1,drone2,drone3]
    for drone in drones_list:
        drone.wz=10
        drone.update(ax=1, ay=0, az=0, dt=1)
        speed= drone.speed()
        print(drone.name,drone.x, drone.y, drone.z, drone.vx, drone.vy, drone.vz)
        print(speed)
        print(drone.yaw)
        
        drone.update(ax=1, ay=0, az=0, dt=1)
        print(drone.yaw)


target = 50  # meters
m = 1  # kg
g = 9.8

drone = DroneState(0, 0, 0, 0, 0, 0, "TestDrone")
pid = PIDController(Kp=2 , Ki=1, Kd=1, integral_limit=10)

for step in range(10000):  # simulate 50 time steps
    error = target - drone.z
    command = pid.compute(error, dt=0.01)
    az = g + command/m
    drone.update(ax=0, ay=0, az=az, dt=0.01)
    if step%100==0:
        print(step, drone.z)

kf = KalmanFilter(estimate=0, variance=1, process_noise=0.01)

# Step 1: predict, then update with a noisy measurement
kf.predict(velocity=1, dt=1)
kf.update(measurement=0.9, measurement_variance=0.5)
print(kf.estimate, kf.variance)

# Step 2: predict again, then update with another noisy measurement
kf.predict(velocity=1, dt=1)
kf.update(measurement=2.2, measurement_variance=0.5)
print(kf.estimate, kf.variance)




drone = DroneState(0, 0, 0, 0, 0, 1, "TrueDrone")  # vz=1, climbing steadily
kf = KalmanFilter(estimate=0, variance=1, process_noise=0.01)

sensor_sd = 3  # how noisy our simulated GPS is

for step in range(20):
    drone.update(ax=0, ay=0, az=0, dt=1)  # true physics: climbs at constant vz=1
    
    kf.predict(velocity=drone.vz, dt=1)
    
    noisy_measurement = random.gauss(drone.z, sensor_sd)
    kf.update(measurement=noisy_measurement, measurement_variance=sensor_sd**2)
    
    print(f"true_z={drone.z:.2f}, noisy_measurement={noisy_measurement:.2f}, kf_estimate={kf.estimate:.2f}")'''




drone = DroneState(0, 0, 0, 0, 0, 0, "AutopilotDrone")
pid = PIDController(Kp=2, Ki=1, Kd=1, integral_limit=8)
kf = KalmanFilter(estimate=0, variance=1, process_noise=0.01)

target = 10
m = 1
g = 9.8
sensor_sd = 0.5
dt = 0.01

for step in range(2000):
    # 1. SENSE: noisy measurement of drone.z
    noisy_measurement = random.gauss(drone.z, sensor_sd)
    # 2. ESTIMATE: kf.predict(...) using drone.vz, then kf.update(...) with the noisy measurement
    kf.predict(velocity=drone.vz, dt=dt)
    kf.update(measurement=noisy_measurement, measurement_variance=sensor_sd**2)
    # 3. DECIDE: error = target - kf.estimate (NOT drone.z!), then pid.compute(...)
    error = target - kf.estimate
    command = pid.compute(error, dt)

    # 4. Convert command to az
    az= g + command/m
    # 5. ACT: drone.update(...)
    drone.update(ax=0, ay=0, az=az, dt=dt)
    
    if step % 100 == 0:
        print(f"true_z={drone.z:.2f}, kf_estimate={kf.estimate:.2f}, command={command}")