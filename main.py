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


'''#1D

drone = DroneState(0, 0, 0, 0, 0, 0, "AutopilotDrone")
pid = PIDController(Kp=2, Ki=1, Kd=3, integral_limit=11)
kf = KalmanFilter(estimate=10, variance=1, process_noise=0.01)

m = 1
g = 9.8
sensor_sd = 0.5
dt = 0.01
waypoints = [10, 20, 5]
current_index = 0
target = waypoints[current_index]
tolerance = 1.0


for step in range(6000):
    # 1. SENSE: noisy measurement of drone.x
    noisy_measurement = random.gauss(drone.z, sensor_sd)
    # 2. ESTIMATE: kf.predict(...) using drone.vz, then kf.update(...) with the noisy measurement
    kf.predict(velocity=drone.vz, dt=dt)
    kf.update(measurement=noisy_measurement, measurement_variance=sensor_sd**2)
    # 3. DECIDE: error = target - kf.estimate (NOT drone.z!), then pid.compute(...)
    error = target - kf.estimate
    tolerance=1
    if abs(error)<tolerance:
        if current_index+1 < len(waypoints):
            current_index+=1
        target = waypoints[current_index]


    command = pid.compute(error, dt)

    # 4. Convert command to az
    az= g+ command/m
    # 5. ACT: drone.update(...)
    drone.update(az=az, ay=0, ax=0, dt=dt)
    
    if step<600:
        print(f"step={step}, target={target}, true_z={drone.z:.2f}, kf_estimate={kf.estimate:.2f}, current_index={current_index}")'''

 
#3D

axis_names= ['x','y','z']
velocity_names= ['vx','vy','vz']
waypoints = [(5, 3, 10), (10, 10, 20), (0, 0, 5)]
current_index = 0
target = waypoints[current_index]
m = 1
g = 9.8
sensor_sd = 0.5
dt = 0.01
tolerance=1
vel_tolerance=2

drone = DroneState(0, 0, 0, 0, 0, 0, "AutopilotDrone")

kf_x= KalmanFilter(estimate=0, variance=1, process_noise=0.01)
kf_y = KalmanFilter(estimate=0, variance=1, process_noise=0.01)
kf_z= KalmanFilter(estimate=0, variance=1, process_noise=0.01)
kalman_filters=[kf_x,kf_y,kf_z]

pid_x= PIDController(Kp=2, Ki=1, Kd=1, integral_limit=8)
pid_y = PIDController(Kp=2, Ki=1, Kd=1, integral_limit=8)
pid_z = PIDController(Kp=2, Ki=1, Kd=3, integral_limit=11)
pid_controllers= [pid_x,pid_y,pid_z]


previous_index = current_index

for step in range(7000):
    accelerations = []
    errors=[]
    velocities=[]
    for i in range(3):
        # 1. get the true position for this axis (drone.x, drone.y, or drone.z)
        true_pos=getattr(drone, axis_names[i])
        axis_velocity=getattr(drone, velocity_names[i])
        velocities.append(axis_velocity)

        # 2. simulate a noisy measurement of that true position
        noisy_measurement = random.gauss(true_pos, sensor_sd)

        # 3. kalman_filters[i].predict(velocity=???, dt=dt)
        kalman_filters[i].predict(velocity=axis_velocity, dt=dt)

        # 4. kalman_filters[i].update(measurement=???, measurement_variance=sensor_sd**2)
        kalman_filters[i].update(measurement=noisy_measurement, measurement_variance=sensor_sd**2)

        # 5. compute error using target[i] and kalman_filters[i].estimate
        error = target[i] - kalman_filters[i].estimate
        errors.append(error)

        if abs(error) < tolerance  and abs(axis_velocity) < vel_tolerance :
            pid_controllers[i].reset()

            
        # 6. command = pid_controllers[i].compute(error, dt)
        command = pid_controllers[i].compute(error, dt)
        if i == 2:
            a=g+ command/m
        else:
            a=command/m
        accelerations.append(a)
        # 7. store this command somewhere (you'll need ax, ay, az all computed before calling drone.update)
    

    drone.update(ax=accelerations[0], ay=accelerations[1], az=accelerations[2], dt=dt)

    if current_index==2 and step%200==0:
        print(f'true_z={drone.z:.2f}, true_x={drone.x:.2f},true_y={drone.y:.2f}, vel_x= {drone.vx: .2f}, vel_y= {drone.vy: .2f}, vel_z= {drone.vz: .2f}')

    
    if all([abs(e) < tolerance for e in errors]) and all([abs(v) < vel_tolerance for v in velocities]):
        if current_index+1 < len(waypoints):
            current_index+=1
            
        target = waypoints[current_index]


    if current_index != previous_index:
        print(f'step={step},true_z={drone.z:.2f}, true_x={drone.x:.2f}, true_y={drone.y:.2f}, current_index={current_index}')
        previous_index = current_index
    
    '''if step<500:
        print(f"step={step}, target={target}, true_z={drone.z:.2f}, kf_estimate={kf_z.estimate:.2f}, current_index={current_index}")'''

