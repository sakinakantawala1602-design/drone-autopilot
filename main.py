from drone_state import DroneState
from pid_controller import PIDController

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
        print(drone.yaw)'''


target = 10  # meters
m = 1  # kg
g = 9.8

drone = DroneState(0, 0, 0, 0, 0, 0, "TestDrone")
pid = PIDController(Kp=2 , Ki=1, Kd=1)

for step in range(5000):  # simulate 50 time steps
    error = target - drone.z
    command = pid.compute(error, dt=0.01)
    az = g + command/m
    drone.update(ax=0, ay=0, az=az, dt=0.01)
    if step%100==0:
        print(step, drone.z)    