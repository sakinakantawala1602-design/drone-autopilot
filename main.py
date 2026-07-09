from drone_state import DroneState

if __name__ == "__main__":    #checks if the file is being excecuted directly or is imported and excecuted in another file
    drone1 = DroneState(0, 0, 0, 0, 0, 0,'Drone A')
    drone2 = DroneState(1, 0, 0, 0, 0, 1,'Drone B')
    drone3 = DroneState(0, 1, 0, 1, 0, 0,'Drone C')
    drones_list=[drone1,drone2,drone3]
    for drone in drones_list:
        drone.update(ax=1, ay=0, az=0, dt=1)
        speed= drone.speed()
        print(drone.name,drone.x, drone.y, drone.z, drone.vx, drone.vy, drone.vz)
        print(speed)



    