from drone_state import DroneState

if __name__ == "__main__":    #checks if the file is being excecuted directly or is imported and excecuted in another file
    drone = DroneState(0, 0, 0, 0, 0, 0)
    drone.update(ax=1, ay=0, az=0, dt=1)
    print(drone.x, drone.y, drone.z, drone.vx, drone.vy, drone.vz)