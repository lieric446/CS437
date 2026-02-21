from picarx import Picarx
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib import colors
import math


#Global Variable
GRID_SIZE = 300                         # in cms
OBSTACLE_PADDING = 1                    # Padding (in cm) used to mark on all sides of obstacles.
CAR_LOCATION = (GRID_SIZE // 2, 0)             # Car position (x,y). bottom center of the grid
CAR_DIMENSIONS = (17, 23)
DIRECTION = (0, 1)        # Car direction as in (x, y).
MAP = np.zeros((GRID_SIZE, GRID_SIZE)     # Initialize empty map
                , dtype=np.int32)
def initMap():
    global CAR_LOCATION
    global MAP
    updateMapwithCar() # Mark the car's initial position on the map
def printMap():
    global MAP
    plt.figure(figsize=(6,6))
    plt.title("Map")
    plt.grid(True)
    cmap_list = ['white', 'red', 'green'] # Colors for values 0, 1, and 2 respectively
    bounds = [0., 0.5, 1.5, 2.5]          # Boundaries to map colors to data range

    # Create the custom colormap and normalizer
    cmap = colors.ListedColormap(cmap_list)
    norm = colors.BoundaryNorm(bounds, cmap.N)
    plt.imshow(MAP.T, origin='lower', cmap=cmap, norm=norm)
    plt.xticks(range(0, GRID_SIZE,30))
    plt.xticks(range(0, GRID_SIZE,30))
    plt.show()
def updateMapWithObstacle():
    global DIRECTION
    global CAR_LOCATION
    global MAP
    global GRID_SIZE
    global OBSTACLE_PADDING
    px.stop()
    car_heading_rad = math.atan2(DIRECTION[1], DIRECTION[0])
    for angle in range(-30, 30, 5):
        px.set_cam_pan_angle(angle)
        time.sleep(0.1)
        distance = px.get_distance()
        if distance < 50:  # If an obstacle is detected within 30 cm
            # Calculate the obstacle's position based on the car's current location and direction
            obs_x = CAR_LOCATION[0] + int(distance * math.cos(math.radians(angle)+car_heading_rad))
            obs_y = CAR_LOCATION[1] + int(distance * math.sin(math.radians(angle)+car_heading_rad))
            for dx in range(-OBSTACLE_PADDING, OBSTACLE_PADDING+1):    #interpolation - adding padding between the points
                for dy in range(-OBSTACLE_PADDING, OBSTACLE_PADDING + 1):
                    if 0 <= obs_x + dx < GRID_SIZE and 0 <= obs_y + dy < GRID_SIZE:
                        MAP[obs_x + dx, obs_y + dy] = 1
    px.set_cam_pan_angle(0)
    printMap()
def updateMapwithCar():
    global DIRECTION
    global CAR_LOCATION
    global MAP
    global GRID_SIZE
    px.stop()
    for dx in range(-CAR_DIMENSIONS[0]//2, CAR_DIMENSIONS[0]//2 + 1):
        for dy in range(-CAR_DIMENSIONS[1]//2, CAR_DIMENSIONS[1]//2 + 1):
            if 0 <= CAR_LOCATION[0] + dx < GRID_SIZE and 0 <= CAR_LOCATION[1] + dy < GRID_SIZE:
                MAP[CAR_LOCATION[0] + dx, CAR_LOCATION[1] + dy] = 2
def turnLeft():
    global DIRECTION
    global CAR_DIMENSIONS
    px.set_dir_servo_angle(29)
    px.backward(8)
    time.sleep(0.5)
    px.set_dir_servo_angle(-29)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(29)
    px.backward(8)
    time.sleep(0.5)
    px.set_dir_servo_angle(-29)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(30)
    px.backward(8)
    time.sleep(0.6)
    px.set_dir_servo_angle(-29)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(0)
    px.backward(8)
    time.sleep(0.7)
    (x,y) = DIRECTION
    DIRECTION = (-y,x)
    (a, b) = CAR_DIMENSIONS
    CAR_DIMENSIONS = (b, a)
    updateMapwithCar()
def turnRight():
    global DIRECTION
    global CAR_DIMENSIONS
    px.set_dir_servo_angle(-28)
    px.backward(8)
    time.sleep(0.5)
    px.set_dir_servo_angle(28)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(-28)
    px.backward(8)
    time.sleep(0.5)
    px.set_dir_servo_angle(28)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(-28)
    px.backward(8)
    time.sleep(0.5)
    px.set_dir_servo_angle(28)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(0)
    px.backward(8)
    time.sleep(0.6)
    (x, y) = DIRECTION
    DIRECTION = (y, -x)
    (a, b) = CAR_DIMENSIONS
    CAR_DIMENSIONS = (b, a)
    updateMapwithCar()
def turn180():
    turnRight()
    turnRight()
def forwardOneStep(): 
    global MAP
    global CAR_LOCATION
    global DIRECTION
    global CAR_DIMENSIONS
    px.forward(20)
    time.sleep(.8)
    px.set_dir_servo_angle(7)
    px.forward(20)
    time.sleep(.8)
    px.set_dir_servo_angle(0)
    CAR_LOCATION = (CAR_LOCATION[0] + 20*DIRECTION[0], CAR_LOCATION[1] + 20*DIRECTION[1])
    MAP[CAR_LOCATION[0], CAR_LOCATION[1]] = 2
    updateMapwithCar()
def backwardOneStep():
    global CAR_LOCATION
    global MAP
    global CAR_LOCATION
    px.backward(20)
    time.sleep(.65)
    px.set_dir_servo_angle(7)
    px.backward(20)
    time.sleep(.65)
    px.set_dir_servo_angle(0)
    CAR_LOCATION = (CAR_LOCATION[0] - 20*DIRECTION[0], CAR_LOCATION[1] - 20*DIRECTION[1])
    MAP[CAR_LOCATION[0], CAR_LOCATION[1]] = 2
    updateMapwithCar()
def main():
    time.sleep(3)  
    initMap()
    forwardOneStep()
    updateMapWithObstacle()
    forwardOneStep()
    updateMapWithObstacle()
    turnLeft()
    updateMapWithObstacle()
    forwardOneStep()
    updateMapWithObstacle()
    turnRight()
    updateMapWithObstacle()
    forwardOneStep()
    updateMapWithObstacle()
    forwardOneStep()
    updateMapWithObstacle()
    forwardOneStep()
    updateMapWithObstacle()
    forwardOneStep()
    updateMapWithObstacle()
    turnRight()
    updateMapWithObstacle()
    printMap()
def getCarLocation():
    global CAR_LOCATION
    carLocation = CAR_LOCATION
    return carLocation
def setCarLocation(carLocation):
    global CAR_LOCATION 
    CAR_LOCATION= carLocation
    
if __name__ == "__main__":
    
    try: 
        px = Picarx()
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally: 
        px.stop()
 
