from picarx import Picarx
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib import colors
import math
from heapq import heappush, heappop

def heuristic(pos, goal):
    """Manhattan distance heuristic"""
    return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])


def astar(start, goal, grid):
    """A* pathfinding algorithm"""
    
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    closed_set = set()
    
    while open_set:
        _, current = heappop(open_set)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        closed_set.add(current)
        
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if (0 <= neighbor[0] < GRID_SIZE and 
                0 <= neighbor[1] < GRID_SIZE and 
                neighbor not in closed_set and 
                grid[neighbor[0], neighbor[1]] != 1):
                
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))
    
    return []


def navigateToGoal(goal):
    """Move car to goal coordinate using A* path"""
    global CAR_LOCATION, DIRECTION
    path = astar(CAR_LOCATION, goal, MAP)
    print(f"Path to goal: {path}")
    print("Awwal")
    
    # printMap()
    updateMapWithObstacle()  # Update the map with any detected obstacles
    print("Updated map:")
    print(MAP)
    if path:  # Check if a valid path exists
        for next_pos in path:
            print(f"Next waypoint: {next_pos}")
            while CAR_LOCATION != next_pos:
                print(f"Current: {CAR_LOCATION}, Next: {next_pos}, Direction: {DIRECTION}")
                print(MAP)
                dx = next_pos[0] - CAR_LOCATION[0]
                dy = next_pos[1] - CAR_LOCATION[1]
                target_dir = (0, 0)
                if dx > 0: target_dir = (1, 0)
                elif dx < 0: target_dir = (-1, 0)
                elif dy > 0: target_dir = (0, 1)
                elif dy < 0: target_dir = (0, -1)

                if CAR_LOCATION == goal:
                    print("Goal reached!")
                    break
                # if DIRECTION != target_dir:
                #     if target_dir == (1, 0):
                #         turnRight()
                #     elif target_dir == (-1, 0):
                #         turnLeft()
                #     elif target_dir == (0, 1):
                #         turnRight()
                #         turnRight()
                #     elif target_dir == (0, -1):
                #         turnRight()
                #         turnRight()
                #         turnRight()
                # if DIRECTION != target_dir:
                print(f"Target direction: {target_dir}, Current direction: {DIRECTION}")
                print(MAP[next_pos[0], next_pos[1]])

                # if MAP[next_pos[0], next_pos[1]] == 2:
                #     # updateMapwithCar()
                #     break
                if MAP[next_pos[0], next_pos[1]] == 0:
                    forwardOneStep()  # Move forward if the path ahead is free
                elif MAP[next_pos[0] - 1, next_pos[1]] == 0:
                    print("Turning left to avoid obstacle...")
                    turnLeft()  # Turn left if the left side is free
                    forwardOneStep()
                    turnRight()  # Turn back to original direction after moving
                elif MAP[next_pos[0], next_pos[1] + 1] == 0:
                    print("Turning right to avoid obstacle...")
                    turnRight()  # Turn right if the left side is blocked but right is free
                    forwardOneStep()
                    turnLeft()  # Turn back to original direction after moving
                else:
                    print("Stopping, no path available.")
                
                updateMapwithCar()  # Update the map with the car's new position
                updateMapWithObstacle()  # Continuously update the map while moving
                path = astar(CAR_LOCATION, goal, MAP)
                print("Updated map2:")
                print(MAP)
                # return
    else:
        print("No path found to the goal.")
#Global Variable
GRID_SIZE = 20                           # in cms
OBSTACLE_PADDING = 1                     # Padding (in cm) used to mark on all sides of obstacles.
CAR_LOCATION = (0, 0)        # Car position (x,y). bottom center of the grid
CAR_DIMENSIONS = (1,1)
DIRECTION = (0, 1)                         # Car direction as in (x, y).
MAP = np.zeros((GRID_SIZE, GRID_SIZE)     # Initialize empty map
                , dtype=np.int32)
STEP_DISTANCE = 1                       # Distance (in cm) for each forward/backward step
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
    print(f"Car Location: {CAR_LOCATION}, Direction: {DIRECTION}, Heading (rad): {car_heading_rad}")
    for angle in range(-30, 30, 5):
        px.set_cam_pan_angle(angle)
        time.sleep(0.1)
        distance = px.get_distance()
        print("before", CAR_LOCATION)
        print(MAP[CAR_LOCATION[0]:CAR_LOCATION[0]+30, CAR_LOCATION[1]:CAR_LOCATION[1]+30])
        MAP[CAR_LOCATION[0], CAR_LOCATION[1]] = 2
        for i in range(0,MAP.shape[0]):
                for j in range(0, MAP.shape[1]):
                    MAP[i, j] = 0
        if distance < 50: # If an obstacle is detected within 50 cm
            # Calculate the obstacle's position based on the car's current location and direction
            obs_x = CAR_LOCATION[0] + int(distance * math.cos(math.radians(angle)+car_heading_rad))//15
            obs_y = CAR_LOCATION[1] + int(distance * math.sin(math.radians(angle)+car_heading_rad))//15
            print(f"Obstacle detected at distance: {distance} cm, Estimated Position: ({obs_x}, {obs_y})")
            # for i in range(0,MAP.shape[0]):
            #     for j in range(0, MAP.shape[1]):
            #         MAP[i, j] = 0
            MAP[CAR_LOCATION[0], CAR_LOCATION[1]] = 2
            for dx in range(-OBSTACLE_PADDING, OBSTACLE_PADDING+1):   
                for dy in range(-OBSTACLE_PADDING, OBSTACLE_PADDING + 1):
                    if 0 <= obs_x + dx < GRID_SIZE and 0 <= obs_y + dy < GRID_SIZE:
                        print(f"Marking obstacle at: ({obs_x + dx}, {obs_y + dy})")
                        MAP[obs_x + dx, obs_y + dy] = 1
            print(MAP[CAR_LOCATION[0]:CAR_LOCATION[0]+30, CAR_LOCATION[1]:CAR_LOCATION[1]+30])
            print("after")
    px.set_cam_pan_angle(0)
    #printMap()
def updateMapwithCar():
    global DIRECTION
    global CAR_LOCATION
    global MAP
    global GRID_SIZE
    px.stop()
    # for dx in range(-CAR_DIMENSIONS[0]//2, CAR_DIMENSIONS[0]//3):
    #     for dy in range(-CAR_DIMENSIONS[1]//2, CAR_DIMENSIONS[1]//3):
    #         if 0 <= CAR_LOCATION[0] + dx < GRID_SIZE and 0 <= CAR_LOCATION[1] + dy < GRID_SIZE:
    #             MAP[CAR_LOCATION[0] + dx, CAR_LOCATION[1] + dy] = 2
def turnLeft():
    global DIRECTION
    global CAR_DIMENSIONS
    global CAR_LOCATION
    px.set_dir_servo_angle(20)
    px.backward(8)
    time.sleep(0.5)
    px.set_dir_servo_angle(-20)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(20)
    px.backward(8)
    time.sleep(0.5)
    px.set_dir_servo_angle(-20)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(20)
    px.backward(8)
    time.sleep(0.6)
    px.set_dir_servo_angle(-20)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(0)
    px.backward(6)
    time.sleep(0.6)
    (i, j) = CAR_LOCATION
    (x,y) = DIRECTION
    (a, b) = CAR_DIMENSIONS
    CAR_LOCATION = b//4, a//4
    if (x, y) == (1,0):
        CAR_LOCATION = (i - b//4, j + a // 4)
    if (x, y) == (0,1):
        CAR_LOCATION = (i - b//4, j - a // 4)
    if (x, y) == (-1,0):
        CAR_LOCATION = (i + b//4, j - a // 4)
    if (x, y) == (0,-1):
        CAR_LOCATION = (i + b//4, j + a // 4)
    DIRECTION = (-y,x)
    CAR_DIMENSIONS = (b, a)
    print(f"Turned left. New direction: {DIRECTION}")
    updateMapwithCar()

# def turnLeft():
#     """Turn left (counter-clockwise 90 degrees) in place"""
#     global DIRECTION
#     global CAR_DIMENSIONS
    
#     print("Turning left...")
    
#     # Turn left using steering and wheel movement
#     # Steer left (positive angle) and move backward to rotate counter-clockwise
#     px.set_dir_servo_angle(35)  # Full left steering
#     px.backward(15)             # Move backward while turning
#     time.sleep(0.7)
#     px.set_dir_servo_angle(35)   # Reset steering to center
#     px.forward(30)             # Move forward to complete the turn
#     time.sleep(0.7)
#     px.set_dir_servo_angle(0)  # Full left steering
#     px.backward(15)             # Move forward while turning
#     time.sleep(0.7)
#     px.stop()
#     time.sleep(0.3)
    
#     # Update direction: counter-clockwise 90 degrees rotation
#     # (1,0) -> (0,1), (0,1) -> (-1,0), (-1,0) -> (0,-1), (0,-1) -> (1,0)
#     (x, y) = DIRECTION
#     DIRECTION = (-y, x)
    
#     # Swap dimensions since car orientation changed
#     (a, b) = CAR_DIMENSIONS
#     CAR_DIMENSIONS = (b, a)
    
#     print(f"Turned left. New direction: {DIRECTION}")
#     updateMapwithCar()

def turnRight():
    global DIRECTION
    global CAR_DIMENSIONS
    global CAR_LOCATION
    px.set_dir_servo_angle(-20)
    px.backward(8)
    time.sleep(0.5)
    px.set_dir_servo_angle(20)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(-20)
    px.backward(8)
    time.sleep(0.6)
    px.set_dir_servo_angle(20)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(-20)
    px.backward(8)
    time.sleep(0.6)
    px.set_dir_servo_angle(20)
    px.forward(12)
    time.sleep(0.6)
    px.set_dir_servo_angle(0)
    px.backward(8)
    time.sleep(0.6)
    (i, j) = CAR_LOCATION
    (x, y) = DIRECTION
    (a, b) = CAR_DIMENSIONS
    if (x, y) == (1,0):
        CAR_LOCATION = (i - b//4, j - a // 4)
    if (x, y) == (0,1):
        CAR_LOCATION = (i + b//4, j - a // 4)
    if (x, y) == (-1,0):
        CAR_LOCATION = (i + b//4, j + a // 4)
    if (x, y) == (0,-1):
        CAR_LOCATION = (i - b//4, j - a // 4)
    DIRECTION = (y, -x)
    CAR_DIMENSIONS = (b, a)
    print(f"Turned right. New direction: {DIRECTION}")
    updateMapwithCar()
def turn180():
    turnRight()
    turnRight()
def forwardOneStep():
    global MAP
    global CAR_LOCATION
    global DIRECTION
    global STEP_DISTANCE
    px.forward(STEP_DISTANCE)
    time.sleep(.4)
    px.set_dir_servo_angle(0)
    px.forward(STEP_DISTANCE)
    time.sleep(.2)
    px.set_dir_servo_angle(0)
    CAR_LOCATION = (CAR_LOCATION[0] + STEP_DISTANCE*DIRECTION[0], CAR_LOCATION[1] + STEP_DISTANCE*DIRECTION[1])
    # MAP[CAR_LOCATION[0], CAR_LOCATION[1]] = 2
    updateMapwithCar()
def backwardOneStep():
    global CAR_LOCATION
    global MAP
    global DIRECTION
    global STEP_DISTANCE
    px.backward(STEP_DISTANCE)
    time.sleep(.4)
    px.set_dir_servo_angle(9)
    px.backward(STEP_DISTANCE)
    time.sleep(.65)
    px.set_dir_servo_angle(0)
    CAR_LOCATION = (CAR_LOCATION[0] - STEP_DISTANCE*DIRECTION[0], CAR_LOCATION[1] - STEP_DISTANCE*DIRECTION[1])
    # MAP[CAR_LOCATION[0], CAR_LOCATION[1]] = 2
    updateMapwithCar()
def main():
    time.sleep(3)  
    initMap()
    forwardOneStep()
    updateMapWithObstacle()
    turnLeft()
    updateMapWithObstacle()
    forwardOneStep()
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
    forwardOneStep()
    updateMapWithObstacle()
    forwardOneStep()
    updateMapWithObstacle()
    turnLeft()
    updateMapWithObstacle()
    forwardOneStep()
    updateMapWithObstacle()
    printMap()    
if __name__ == "__main__":
    
    try: 
        px = Picarx()
        # turnLeft()
        # turn180()
        # turnRight()
        # main()
        navigateToGoal((0, 5))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally: 
        px.stop()
 
