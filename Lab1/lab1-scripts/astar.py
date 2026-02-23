from picarx import Picarx
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib import colors
import math
import heapq

#Global Variable
GRID_SIZE = 15                        # in cms
OBSTACLE_PADDING = 0                    # Padding (in cm) used to mark on all sides of obstacles.
CAR_LOCATION = (7, 0)             # Car position (x,y). bottom center of the grid
CAR_DIMENSIONS = (1, 1)
DIRECTION = (0, 1)        # Car direction as in (x, y).
MAP = np.zeros((GRID_SIZE, GRID_SIZE)     # Initialize empty map
                , dtype=np.int32)


# Global Goal Variable
GOAL_LOCATION = (7, 10)  # Set your target (x, y) here

def heuristic(a, b):
    # Manhattan distance for grid movement
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, grid):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)] # Up, Down, Right, Left
    
    close_set = set()
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (f_score[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1] # Path from start to goal

        close_set.add(current)
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Boundary and Obstacle Check
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE:
                if grid[neighbor[0], neighbor[1]] == 1: # Skip obstacles
                    continue
            else:
                continue

            tentative_g_score = g_score[current] + 1
            
            if neighbor in close_set and tentative_g_score >= g_score.get(neighbor, 0):
                continue
                
            if tentative_g_score < g_score.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (f_score[neighbor], neighbor))
                
    return None # No path found

def scan_and_is_cell_free(target_cell):
    """
    Performs a scan update, then returns True if the target cell is *not* an obstacle.
    """
    global MAP, GRID_SIZE

    updateMapWithObstacle()

    x, y = target_cell
    if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
        # Treat out-of-bounds as blocked for safety
        return False

    # Free if not hard obstacle (1). Cells with 0 or 2 (car) are traversable.
    return MAP[x, y] != 1


def move_to_next_cell(target_cell):
    """
    Turns to face the target adjacent cell, scans to verify it is free,
    and only then moves one step. Returns True if moved, False if blocked.
    """
    global CAR_LOCATION, DIRECTION

    # Vector toward the target
    target_vector = (target_cell[0] - CAR_LOCATION[0], target_cell[1] - CAR_LOCATION[1])

    # Sanity: the target must be one of the 4-neighborhood cells
    if target_vector not in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        raise ValueError(f"Target cell {target_cell} is not adjacent to {CAR_LOCATION}")

    # If not already facing the target, rotate accordingly (do not move yet)
    if target_vector != DIRECTION:
        # 2D cross/dot for 90°/180° decisions
        turn_val = DIRECTION[0] * target_vector[1] - DIRECTION[1] * target_vector[0]
        dot_val = DIRECTION[0] * target_vector[0] + DIRECTION[1] * target_vector[1]

        if dot_val == -1:
            turn180()
        elif turn_val > 0:
            turnLeft()
        elif turn_val < 0:
            turnRight()
        # If dot_val==1 we are already aligned (handled by the if-guard above)

    # ---------- NEW: Scan *after* alignment, before moving ----------
    if not scan_and_is_cell_free(target_cell):
        print(f"Blocked ahead at {target_cell}; re-planning without moving.")
        return False

    # Safe to proceed
    forwardOneStep()
    return True

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
        if distance < 40: # If an obstacle is detected within 50 cm
            # Calculate the obstacle's position based on the car's current location and direction
            obs_x = CAR_LOCATION[0] + (int(distance * math.cos(math.radians(angle)+car_heading_rad)) // 20)
            obs_y = CAR_LOCATION[1] + (int(distance * math.sin(math.radians(angle)+car_heading_rad)) // 20)
            #print((obs_x,obs_y), (CAR_LOCATION[0],CAR_LOCATION[1]))
            
            if 0 <= obs_x < GRID_SIZE and 0 <= obs_y < GRID_SIZE:
                MAP[obs_x, obs_y] = 1

            #for dx in range(-OBSTACLE_PADDING, OBSTACLE_PADDING+1):    #interpolation - adding padding between the points
            #    for dy in range(-OBSTACLE_PADDING, OBSTACLE_PADDING + 1):
            #        if 0 <= obs_x + dx < GRID_SIZE and 0 <= obs_y + dy < GRID_SIZE:
            #            MAP[obs_x + dx, obs_y + dy] = 1
    px.set_cam_pan_angle(0)
    printMap()
def updateMapwithCar():
    global DIRECTION
    global CAR_LOCATION
    global MAP
    global GRID_SIZE
    px.stop()
    MAP[CAR_LOCATION[0] , CAR_LOCATION[1]]=2
    #for dx in range(-CAR_DIMENSIONS[0]//2, CAR_DIMENSIONS[0]//2 + 1):
    #    for dy in range(-CAR_DIMENSIONS[1]//2, CAR_DIMENSIONS[1]//2 + 1):
    #        if 0 <= CAR_LOCATION[0] + dx < GRID_SIZE and 0 <= CAR_LOCATION[1] + dy < GRID_SIZE:
    #            MAP[CAR_LOCATION[0] + dx, CAR_LOCATION[1] + dy] = 2
def turnLeft():
    global DIRECTION
    global CAR_DIMENSIONS
    global CAR_LOCATION
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
    time.sleep(0.6)
    (i, j) = CAR_LOCATION
    (x,y) = DIRECTION
    (a, b) = CAR_DIMENSIONS
    #if (x, y) == (1,0):
    #    CAR_LOCATION = (i - b//4, j + a // 4)
    #if (x, y) == (0,1):
    #    CAR_LOCATION = (i - b//4, j - a // 4)
    #if (x, y) == (-1,0):
    #    CAR_LOCATION = (i + b//4, j - a // 4)
    #if (x, y) == (0,-1):
    #    CAR_LOCATION = (i + b//4, j + a // 4)
    DIRECTION = (-y,x)
    CAR_DIMENSIONS = (b, a)
    updateMapwithCar()
def turnRight():
    global DIRECTION
    global CAR_DIMENSIONS
    global CAR_LOCATION
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
    (i, j) = CAR_LOCATION
    (x, y) = DIRECTION
    (a, b) = CAR_DIMENSIONS
    #if (x, y) == (1,0):
    #    CAR_LOCATION = (i - b//4, j - a // 4)
    #if (x, y) == (0,1):
    #    CAR_LOCATION = (i + b//4, j - a // 4)
    #if (x, y) == (-1,0):
    #    CAR_LOCATION = (i + b//4, j + a // 4)
    #if (x, y) == (0,-1):
    #    CAR_LOCATION = (i - b//4, j - a // 4)
    DIRECTION = (y, -x)
    CAR_DIMENSIONS = (b, a)
    updateMapwithCar()
def turn180():
    turnRight()
    turnRight()
def forwardOneStep():
    global MAP
    global CAR_LOCATION
    global DIRECTION
    px.forward(20)
    time.sleep(.8)
    px.set_dir_servo_angle(7)
    px.forward(20)
    time.sleep(.8)
    px.set_dir_servo_angle(0)
    CAR_LOCATION = (CAR_LOCATION[0] + DIRECTION[0], CAR_LOCATION[1] + DIRECTION[1])
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
    CAR_LOCATION = (CAR_LOCATION[0] - DIRECTION[0], CAR_LOCATION[1] - DIRECTION[1])
    MAP[CAR_LOCATION[0], CAR_LOCATION[1]] = 2
    updateMapwithCar()

def main():
    global CAR_LOCATION, GOAL_LOCATION, MAP
    print(f"Starting Navigation from {CAR_LOCATION} to {GOAL_LOCATION}")
    initMap()
    updateMapWithObstacle()
    while CAR_LOCATION != GOAL_LOCATION:        

        # 2) PLAN: Run A* to find current best path
        path = a_star(CAR_LOCATION, GOAL_LOCATION, MAP)

        if path:
            next_step = path[0]
            print(f"Path found. Next step: {next_step}")

            # 3) ACT: Try to move to the very next cell (scan-before-move inside)
            moved = move_to_next_cell(next_step)

            if not moved:
                # If blocked, do not move; the map was just updated by the scan,
                # so the next loop iteration will replan with the new obstacle.
                pass
        else:
            print("Target unreachable! Searching for new path...")
            # If still no path, rotate to expand field of view and try again
            turnLeft()
            updateMapWithObstacle()
            time.sleep(1)

        printMap()  # Optional visualization each step

    print("Goal Reached!")

    
if __name__ == "__main__":
    
    try: 
        px = Picarx()
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("ERROR: {e}")
    finally: 
        px.stop()
 
