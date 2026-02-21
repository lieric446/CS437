import time
import math
from picarx import Picarx


ROWS = 30          # grid height (cells)
COLS = 30          # grid width  (cells)
CELL_SIZE_CM = 10  # 1 cell = 10 cm

STEP_TIME = 0.1            # seconds per loop
FORWARD_PWM = 25           # 0..100 (not cm)
MIN_VALID_CM = 3
MAX_VALID_CM = 250         # ignore farther readings
heading_rad = math.radians(90)  # initial heading: facing "up" in our map frame
UNKNOWN = 0
FREE = 1
OBSTACLE = 2
ROBOT = 3

SCAN_START =-30
SCAN_END = 30
SCAN_STEP = 10

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def empty_grid(rows, cols):
    return [[UNKNOWN for _ in range(cols)] for _ in range(rows)]


def print_grid(grid):
    # Simple print: . unknown, space free, # obstacle, R robot
    chars = {UNKNOWN: ".", FREE: " ", OBSTACLE: "#", ROBOT: "R"}
    for r in range(len(grid)):
        line = "".join(chars.get(grid[r][c], "?") for c in range(len(grid[0])))
        print(line)
    print("-" * len(grid[0]))


def mark_robot(grid, r, c):
    # Clear old robot marks
    for rr in range(len(grid)):
        for cc in range(len(grid[0])):
            if grid[rr][cc] == ROBOT:
                grid[rr][cc] = FREE
    grid[r][c] = ROBOT


def mark_free_line(grid, r0, c0, r1, c1):
    """
    Mark FREE cells along a line (Bresenham) from (r0,c0) to (r1,c1),
    excluding the last cell (which may be obstacle).
    """
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1

    r, c = r0, c0
    if dc > dr:
        err = dc // 2
        while c != c1:
            # mark current cell free (don't overwrite obstacle)
            if grid[r][c] == UNKNOWN:
                grid[r][c] = FREE
            err -= dr
            if err < 0:
                r += sr
                err += dc
            c += sc
    else:
        err = dr // 2
        while r != r1:
            if grid[r][c] == UNKNOWN:
                grid[r][c] = FREE
            err -= dc
            if err < 0:
                c += sc
                err += dr
            r += sr


def cm_to_cell(x_cm, y_cm):
    col = int(x_cm / CELL_SIZE_CM)
    row_from_bottom = int(y_cm / CELL_SIZE_CM)
    row = (ROWS - 1) - row_from_bottom
    return row, col


def in_bounds(r, c):
    return 0 <= r < ROWS and 0 <= c < COLS


def main():
    px = Picarx()

    grid = empty_grid(ROWS, COLS)

    x_cm = (COLS * CELL_SIZE_CM) / 2.0
    y_cm = 0.0  # bottom = 0 cm, increasing upward

    # Heading: 90° means facing upward in our world (y+)
    heading_rad = math.radians(90)

    # Initial robot cell
    r, c = cm_to_cell(x_cm, y_cm)
    r = clamp(r, 0, ROWS - 1)
    c = clamp(c, 0, COLS - 1)
    grid[r][c] = FREE
    mark_robot(grid, r, c)

    print("Mapping started. Ctrl+C to stop.\n")

    try:
        while True:
            for angle in range(SCAN_START, SCAN_END + 1, SCAN_STEP):
                scan_heading_rad = heading_rad + math.radians(angle)
                px.set_cam_pan_angle(angle)  # Rotate servo to angle (if hardware supports)
                dist = px.ultrasonic.read()

                if dist is not None and MIN_VALID_CM <= dist <= MAX_VALID_CM:
                    # Obstacle point in world cm along heading
                    ox = x_cm + dist * math.cos(scan_heading_rad)
                    oy = y_cm + dist * math.sin(scan_heading_rad)

                    r0, c0 = cm_to_cell(x_cm, y_cm)
                    r1, c1 = cm_to_cell(ox, oy)

                    if in_bounds(r0, c0) and in_bounds(r1, c1):
                        # mark free cells from robot to obstacle (excluding obstacle cell)
                        mark_free_line(grid, r0, c0, r1, c1)

                        # mark obstacle cell
                        if grid[r1][c1] != ROBOT:
                            grid[r1][c1] = OBSTACLE

                # Always mark current robot cell as free
                r, c = cm_to_cell(x_cm, y_cm)
                if in_bounds(r, c):
                    if grid[r][c] == UNKNOWN:
                        grid[r][c] = FREE
                    mark_robot(grid, r, c)

                # 3) Print grid occasionally
                print_grid(grid)

                CM_PER_SEC = 15.0  # <-- calibrate this number!
                dx = (CM_PER_SEC * STEP_TIME) * math.cos(heading_rad)
                dy = (CM_PER_SEC * STEP_TIME) * math.sin(heading_rad)

                px.forward(FORWARD_PWM)
                time.sleep(STEP_TIME)
                px.stop()

                x_cm += dx
                y_cm += dy

                # clamp inside map bounds (world cm)
                x_cm = clamp(x_cm, 0.0, COLS * CELL_SIZE_CM - 0.001)
                y_cm = clamp(y_cm, 0.0, ROWS * CELL_SIZE_CM - 0.001)

                time.sleep(0.1)

    except KeyboardInterrupt:
        px.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()
