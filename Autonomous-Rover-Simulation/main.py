from robot import Robot
from sensors.lidar import LidarScan
from utils.geometry import normalize_angle
import matplotlib.pyplot as plt
import math
import csv

# -----------------------------------------------------
# Runtime modes & visualization toggles
# -----------------------------------------------------
MODE = "MANUAL"        # MANUAL | AUTO
SHOW_LIDAR = True
SHOW_ODOM = True


# -----------------------------------------------------
# Control commands (shared state)
# -----------------------------------------------------
v = 0.0   # linear velocity [m/s]
w = 0.0   # angular velocity [rad/s]
ld = 1.0
i = 0
def on_key(event):
    """Keyboard control & visualization toggles."""
    global v, w, MODE, SHOW_LIDAR, SHOW_ODOM

    # --- visualization toggles ---
    if event.key == 'o':
        SHOW_ODOM = not SHOW_ODOM
        print(f"Odometry visualization: {'ON' if SHOW_ODOM else 'OFF'}")
        return

    if event.key == 'l':
        SHOW_LIDAR = not SHOW_LIDAR
        print(f"LiDAR visualization: {'ON' if SHOW_LIDAR else 'OFF'}")
        return

    # --- mode switching ---
    if event.key == 'm':
        MODE = "MANUAL"
        v = 0.0
        w = 0.0
        print("Switched to MANUAL mode")
        return

    if event.key == 'a':
        MODE = "AUTO"
        print("Switched to AUTO mode")
        return

    # --- manual control ---
    if MODE != "MANUAL":
        return

    if event.key == 'up':
        v += 1.5
    elif event.key == 'down':
        v -= 1.5
    elif event.key == 'left':
        w += 2.0
    elif event.key == 'right':
        w -= 2.0
    elif event.key == ' ':
        v = 0.0
        w = 0.0

    # clamp commands
    v = max(min(v, 6.0), -6.0)
    w = max(min(w, 6.0), -6.0)


if __name__ == "__main__":

    lidar = LidarScan(max_range=4.0)
    robot = Robot()

    plt.close('all')
    fig = plt.figure(num=2)
    fig.canvas.manager.set_window_title("Autonomy Debug View")
    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.show(block=False)

    dt = 0.01 

    # -------------------------------------------------
    # Main simulation loop
    # -------------------------------------------------
    while plt.fignum_exists(fig.number):

        # ground truth pose
        real_x, real_y, real_theta = robot.get_ground_truth()
        # odometry estimate
        ideal_x, ideal_y, ideal_theta = robot.get_odometry()
        # LiDAR scan 
        lidar_ranges, lidar_points, lidar_rays, lidar_hits = lidar.get_scan((real_x, real_y, real_theta))

        if MODE == "AUTO":

            # ---------------------------------------------
            # write your autonomous code here!!!!!!!!!!!!!
            # ---------------------------------------------
            path_points = []
            try:
                with open('path.csv', mode='r') as file:
                    reader = csv.reader(file)
                    next(reader)
                    for row in reader:
                        path_points.append((float(row[0]), float(row[1])))
            except FileNotFoundError:
                print(f"Error: {'path.csv'} not found in project root.")
            
            while i < len(path_points):
                d = math.dist(path_points[i],[ideal_x,ideal_y])
                i+=1
                if d >= ld:
                    alpha = normalize_angle(math.atan2(path_points[i][1]-ideal_y, path_points[i][0]-ideal_x)-ideal_theta)
                    break
            v = 10.0        
            k = 2*math.sin(alpha)/(d**2)
            if i >= len(path_points):
                v = 0.0
            
            
                



            # ---------------------------------------------
            # Allowed inputs:
            #   - real_x, real_y, real_theta
            #   - ideal_x, ideal_y, ideal_theta (odometry you have to use for logic)
            #   - lidar_ranges (lidar data you have to use for logic, array of length 36 corresponding to 36 beams)
            #
            # Required outputs:
            #   - v, w (linear and angular velocity commands)


            
            w = v*k

            # ---------------------------------------------
            # don't edit below this line (visualization & robot stepping)
            # ---------------------------------------------
        robot.step(
            lidar_points,
            lidar_rays,
            lidar_hits,
            v,
            w,
            dt,
            show_lidar=SHOW_LIDAR,
            show_odom=SHOW_ODOM
        )

        plt.pause(dt)
