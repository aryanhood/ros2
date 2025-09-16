import math
import time
import statistics

# -----------------------
# CONFIG
# -----------------------
DT = 0.05              # control loop interval (s)
V_MAX = 0.6            # max linear speed (m/s)
W_MAX = 2.5            # max angular speed (rad/s)

DIST_TOL = 0.001       # final position tolerance (m)
ANG_TOL  = 0.001       # final angle tolerance (rad)

# PD gains (continuous-time ideal; tuned earlier for near-critical damping)
Kp_linear  = 1.3
Kd_linear  = 2.28
Kp_angular = 4.8
Kd_angular = 4.38

# Square settings
SIDE = 2.0             # side length (m)
START = (0.0, 0.0, 0.0)  # x, y, theta

# Calibration settings
CALIB_DISTANCE = 0.6   # distance to move in calibration step (m)
N_CALIB_RUNS = 4       # number of calibration runs to average overshoot

# -----------------------
# UTILS
# -----------------------
def normalize_angle(a):
    while a > math.pi:
        a -= 2*math.pi
    while a <= -math.pi:
        a += 2*math.pi
    return a

# -----------------------
# SIMULATED ROBOT STATE
# (If you're running on a real robot, replace the state update
# with real odometry and actuators. This script uses the unicycle
# kinematic update to be self-contained.)
# -----------------------
x = START[0]
y = START[1]
theta = START[2]

def step_state(v, w, dt=DT):
    global x, y, theta
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += w * dt
    theta = normalize_angle(theta)

# -----------------------
# PD CONTROLLERS
# -----------------------
prev_dist_err = 0.0
prev_ang_err  = 0.0

def pd_linear_control(dist_err):
    global prev_dist_err
    d = (dist_err - prev_dist_err) / DT
    u = Kp_linear * dist_err + Kd_linear * d
    prev_dist_err = dist_err
    return max(min(u, V_MAX), -V_MAX)

def pd_angular_control(ang_err):
    global prev_ang_err
    d = (ang_err - prev_ang_err) / DT
    u = Kp_angular * ang_err + Kd_angular * d
    prev_ang_err = ang_err
    return max(min(u, W_MAX), -W_MAX)

# -----------------------
# CALIBRATION: Measure overshoot when moving a step distance
# Approach:
# - For each run: rotate to heading, then move to target DIST.
# - Record maximum distance reached along approach direction after velocity crosses zero / command finishes.
# - Overshoot = max_distance_reached - commanded_distance
# - Average across runs.
# -----------------------
def calibrate_overshoot(calib_distance=CALIB_DISTANCE, runs=N_CALIB_RUNS):
    global x, y, theta, prev_dist_err, prev_ang_err
    overshoots = []
    print("=== CALIBRATION START ===")
    for r in range(runs):
        # reset robot to origin (repeatable)
        x0, y0, th0 = 0.0, 0.0, 0.0
        set_state(x0, y0, th0)
        # aim straight along +x (theta_goal = 0)
        target_x = x0 + calib_distance
        target_y = y0
        max_reached = -1e9
        prev_dist_err = 0.0
        prev_ang_err = 0.0
        reached_command_point = False
        loop_iter = 0
        while True:
            loop_iter += 1
            dx = target_x - x
            dy = target_y - y
            dist_err = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            ang_err = normalize_angle(angle_to_goal - theta)

            # rotate if big angular error
            if abs(ang_err) > ANG_TOL:
                v = 0.0
                w = pd_angular_control(ang_err)
            else:
                # move toward the commanded point (PD)
                v = pd_linear_control(dist_err)
                w = 0.0

            # update state
            step_state(v, w)

            # compute projection along approach direction (+x)
            traveled_x = x - x0
            if traveled_x > max_reached:
                max_reached = traveled_x

            # check if we've passed the commanded point and stabilized a bit
            if not reached_command_point and dist_err < 0.002:
                # mark that the controller reached near commanded point
                reached_command_point = True
                # continue a little longer to observe overshoot peak
                overshoot_counter = 0

            if reached_command_point:
                overshoot_counter += 1
                # run extra few steps to capture peak
                if overshoot_counter > int(0.5 / DT):  # half-second observation
                    break

            # safety break
            if loop_iter > 2000:
                break

            time.sleep(0)  # no real wait needed in simulation

        overshoot = max_reached - calib_distance
        overshoots.append(overshoot)
        print(f" Calib run {r+1}/{runs}: measured_overshoot = {overshoot:.6f} m")

    avg_ov = statistics.mean(overshoots)
    stdev = statistics.stdev(overshoots) if len(overshoots) > 1 else 0.0
    print(f"=== CALIB DONE: avg overshoot = {avg_ov:.6f} m (stdev {stdev:.6f}) ===\n")
    return avg_ov

# helper to set state (for calibration repeatability)
def set_state(nx, ny, nth):
    global x, y, theta, prev_dist_err, prev_ang_err
    x = nx; y = ny; theta = nth
    prev_dist_err = 0.0
    prev_ang_err  = 0.0

# -----------------------
# HIGH LEVEL: move_to with overshoot-compensation
# direction: approach from current pose to (tx,ty)
# overshoot_comp: positive number (m) along approach direction to subtract from commanded distance
# -----------------------
def move_to(tx, ty, overshoot_comp=0.0):
    global x, y, theta, prev_dist_err, prev_ang_err
    max_iters = 5000
    it = 0
    while it < max_iters:
        it += 1
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        ang_err = normalize_angle(angle_to_goal - theta)

        # stop condition (both position and final orientation small)
        if dist < DIST_TOL and abs(ang_err) < ANG_TOL:
            # final micro-correction: zero velocities for a few steps to settle
            for _ in range(3):
                step_state(0.0, 0.0)
            return True

        # Phase: rotate first until roughly aligned
        if abs(ang_err) > 3*ANG_TOL:
            v_cmd = 0.0
            w_cmd = pd_angular_control(ang_err)
        else:
            # we are aligned: move toward a compensated target distance
            # compute compensated target point along line (tx,ty) by subtracting overshoot_comp
            # projection of current to target distance:
            # commanded approach distance = dist - overshoot_comp (but not less than 0)
            adjusted_dist = max(dist - overshoot_comp, 0.0)
            # Use PD toward the *adjusted* target distance, implemented by scaling dist_err_for_pd
            # Equivalent: set a virtual target located at distance adjusted_dist from current along goal direction
            dist_err_for_pd = adjusted_dist
            v_cmd = pd_linear_control(dist_err_for_pd)
            w_cmd = 0.0

        step_state(v_cmd, w_cmd)
        time.sleep(0)  # no real wait needed in simulation

    return False

# -----------------------
# SQUARE TRAVERSE
# -----------------------
def traverse_square(side=SIDE, overshoot_comp=0.0):
    # keep track of corners (clockwise)
    corners = [
        (0.0, 0.0),
        (side, 0.0),
        (side, side),
        (0.0, side),
        (0.0, 0.0)
    ]
    # start from first corner
    set_state(corners[0][0], corners[0][1], 0.0)
    print("Starting square traverse...")
    for i in range(1, len(corners)):
        tx, ty = corners[i]
        print(f"-> Moving to corner {i}: ({tx:.3f}, {ty:.3f}) with overshoot_comp={overshoot_comp:.6f}")
        ok = move_to(tx, ty, overshoot_comp=overshoot_comp)
        if not ok:
            print(" Failed to reach corner in allotted iterations")
            return False
        # after reach, orient to face next corner (except final)
        if i < len(corners)-1:
            next_tx, next_ty = corners[i+1]
            # compute desired heading
            desired_heading = math.atan2(next_ty - ty, next_tx - tx)
            # rotate to it
            rotate_to(desired_heading)
    print("Square traverse complete.")
    return True

def rotate_to(target_theta):
    global x, y, theta, prev_dist_err, prev_ang_err
    prev_dist_err = 0.0
    prev_ang_err = 0.0
    it = 0
    while it < 1000:
        it += 1
        ang_err = normalize_angle(target_theta - theta)
        if abs(ang_err) < ANG_TOL:
            return True
        w = pd_angular_control(ang_err)
        step_state(0.0, w)
        time.sleep(0)
    return False

# -----------------------
# MAIN: calibrate, then traverse square using compensation
# -----------------------
if __name__ == "__main__":
    # 1) calibrate overshoot magnitude (measured in meters)
    avg_overshoot = calibrate_overshoot(calib_distance=CALIB_DISTANCE, runs=N_CALIB_RUNS)

    # 2) traverses square using that overshoot compensation
    #    We subtract the overshoot so the initial overshoot lands at the exact corner.
    comp = max(avg_overshoot, 0.0)  # safety: don't use negative
    # small safety margin (optional) to avoid steady-state bias; tune if required
    safety_margin = 0.0
    comp -= safety_margin

    print(f"Using overshoot compensation = {comp:.6f} m")
    success = traverse_square(side=SIDE, overshoot_comp=comp)

    # final status
    if success:
        print("SUCCESS: Square traced using overshoot compensation.")
    else:
        print("FAILED: adjust gains, dt, or compensation and retry.")