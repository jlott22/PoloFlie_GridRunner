import time 
import sys
import math
from machine import Pin, UART
from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions

#flash lights to indicate entering program
rgb_leds = robot.RGBLEDs()
rgb_leds.set_brightness(5)

for _ in range(5):
    # Turn all LEDs green
    for led in range(6):
        rgb_leds.set(led, [0, 200, 0])
    rgb_leds.show()
    time.sleep_ms(100)

    # Turn all LEDs off
    for led in range(6):
        rgb_leds.set(led, [0, 0, 0])
    rgb_leds.show()
    time.sleep_ms(100)


'''
need to add jewel code after its all working!!!!
'''

# UART0 for ESP32 communication (TX=GP28, RX=GP29)
uart = UART(0, baudrate=230400, tx=28, rx=29)

# Initialize display and robot components
display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
bump_sensors = robot.BumpSensors()

# Direction ordering: N=0, E=1, S=2, W=3 for cyclic rotations
direction_order = ['N', 'E', 'S', 'W']
curr_dir_idx = 0  # Start facing North

# Robot's current grid coordinates (in cells)
x = 1
y = 0

# Physical grid cell size (meters)
cell_size = 0.25

# Movement control globals
movement_target = 0       # Number of intersections to traverse
intersections_passed = 0
stop_pending = False

# --- Publish message ---
def publish_msg(topic,msg):
    '''
    takes the topic to be published to and the message to be published
    both strings
    '''
    uart.write(str(topic) + '=' + str(msg) + '-')

# --- Sensor Calibration ---
def calibrate_sensors():
# moving message sent in go_to call
    motors.set_speeds(920, -920)
    for _ in range(50):
        line_sensors.calibrate()
        time.sleep(0.01)
    motors.set_speeds(0, 0)
    bump_sensors.calibrate()
    time.sleep(0.5)
    go_to(x*cell_size,y + cell_size)
    publish_msg('coord',f"{x*cell_size}/{y*cell_size}")
    time.sleep(0.5)
    publish_msg('status','stopped')

# --- Position Update ---
def update_location():
    global x, y, curr_dir_idx, intersections_passed
    facing = direction_order[curr_dir_idx]
    if facing == 'N':
        y += intersections_passed
    elif facing == 'S':
        y -= intersections_passed
    elif facing == 'E':
        x += intersections_passed
    elif facing == 'W':
        x -= intersections_passed
    intersections_passed = 0

# --- Line Following & Intersection Counting ---
def ride_the_line():
    global intersections_passed, movement_target, stop_pending
    last_p = 0
    ignore_timer = time.time()

    #only runs if intersection goal not reached
    while intersections_passed < movement_target:
        # Check for stop command
        if uart.any():
            cmd = uart.readline().decode('utf-8').strip()
            if cmd == 'stop':
                stop_pending = True

        # Read sensors
        line = line_sensors.read_calibrated()
        line_sensors.start_read()
        if sum(line) == 0:
            continue

        # Proportional line-following
        l = (0*line[0] + 1000*line[1] + 2000*line[2] + 3000*line[3] + 4000*line[4]) // sum(line)
        p = l - 2000
        d = p - last_p
        last_p = p
        pid = 0 if abs(p) < 50 else p*90 + d*300
        smoothed = 0.7 * last_p + 0.3 * pid
        left_spd = max(300, min(1000, 900 + smoothed))
        right_spd = max(300, min(1000, 900 - smoothed))
        motors.set_speeds(left_spd, right_spd)

        # Reset ignore after 1s
        if time.time() - ignore_timer >= 1:
            ignore_timer = 0

        # Intersection detection
        if ignore_timer == 0 and (line[0] > 500 or line[4] > 500):
            motors.set_speeds(400, 400)
            time.sleep(0.1)
            if line[0] > 500 or line[4] > 500:
                intersections_passed += 1
            ignore_timer = time.time()
            motors.set_speeds(900, 900)

        # Stop if needed
        if stop_pending or intersections_passed >= movement_target:
            motors.set_speeds(0, 0)
            update_location()
            stop_pending = False
            return

        # Bump detection
        bump_sensors.read()
        if bump_sensors.left_is_pressed() or bump_sensors.right_is_pressed():
            motors.set_speeds(0, 0)
            publish_msg('alert',f"bingo:{x*cell_size}/{y*cell_size}")
            return

# --- Rotation & Movement Commands ---
def rotate_to(target_idx):
    global curr_dir_idx
    diff = (target_idx - curr_dir_idx) % 4
    if diff == 1:
        motors.set_speeds(1200, -300)
        time.sleep(0.4)
    elif diff == 2:
        motors.set_speeds(1200, -300)
        time.sleep(0.4)
        motors.set_speeds(1200, -300)
        time.sleep(0.4)
    elif diff == 3:
        motors.set_speeds(-300, 1200)
        time.sleep(0.4)
    motors.set_speeds(0, 0)
    curr_dir_idx = target_idx


def move_cells(n):
    global movement_target
    movement_target = n
    ride_the_line()

# --- Navigate to Meter Coordinates ---
def go_to(x_m, y_m):
    publish_msg('status','moving')
    target_x = int(round(x_m / cell_size))
    target_y = int(round(y_m / cell_size))
    dx_cells = target_x - x
    dy_cells = target_y - y

    # Move in Y
    if dy_cells != 0:
        dir_idx = direction_order.index('N') if dy_cells > 0 else direction_order.index('S')
        rotate_to(dir_idx)
        move_cells(abs(dy_cells))
        
    # Move in X
    if dx_cells != 0:
        dir_idx = direction_order.index('E') if dx_cells > 0 else direction_order.index('W')
        rotate_to(dir_idx)
        move_cells(abs(dx_cells))

# --- Main Loop ---
while True:
    if uart.any():
        msg = uart.readline().decode('utf-8').strip()
        if msg == 'calibrate':
            calibrate_sensors()
        elif msg == 'exit':
            sys.exit()
        else:
            try:
                x_t, y_t = map(float, msg.split('/'))
                go_to(x_t, y_t)
                publish_msg('coord',f"{x*cell_size}/{y*cell_size}")
                time.sleep(0.5)
                publish_msg('status','stopped')
            except ValueError:
                continue
            

