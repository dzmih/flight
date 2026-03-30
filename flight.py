import time
import math
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ======= НАЛАШТУВАННЯ =======
SITL_ADDRESS = 'udp:172.25.224.1:14551'

TAKEOFF_ALT = 200.0  # метри

TARGET_LAT = 50.443326
TARGET_LON = 30.448078

YAW_HOLD = None  # встановиться після зльоту

# RC значення
RC_MID     = 1500
RC_THR_MID = 1500
RC_THR_HOVER = 1510

KP = 0.0  # коефіцієнт пропорційності (відстань -> RC відхилення)
ARRIVE_DIST = 3.0  # метри — вважаємо що прилетіли

# ============================

def get_distance_m(lat1, lon1, lat2, lon2):
    R = 6371000
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_lat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def get_bearing(lat1, lon1, lat2, lon2):
    d_lon = math.radians(lon2 - lon1)
    x = math.sin(d_lon) * math.cos(math.radians(lat2))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(d_lon)
    return math.degrees(math.atan2(x, y)) % 360

def send_rc_override(vehicle, roll, pitch, throttle, yaw):
    vehicle.channels.overrides = {
        '1': roll,
        '2': pitch,
        '3': throttle,
        '4': yaw
    }

def arm_and_takeoff(vehicle, alt):
    print("Arming...")
    vehicle.mode = VehicleMode("STABILIZE")
    
    vehicle._master.mav.param_set_send(
        vehicle._master.target_system,
        vehicle._master.target_component,
        b'ARMING_CHECK',
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(2)
    vehicle.armed = True
    while not vehicle.armed:
        print("  Waiting for arm...")
        time.sleep(1)
    print("Armed!")

    print(f"Taking off to {alt}m...")
    
    # Агресивний зліт
    send_rc_override(vehicle, RC_MID, RC_MID, 1700, RC_MID)
    time.sleep(2)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"  Alt: {current_alt:.1f}m")
        
        if current_alt >= alt * 0.95:
            print("Target altitude reached")
            # Після "Target altitude reached"
            print("Rotating to face target...")
            # Просто фіксуємо поточний heading як YAW_HOLD
            global YAW_HOLD
            YAW_HOLD = RC_MID
            break
        
        # Динамічний throttle залежно від висоти
        if current_alt < alt * 0.5:
            thr = 1700
        elif current_alt < alt * 0.8:
            thr = 1620
        else:
            thr = 1560
            
        send_rc_override(vehicle, RC_MID, RC_MID, thr, RC_MID)
        time.sleep(0.3)

def fly_to_target(vehicle, target_lat, target_lon, hold_alt):
    global YAW_HOLD
    YAW_HOLD = RC_MID  # фіксований yaw

    print(f"Flying to {target_lat}, {target_lon}")

    while True:
        loc = vehicle.location.global_relative_frame
        curr_lat = loc.lat
        curr_lon = loc.lon
        curr_alt = loc.alt

        dist = get_distance_m(curr_lat, curr_lon, target_lat, target_lon)
        bearing = get_bearing(curr_lat, curr_lon, target_lat, target_lon)
        heading = vehicle.heading

        print(f"  Dist: {dist:.1f}m | Bearing: {bearing:.1f} | Heading: {heading} | Alt: {curr_alt:.1f}m")

        if dist < ARRIVE_DIST:
            print("Arrived at target!")
            break

        # Кут між heading дрона і напрямком до цілі
        angle = math.radians(bearing - heading)

        # Фіксований нахил для крейсерського польоту
        if dist > 50:
            tilt = 300  # було 150
        elif dist > 20:
            tilt = 100  # було 80
        else:
            tilt = 30

        north = math.cos(angle)
        east  = math.sin(angle)

        pitch_rc = int(RC_MID - north * tilt)
        roll_rc  = int(RC_MID + east  * tilt)

        pitch_rc = max(1200, min(1800, pitch_rc))
        roll_rc  = max(1200, min(1800, roll_rc))

        # Утримання висоти
        alt_err = hold_alt - curr_alt
        thr = int(1510 + alt_err * 5)  # було 1530 + alt_err * 3
        thr = max(1400, min(1650, thr))

        send_rc_override(vehicle, roll_rc, pitch_rc, thr, YAW_HOLD)
        time.sleep(0.2)

    send_rc_override(vehicle, RC_MID, RC_MID, 1530, YAW_HOLD)

def land(vehicle):
    print("Landing...")
    vehicle.parameters['LAND_SPEED'] = 150  # см/с, було ~50
    vehicle.mode = VehicleMode("LAND")
    while vehicle.location.global_relative_frame.alt > 0.5:
        print(f"  Alt: {vehicle.location.global_relative_frame.alt:.1f}m")
        time.sleep(1)
    print("Landed!")
    vehicle.channels.overrides = {}

# ======= MAIN =======
print("Connecting...")
vehicle = connect(SITL_ADDRESS, wait_ready=True)
print(f"Connected. Mode: {vehicle.mode.name}")

arm_and_takeoff(vehicle, TAKEOFF_ALT)
fly_to_target(vehicle, TARGET_LAT, TARGET_LON, TAKEOFF_ALT)
land(vehicle)

vehicle.close()
print("Done.")