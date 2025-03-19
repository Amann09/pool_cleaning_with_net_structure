from pymavlink import mavutil
import math



def calculate_geofence(half_diagonal_km, aspect_ratio=1.0):
    """
    Calculate geofence corners using half-diagonal length and aspect ratio
    Parameters:
    - half_diagonal_km: Half of rectangle's diagonal (km)
    - aspect_ratio: Width/length ratio (default 1.0 for square)
    """
    diagonal = 2 * half_diagonal_km
    length = diagonal / math.sqrt(1 + aspect_ratio**2)
    width = aspect_ratio * length
    got_gps = False
    while got_gps==False:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg is not None:
            center_lat = msg.lat / 1e7
            center_lon = msg.lon / 1e7
            print(f"Latitude: {center_lat}, Longitude: {center_lon}")
            got_gps=True
        else:
            print("Calculating again!!")
            continue
    # Convert to angular offsets
    R = 6371.0  # Earth radius in km
    lat_offset = math.degrees(length/2 / R)
    lon_offset = math.degrees(width/2 / (R * math.cos(math.radians(center_lat))))
    return {
        'top_left': (center_lat + lat_offset, center_lon - lon_offset),
        'top_right': (center_lat + lat_offset, center_lon + lon_offset),
        'bottom_left': (center_lat - lat_offset, center_lon - lon_offset),
        'bottom_right': (center_lat - lat_offset, center_lon + lon_offset)
    }

baud_rate = 115200
pixhawk_port = "/dev/ttyACM0"
master = mavutil.mavlink_connection(pixhawk_port, baud=baud_rate)
master.wait_heartbeat()
print("Heartbeat recieved from the system!!")
print(f"Connected to pixhawk at {pixhawk_port}")
print("Geofence corners: ", calculate_geofence(half_diagonal_km=0.01, aspect_ratio=1.0))