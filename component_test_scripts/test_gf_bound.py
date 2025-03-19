from pymavlink import mavutil
import math
import numpy as np


def get_yaw():
    imu_active = True
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    while imu_active == True:
        if msg:
            yaw_deg = msg.yaw * 180 / np.pi
            if yaw_deg < 0:
                yaw_deg = yaw_deg + 360
            imu_active = False  
            return yaw_deg
        else:
            print("No attitude data received within timeout. Continuing!!!")
            continue



def get_relative_pos(lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2/1e7 - lon1/1e7)
        lat1 = math.radians(lat1/1e7)
        lat2 = math.radians(lat2/1e7)
        
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
        return math.degrees(math.atan2(y, x)) % 360


def calculate_bearing(geofence_bearing, vehicle_heading):
    """Determine front/front-right/front-left position"""
    diff = (geofence_bearing - vehicle_heading) % 360
    if diff <= 45 or diff >= 315:
        return "Front"
    elif 22.5 < diff <= 90:
        return "Front-Right"
    elif 270 < diff < 337.5:
        return "Front-Left"
    elif 45 < diff < 135:
        return "Right"
    elif 225 < diff < 315:
        return "Left"
    else:
        return "Other"
    
def geofence_detected(vertices):
    def distance_to_line_segment(p1, p2, p):
        """Calculate distance from point p to line segment p1-p2"""
        x0, y0 = p[0]/1e7, p[1]/1e7
        x1, y1 = p1[0]/1e7, p1[1]/1e7
        x2, y2 = p2[0]/1e7, p2[1]/1e7
        lat_dist = 111319  # meters per degree latitude (or 111111 meter)
        lon_dist = lat_dist * math.cos(math.radians(y0))
        dx = (x2 - x1) * lon_dist
        dy = (y2 - y1) * lat_dist
        l2 = dx**2 + dy**2
        if l2 == 0:
            return math.hypot((x0 - x1) * lon_dist, (y0 - y1) * lat_dist)
        t = max(0, min(1, ((x0 - x1) * dx + (y0 - y1) * dy) / l2))
        nearest_x = x1 + t * dx / lon_dist
        nearest_y = y1 + t * dy / lat_dist
        return math.hypot((x0 - nearest_x) * lon_dist, (y0 - nearest_y) * lat_dist)
    pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if pos:
        current_lat = pos.lat
        current_lon = pos.lon
        heading = get_yaw() % 360
        # Calculate distance to each edge
        distances = []
        for i in range(len(vertices)):
            p1 = vertices[i]
            p2 = vertices[(i+1) % len(vertices)]
            distances.append(distance_to_line_segment(p1, p2, (current_lat, current_lon)))
        min_distance = min(distances)
        if min_distance <= distance_thr:
            closest_edge_idx = distances.index(min_distance)
            p1 = vertices[closest_edge_idx]
            p2 = vertices[(closest_edge_idx+1) % len(vertices)]
            # Calculate bearing to closest point on edge
            midpoint_lat = (p1[0] + p2[0]) / 2
            midpoint_lon = (p1[1] + p2[1]) / 2
            fence_bearing = calculate_bearing(current_lat, current_lon, midpoint_lat, midpoint_lon)
            direction = get_relative_pos(heading, fence_bearing)
            if direction != "Other":
                print(f"Geofence approaching from {direction} ({min_distance:.1f}m)")
                return direction, min_distance


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



distance_thr = 1
baud_rate = 115200
pixhawk_port = "/dev/ttyACM0"
master = mavutil.mavlink_connection(pixhawk_port, baud=baud_rate)
master.wait_heartbeat()
print("Heartbeat recieved from the system!!")
print(f"Connected to pixhawk at {pixhawk_port}")

corners = calculate_geofence(half_diagonal_km=0.01, aspect_ratio=1)
vertices = [
            corners['top_left'],
            corners['top_right'],
            corners['bottom_right'],
            corners['bottom_left']
            ]

print(geofence_detected(vertices))