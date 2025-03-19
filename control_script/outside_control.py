from pymavlink import mavutil
import numpy as np
import time
import serial
import datetime
import logging
import csv
import math
import random



class OutsideControl:
    def __init__(self, master, ser):
        self.master = master
        self.ser = ser

        self.time_horizon = 3600 # in seconds
        self.distance_thr = 1 # in meter
        self.rc_channel_values = [65535 for _ in range(18)]
        self.angles = [15, 30, 45, 60, 75, 90, 170]


    def go_forward(self):
        run_motor = True
        channel = 3
        pwm = 1400
        while run_motor == True:
            self.rc_channel_values[channel-1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system, #target_system
                self.master.target_component, # target_component
                *self.rc_channel_values # RC channel list, in microseconds
            )
        if self.geofence_detected() == True:
            run_motor = False
        return


    def turn_rotate(self, channel, pwm, endpoint):
        run_motor = True
        while run_motor:
            self.rc_channel_values[channel-1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system, #target_system
                self.master.target_component, # target_component
                *self.rc_channel_values # RC channel list, in microseconds
            )
            reading = (self.get_yaw()) % 360
            if 0 <= abs(reading - endpoint) <= 10:
                run_motor = False
        return

    
    def log_GPS(self, log_file_name):
        logging.basicConfig(level=logging.INFO)

        try:
            with open(log_file_name, 'a', newline='') as csv_file:
                fieldnames = ['Timestamp', 'Latitude', 'Longitude', 'Altitude']
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

                if csv_file.tell() == 0:
                    writer.writeheader()

                while True:
                    msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True)
                    if msg is not None:
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        alt = msg.alt
                        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        writer.writerow({
                            'Timestamp': timestamp,
                            'Latitude': lat,
                            'Longitude': lon,
                            'Altitude': alt
                        })
                        logging.info(f"Logged GPS data: Lat={lat}, Lon={lon}, Alt={alt}")
                    else:
                        logging.warning("No GPS message received.")
        except Exception as e:
            logging.error(f"Error logging GPS data: {e}")
        


    def calculate_geofence(self, half_diagonal_km, aspect_ratio=1.0):
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
            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True)
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


    def get_relative_pos(self, lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2/1e7 - lon1/1e7)
        lat1 = math.radians(lat1/1e7)
        lat2 = math.radians(lat2/1e7)
        
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
        return math.degrees(math.atan2(y, x)) % 360


    def calculate_bearing(self, geofence_bearing, vehicle_heading):
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


    def geofence_detected(self, vertices):

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


        pos = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

        if pos:
            current_lat = pos.lat
            current_lon = pos.lon
            heading = self.get_yaw() % 360

            # Calculate distance to each edge
            distances = []
            for i in range(len(vertices)):
                p1 = vertices[i]
                p2 = vertices[(i+1) % len(vertices)]
                distances.append(distance_to_line_segment(p1, p2, (current_lat, current_lon)))

            min_distance = min(distances)

            if min_distance <= self.distance_thr:
                closest_edge_idx = distances.index(min_distance)
                p1 = vertices[closest_edge_idx]
                p2 = vertices[(closest_edge_idx+1) % len(vertices)]

                # Calculate bearing to closest point on edge
                midpoint_lat = (p1[0] + p2[0]) / 2
                midpoint_lon = (p1[1] + p2[1]) / 2
                fence_bearing = self.calculate_bearing(current_lat, current_lon, midpoint_lat, midpoint_lon)

                direction = self.get_relative_pos(heading, fence_bearing)
                if direction != "Other":
                    print(f"Geofence approaching from {direction} ({min_distance:.1f}m)")
                    return direction, min_distance



    def get_yaw(self):
        imu_active = True
        msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
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

    
    def sign(self, direction):
        if direction == 'Front-Right':
            return -1

        elif direction == 'Front-Left':
            return 1
        
        elif direction == 'Front':
            return 1
        
        elif direction == 'Right':
            return -1
        
        elif direction == 'Left':
            return 1
        
        else: 
            return 1
        
    
    def rotate(self, theta):
        turn = None
        if theta > 0:
            turn = "anti-clock"
        else:
            turn = "clock"
        
        if turn == 'clock':
            START_THETA = (self.get_yaw()) % 360
            endpointRIGHT = (START_THETA + theta) % 360
            endpointLEFT = (START_THETA - theta) % 360

            print(f"Rotating the vehicle in {turn} direction")

            # reading = (self.get_yaw() * (180/np.pi)) % 360
            # print(f"(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            # print(f' [turning right {theta} degrees]')

            reading = (self.get_yaw()) % 360
            print(f"(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            print(f' [turning right {theta} degrees]')

            rotate_condition = 0 <= abs(reading - endpointRIGHT) <= 10
            channel = 1
            pwm = 1700
            
            self.turn_rotate(channel, pwm, endpointRIGHT)

            # rotate = True
            # while rotate:
            #     print("Turning right!!")
            #     self.turn_rotate(channel, pwm, endpointRIGHT)
            #     if rotate_condition == True:
            #         print("Turning right done!!")
            #         rotate = False
            #         self.initial()
            #     else:
            #         continue

            # rotating = True
            # while rotating == True:
            #     reading = (self.get_yaw() * (180/np.pi)) % 360
            #     print(f"(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            #     print(f' [turning right {theta} degrees]')
            #     rotate_condition = 0 <= abs(reading - endpointRIGHT) <= 10
            #     self.turn_rotate(channel, pwm, endpointRIGHT)
            #     if rotate_condition == True:
            #         rotating = False


            # if rotate_condition == True:
            #     print("Turning right complete!")
            #     self.initial()
            # elif rotate_condition == False:
            #     print("Turning Right!!")
            #     self.turn_rotate(channel, pwm, endpointRIGHT)

                # run_motor = True
                # while run_motor:
                #     self.rc_channel_values[channel-1] = pwm
                #     self.master.mav.rc_channels_override_send(
                #         self.master.target_system, #target_system
                #         self.master.target_component, # target_component
                #         *self.rc_channel_values # RC channel list, in microseconds
                #     )
                #     reading = (self.get_yaw() * (180/np.pi)) % 360
                #     print("Reading: ", reading, " EndpointRight: ", endpointRIGHT)
                #     if 0 <= abs(reading - endpointRIGHT) <= 10:
                #         run_motor = False
        if turn == 'anti-clock':
            START_THETA = (self.get_yaw()) % 360
            endpointRIGHT = (START_THETA + theta) % 360
            endpointLEFT = (START_THETA - theta) % 360

            print(f"Rotating the vehicle in {turn} direction")

            reading = (self.get_yaw()) % 360
            print(f"(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            print(f' [turning left {theta} degrees]')

            rotate_condition = 0 <= abs(reading - endpointLEFT) <= 10
            channel = 1
            pwm = 1200

            self.turn_rotate(channel, pwm, endpointLEFT)
            
            # rotate = True
            # while rotate:
            #     print("Turning left!!")
            #     self.turn_rotate(channel, pwm, endpointLEFT)
            #     if rotate_condition == True:
            #         print("Turning left done!!")
            #         rotate = False
            #         self.initial()
            #     else:
            #         continue

            # if rotate_condition == True:
            #     print("Turning left complete!")
            #     self.initial()
            # elif rotate_condition == False:
            #     print("Turning Left!!")
            #     self.turn_rotate(channel, pwm, endpointLEFT)

                # run_motor = True
                # while run_motor:
                #     self.rc_channel_values[channel-1] = pwm
                #     self.master.mav.rc_channels_override_send(
                #         self.master.target_system, #target_system
                #         self.master.target_component, # target_component
                #         *self.rc_channel_values # RC channel list, in microseconds
                #     )
                #     reading = (self.get_yaw() * (180/np.pi)) % 360
                #     print("Reading: ", reading, " EndpointLeft: ", endpointLEFT)
                #     if 0 <= abs(reading - endpointLEFT) <= 10:
                #         run_motor = False


    
    def start(self):
        log_data = True
        start_time = time.time()

        while log_data == True:
            while (time.time() - start_time) <= self.time_horizon:
                if (time.time() - start_time) <= 10:
                    run_motor = True
                    channel = 3
                    pwm = 1400
                    while run_motor == True:
                        self.rc_channel_values[channel-1] = pwm
                        self.master.mav.rc_channels_override_send(
                            self.master.target_system, #target_system
                            self.master.target_component, # target_component
                            *self.rc_channel_values # RC channel list, in microseconds
                        )
                        if (time.time() - start_time) > 10:
                            pwm = 1100
                            self.rc_channel_values[channel-1] = pwm
                            self.master.mav.rc_channels_override_send(
                            self.master.target_system, #target_system
                            self.master.target_component, # target_component
                            *self.rc_channel_values # RC channel list, in microseconds
                            )
                            run_motor = False

                elif (time.time() - start_time) > 10:

                    self.log_GPS(log_file_name='gps_data.csv')
                    corners = self.calculate_geofence(half_diagonal_km=0.05, aspect_ratio=1/2)
                    vertices = [
                                corners['top_left'],
                                corners['top_right'],
                                corners['bottom_right'],
                                corners['bottom_left']
                                ]
                    # Send vertices to autopilot
                    for idx, (lat, lon) in enumerate(vertices):
                        self.master.mav.command_long_send(
                            target_system=self.master.target_system,
                            target_component=self.master.target_component,
                            command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                            confirmation=0,
                            param1=idx+1,  # Vertex index (1-based)
                            param2=lat,
                            param3=lon,
                            param4=0,      # Altitude (not used)
                        )

                    # Finalize geofence (4 vertices)
                    self.master.mav.command_long_send(
                        target_system=self.master.target_system,
                        target_component=self.master.target_component,
                        command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                        confirmation=0,
                        param1=4,  # Total vertices
                    )

                    direction, min_distance = self.geofence_detected(vertices)
                    if min_distance <= self.distance_thr:
                        angles = self.angles
                        random.shuffle(angles)
                        for angle in angles:
                            sign = self.sign(direction)
                            self.rotate(sign*angle)
                        

                    else:
                        self.go_forward()

                

def main():
    baud_rate = 115200
    pixhawk_port = "/dev/ttyACM0"
    master = mavutil.mavlink_connection(pixhawk_port, baud=baud_rate)
    master.wait_heartbeat()
    print("Heartbeat recieved from the system!!")
    print(f"Connected to pixhawk at {pixhawk_port}")

    arduino_port = "/dev/ttyACM1"
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)
    print(f"Connected to Arduino Mega at {arduino_port}")

    print("All connections established!")

    control = OutsideControl(master, ser)
    control.start()

if __name__=="__main__":
    main()