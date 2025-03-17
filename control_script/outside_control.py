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


    def geofence_detected(self):
        pass

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

                    if self.geofence_detected() == True:
                        angles = self.angles
                        random.shuffle(angles)
                        imu_reading = (self.get_yaw()) % 360

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