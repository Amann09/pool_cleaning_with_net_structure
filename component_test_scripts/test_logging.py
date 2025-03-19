import logging
import datetime
import csv
from pymavlink import mavutil


def log_GPS(log_file_name):
    logging.basicConfig(level=logging.INFO)
    try:
        with open(log_file_name, 'a', newline='') as csv_file:
            fieldnames = ['Timestamp', 'Latitude', 'Longitude', 'Altitude']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            if csv_file.tell() == 0:
                writer.writeheader()
            while True:
                msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
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


baud_rate = 115200
pixhawk_port = "/dev/ttyACM0"
master = mavutil.mavlink_connection(pixhawk_port, baud=baud_rate)
master.wait_heartbeat()
print("Heartbeat recieved from the system!!")
print(f"Connected to pixhawk at {pixhawk_port}")
log_GPS(log_file_name="test_file.csv")