import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry  

from influxdb_client import InfluxDBClient, Point, WritePrecision
import os
import json
from datetime import datetime

class InfluxDBWriterNode(Node):
    def __init__(self):
        super().__init__('influxdb_writer_node')

        # Abonnement à la position réelle du robot (Odometry)
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

        # Abonnement aux données ROS de température
        self.create_subscription(String, 'temperature_data', self.listener_callback, 10)

        #  Connexion à InfluxDB Cloud
        self.influx_url = os.getenv("INFLUX_URL", "https://us-east-1-1.aws.cloud2.influxdata.com")
        self.influx_token = os.getenv("INFLUX_TOKEN", "FMe-TMxZeTmYJ163IWbWSdBMlV-OTg5tMTo40jbPQ0UnGIZpw3vPEEJCmfkBIVuwTNSUm60bFUCUJ27RWmSOAw==")
        self.influx_org = os.getenv("INFLUX_ORG", "iot_db")
        self.influx_bucket = "iot_db"
        self.client = InfluxDBClient(url=self.influx_url, token=self.influx_token, org=self.influx_org)
        self.get_logger().info(" Connecté à InfluxDB")

    def pose_callback(self, msg):
        self.pose = msg.pose.pose  # Accès à la position via Odometry

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            temperature = data.get("temperature")
            protocole = data.get("protocole")
            protocole_id = data.get("protocole_id")
            rssi = data.get("rssi")
            battery = data.get("battery_state")

            if self.pose:
                pos_x = float(self.pose.position.x)
                pos_y = float(self.pose.position.y)
            else:
                pos_x = 0.0
                pos_y = 0.0

            point = Point("temperature_data") \
                .field("temperature", float(temperature)) \
                .field("rssi", int(rssi)) \
                .field("battery", int(battery)) \
                .field("position_x", pos_x) \
                .field("position_y", pos_y) \
                .field("protocole_id", int(protocole_id)) \
                .tag("protocole", protocole) \
                .time(datetime.utcnow(), WritePrecision.NS)

            self.client.write_api().write(bucket=self.influx_bucket, org=self.influx_org, record=point)
            self.get_logger().info(f" Donnée enregistrée : Temp={temperature} | x={pos_x} | y={pos_y} | ID={protocole_id}")

        except Exception as e:
            self.get_logger().error(f" Erreur InfluxDB : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InfluxDBWriterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()