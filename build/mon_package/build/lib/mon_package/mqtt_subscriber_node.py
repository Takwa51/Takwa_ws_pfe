import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import paho.mqtt.client as mqtt

class MQTTSubscriberNode(Node):
    def __init__(self):
        super().__init__('mqtt_subscriber_node')

        self.publisher_ = self.create_publisher(String, 'temperature_data', 10)

        self.broker_address = "10.120.2.157"  # Adresse IP du broker Mosquitto 
        self.broker_port = 1883
        self.topic = "iot/temperature"

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set("takwa", "takwa")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect(self.broker_address, self.broker_port, 60)
        self.get_logger().info(" Connecté au broker MQTT")

        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f" Abonnement au topic : {self.topic}")
            client.subscribe(self.topic)
        else:
            self.get_logger().error(f" Connexion MQTT échouée avec le code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload)
            temperature = data.get("temperature")
            protocole = data.get("protocole")
            protocole_id = data.get("protocole_id")
            rssi = data.get("rssi")
            battery = data.get("battery_state")

            self.get_logger().info(f" Reçu : Temp={temperature}°C | RSSI={rssi} | Protocole={protocole} | ID={protocole_id} | Battery={battery}%")

            ros_msg = String()
            ros_msg.data = json.dumps({
                "temperature": temperature,
                "protocole": protocole,
                "protocole_id": protocole_id,
                "rssi": rssi,
                "battery_state": battery
            })
            self.publisher_.publish(ros_msg)
        except Exception as e:
            self.get_logger().error(f"Erreur de traitement MQTT : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MQTTSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
