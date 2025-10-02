import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
import time
import math
import json
from datetime import datetime
import pandas as pd
import joblib

# ===== Chargement du modèle de régression =====
MODEL_PATH = "/home/samar/takwa_ws/src/mon_package/models/best_random_forest_pipeline.joblib"
model = joblib.load(MODEL_PATH)

CHECKPOINTS = [
  (-0.7622432708740234, -0.7395186424255371),   # Point 1
  (-6.340085029602051, 5.2895965576171875),     # Point 2
  (3.112633228302002, 18.122364044189453),      # Point 3
]

ZONE_TYPES = [
  "porte/escalier",
  "coloire",
  "radiateur",
]

TEMP_THRESHOLDS = [
  {"mean": 21.0, "std": 1.5},
  {"mean": 23.0, "std": 2.0},
  {"mean": 26.0, "std": 1.0},
]

DIST_THRESHOLD = 0.5  # seuil pour considérer qu'on a atteint un checkpoint

class CheckpointMonitor(Node):
    def __init__(self):
        super().__init__('checkpoint_monitor')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.sub_temp = self.create_subscription(
            String,
            '/temperature_data',
            self.temp_callback,
            10
        )

        # === Publisher pour le nouveau topic diagnostic_data ===
        self.pub_diagnostic = self.create_publisher(String, '/diagnostic_data', 10)

        self.temp_buffer = []
        self.robot_position = None
        self.checkpoint_index = 0
        self.checkpoint_reached = False
        self.checkpoint_arrival_time = None
        self.start_time = time.time()

        self.timer = self.create_timer(0.5, self.monitor_checkpoint)

    def odom_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            self.robot_position = (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('TF2 non disponible pour map → base_link')

    def temp_callback(self, msg):
        try:
            data = json.loads(msg.data)
            data['timestamp'] = time.time()
            self.temp_buffer.append(data)
        except Exception as e:
            self.get_logger().warn(f"Erreur parsing JSON: {e}")

    def monitor_checkpoint(self):
        if self.robot_position is None or self.checkpoint_index >= len(CHECKPOINTS):
            return

        goal_x, goal_y = CHECKPOINTS[self.checkpoint_index]
        zone_type = ZONE_TYPES[self.checkpoint_index]
        robot_x, robot_y = self.robot_position
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)

        if distance < DIST_THRESHOLD and not self.checkpoint_reached:
            self.checkpoint_reached = True
            self.checkpoint_arrival_time = time.time()
            self.get_logger().info(
                f' Arrivé au checkpoint {self.checkpoint_index + 1} '
                f'x={goal_x:.2f} y={goal_y:.2f}, zone type {zone_type}, attente de 20 secondes...'
            )
        elif self.checkpoint_reached:
            elapsed = time.time() - self.checkpoint_arrival_time
            if elapsed >= 20.0:
                self.diagnose_checkpoint()
                self.checkpoint_index += 1
                self.checkpoint_reached = False

    def diagnose_checkpoint(self):
        cp_num = self.checkpoint_index + 1
        expected_proto = cp_num
        start_time = self.start_time
        end_time = self.checkpoint_arrival_time + 20.0

        filtered = [
            d for d in self.temp_buffer
            if start_time <= d.get("timestamp", 0) <= end_time and d.get("protocole_id") == expected_proto
        ]

        self.get_logger().info(f" Diagnostic du Checkpoint {cp_num} (Protocole attendu: {expected_proto})")

        protocols = {'Wi-Fi': None, 'BLE': None, 'LoRa': None}
        for proto in protocols.keys():
            for d in filtered:
                if d.get("protocole") == proto:
                    protocols[proto] = d
                    break

        all_received = all(protocols.values())

        if all_received:
            self.get_logger().info(" Les 3 protocoles (Wi-Fi, BLE, LoRa) ont été reçus.")
        else:
            for proto, data in protocols.items():
                if data is None:
                    self.get_logger().warn(f" Données avec protocole {proto} absentes.")

                    # ===== Prédiction température pour protocole manquant =====
                    try:
                        timeline_dt = datetime.fromtimestamp(time.time())
                        df_input = pd.DataFrame([{
                            'x': float(self.robot_position[0]),
                            'y': float(self.robot_position[1]),
                            'protocol': str(proto),
                            'zone_type': str(ZONE_TYPES[self.checkpoint_index]),
                            'hour_of_day': int(timeline_dt.hour),
                            'minute_of_hour': int(timeline_dt.minute),
                            'protocol_id': str(expected_proto)
                        }])

                        predicted_temp = model.predict(df_input)[0]
                        self.get_logger().info(
                            f" Température estimée pour {proto} à cette position : {predicted_temp:.2f}°C"
                        )

                        # Mettre à jour la structure pour publication
                        protocols[proto] = {
                            "temperature": None,
                            "rssi": None,
                            "battery_state": None,
                            "status": "Absente - prédiction ML appliquée",
                            "predicted_temperature": float(predicted_temp)
                        }
                    except Exception as e:
                        self.get_logger().error(f" Erreur prédiction température {proto} : {e}")

        # === Anomalies ===
        anomalies_list = []

        if cp_num <= len(TEMP_THRESHOLDS):
            stats = TEMP_THRESHOLDS[self.checkpoint_index]
            mean, std = stats["mean"], stats["std"]
            low, high = mean - 2 * std, mean + 2 * std

            for proto, data in protocols.items():
                if data and data.get("temperature") is not None:
                    temp = data.get("temperature")
                    rssi = data.get("rssi")
                    battery = data.get("battery_state")

                    anomaly = temp < low or temp > high
                    rssi_low = rssi < -85 if rssi is not None else False
                    battery_low = battery < 15 if battery is not None else False
                    battery_critical = battery < 10 if battery is not None else False

                    if anomaly:
                        if battery_critical:
                            self.get_logger().error(f" Alerte maintenance ({proto}) ➔ Batterie < 10%")
                        elif battery_low or rssi_low:
                            self.get_logger().warn(
                                f" Anomalie douteuse ({proto}) ➔ Temp = {temp}°C hors [{low:.1f}, {high:.1f}], RSSI/batterie faibles"
                            )
                        else:
                            self.get_logger().error(
                                f" Anomalie confirmée ({proto}) ➔ Temp = {temp}°C hors [{low:.1f}, {high:.1f}]"
                            )

                        cause = diagnostic_cause_probable(temp, rssi, battery)
                        self.get_logger().info(f" Cause probable : {cause}")

                        anomalies_list.append({
                            "protocol": proto,
                            "type": "Anomalie confirmée" if not (battery_low or rssi_low) else "Anomalie douteuse",
                            "cause_probable": cause,
                            "temperature": temp
                        })

        # === Publication sur le topic diagnostic_data ===
        diagnostic_msg = {
            "checkpoint": cp_num,
            "zone_type": ZONE_TYPES[self.checkpoint_index],
            "timestamp": time.time(),
            "protocols": protocols,
            "anomalies": anomalies_list,
            "total_messages_received": len(filtered)
        }
        self.pub_diagnostic.publish(String(data=json.dumps(diagnostic_msg)))

        # === Affichage final ===
        for proto, data in protocols.items():
            if data and data.get("temperature") is not None:
                self.get_logger().info(
                    f" Protocole {proto} ➔ "
                    f"Température = {data.get('temperature')}°C | "
                    f"Protocole ID = {data.get('protocol_id', expected_proto)} | "
                    f"RSSI = {data.get('rssi')} | "
                    f"Battery = {data.get('battery_state')}%"
                )

        if any(protocols.values()):
            self.get_logger().info(
                f" Total messages reçus pour protocole_id={expected_proto} : {len(filtered)}"
            )
        else:
            self.get_logger().warn(
                f" Aucune donnée reçue pour le protocole_id {expected_proto}"
            )


def diagnostic_cause_probable(temp, rssi, battery):
    if temp is None or rssi is None or battery is None:
        return "Données incomplètes"

    if temp == "absente":
        if rssi < -90 and battery < 20:
            return " Capteur déconnecté ou panne matérielle"
    else:
        if temp < -50 or temp > 100:
            return " Température invalide ou capteur défectueux"
        if rssi < -85 and battery < 20:
            return " Mauvaise communication + Capteur faiblissant"
        elif rssi > -85 and battery < 20:
            return "⚠ Capteur en fin de vie (batterie faible)"
        elif rssi > -70 and battery >= 20:
            return "Vraie anomalie thermique "
    return " Cause incertaine"


def main(args=None):
    rclpy.init(args=args)
    node = CheckpointMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
