import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from influxdb_client import InfluxDBClient
import joblib
from datetime import datetime, timedelta
from collections import defaultdict

class AnomalyDetectorNode(Node):
    def __init__(self):
        super().__init__('anomaly_detector_node')
        self.subscription = self.create_subscription(
            String,
            '/temperature_data',
            self.listener_callback,
            10
        )
        self.subscription  # pour éviter un warning

        # InfluxDB config
        self.influxdb_url = "http://localhost:8086"
        self.influxdb_token = "ton_token"
        self.influxdb_org = "ton_org"
        self.influxdb_bucket = "thermal_monitoring"
        self.influx_client = InfluxDBClient(url=self.influxdb_url, token=self.influxdb_token, org=self.influxdb_org)
        self.query_api = self.influx_client.query_api()

        # Chargement du modèle ML
        self.ml_model = joblib.load("/chemin/vers/model.pkl")

        # Structure pour capteurs muets
        self.data_buffer = defaultdict(lambda: {})  # zone -> {protocole_id: timestamp}
        self.timeout = 10  # secondes

        # Timer toutes les 10 sec pour vérifier capteurs muets
        self.create_timer(self.timeout, self.check_silent_sensors)

        # Dictionnaire des zones avec coordonnées fixes
        self.zones = {
            'radiateur': [(362, 965), (213, 1181), (400, 413)],
            'porte': [(293, 90), (489, 803), (246, 1028), (420, 1508)],
            'normal': [(392, 230), (277, 536), (205, 837), (366, 659), (417, 1198), (269, 1377)]
        }

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            temp = data.get("temperature")
            rssi = data.get("rssi")
            battery = data.get("battery_state")
            protocole_id = data.get("protocole_id")
            x = data.get("x")
            y = data.get("y")

            zone = self.trouver_zone((x, y))
            if zone is None:
                self.get_logger().warning("Position hors zone connue")
                return

            # Stocker temps de réception pour détection capteurs muets
            self.data_buffer[zone][protocole_id] = time.time()

            # Partie A : détection d'anomalie
            seuil_bas, seuil_haut = self.calculer_seuils(zone)
            if seuil_bas is None:
                return

            diagnostic = None
            if temp < seuil_bas or temp > seuil_haut:
                confiance = "FAIBLE" if rssi < -85 or battery < 15 else "ÉLEVÉE"
                diagnostic = f" Anomalie détectée dans la zone {zone} : Temp={temp}°C | Confiance={confiance}"
                if battery < 10:
                    diagnostic += " ⚠️ Batterie critique"
                self.get_logger().warning(diagnostic)
            else:
                self.get_logger().info(f"✅ Température normale dans {zone} : {temp}°C")
        except Exception as e:
            self.get_logger().error(f"Erreur traitement message : {e}")

    def trouver_zone(self, position):
        x, y = position
        for zone, points in self.zones.items():
            for px, py in points:
                if abs(x - px) < 20 and abs(y - py) < 20:
                    return zone
        return None

    def calculer_seuils(self, zone):
        try:
            now = datetime.utcnow()
            start = now - timedelta(hours=24)
            query = f'''
            from(bucket: "{self.influxdb_bucket}")
              |> range(start: {start.isoformat()}Z, stop: {now.isoformat()}Z)
              |> filter(fn: (r) => r._measurement == "temperature_data" and r.zone == "{zone}")
              |> keep(columns: ["_value"])
            '''
            result = self.query_api.query(org=self.influxdb_org, query=query)
            values = [record.get_value() for table in result for record in table.records]
            if len(values) < 5:
                return None, None
            mean = sum(values) / len(values)
            std = (sum((v - mean)**2 for v in values) / len(values))**0.5
            return mean - 2 * std, mean + 2 * std
        except Exception as e:
            self.get_logger().error(f"Erreur seuils zone {zone} : {e}")
            return None, None

    def check_silent_sensors(self):
        now = time.time()
        for zone, capteurs in self.data_buffer.items():
            manquants = []
            for pid in [1, 2, 3]:
                if pid not in capteurs or now - capteurs[pid] > self.timeout:
                    manquants.append(pid)

            if manquants:
                self.get_logger().warning(f"❗ Capteurs manquants à {zone} : {manquants}")

                if len(manquants) == 1:
                    # Moyenne des deux autres capteurs
                    autres = [pid for pid in [1, 2, 3] if pid not in manquants]
                    temperatures = self.recuperer_temps_recents(zone, autres)
                    if len(temperatures) == 2:
                        estimation = sum(temperatures) / 2
                        self.get_logger().info(f" Température estimée ({zone}) pour capteur {manquants[0]} : {estimation:.2f}°C")
                elif len(manquants) == 2:
                    # Prédiction ML
                    estimation = self.predire_temperature(zone)
                    if estimation is not None:
                        self.get_logger().info(f"Prédiction ML ({zone}) pour capteurs {manquants} : {estimation:.2f}°C")

    def recuperer_temps_recents(self, zone, protocoles):
        try:
            now = datetime.utcnow()
            start = now - timedelta(minutes=10)
            query = f'''
            from(bucket: "{self.influxdb_bucket}")
              |> range(start: {start.isoformat()}Z, stop: {now.isoformat()}Z)
              |> filter(fn: (r) => r._measurement == "temperature_data" and r.zone == "{zone}")
              |> keep(columns: ["_value", "protocole_id"])
            '''
            result = self.query_api.query(org=self.influxdb_org, query=query)
            valeurs = []
            for table in result:
                for record in table.records:
                    if int(record.values.get("protocole_id")) in protocoles:
                        valeurs.append(record.get_value())
            return valeurs
        except:
            return []

    def predire_temperature(self, zone):
        try:
            now = datetime.utcnow()
            timestamp = now.timestamp()
            x, y = self.zone_to_xy(zone)
            features = [[x, y, timestamp]]
            pred = self.ml_model.predict(features)
            return pred[0]
        except Exception as e:
            self.get_logger().error(f"Erreur prédiction ML : {e}")
            return None

    def zone_to_xy(self, zone):
        points = self.zones.get(zone, [])
        if points:
            xs, ys = zip(*points)
            return sum(xs) / len(xs), sum(ys) / len(ys)
        return 0, 0

def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


