#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path
import json
import os

LOG_PATH = Path("/home/samar/takwa_ws/logs_grafana/diagnostic.log")

class DiagnosticFileLogger(Node):
    def __init__(self):
        super().__init__('diagnostic_file_logger')
        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        self.create_subscription(String, '/diagnostic_data', self.cb, 10)
        self.get_logger().info(f"DiagnosticFileLogger initialisé - écriture dans {LOG_PATH}")

    def cb(self, msg: String):
        # msg.data est une chaîne JSON
        try:
            # normaliser sur une ligne JSON compacte
            obj = json.loads(msg.data)
            line = json.dumps(obj, separators=(',', ':'))
        except Exception:
            # si ce n'est pas JSON valide, on écrit la chaîne brute
            line = msg.data.strip()
        try:
            with open(LOG_PATH, 'a') as f:
                f.write(line + '\n')
            self.get_logger().info("Diagnostic inscrit dans le fichier de logs")
        except Exception as e:
            self.get_logger().error(f"Erreur écriture log: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticFileLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
