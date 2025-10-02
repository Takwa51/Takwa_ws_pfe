#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

class DiagnosticSaver(Node):
    def __init__(self):
        super().__init__('diagnostic_saver')
        self.subscription = self.create_subscription(
            String,
            '/diagnostic_data',
            self.listener_callback,
            10)
        
        self.file_path = "diagnostic.json"

        # Si le fichier n’existe pas, créer une liste JSON vide
        if not os.path.exists(self.file_path):
            with open(self.file_path, "w") as f:
                json.dump([], f)

    def listener_callback(self, msg):
        try:
            # Charger le JSON existant (liste)
            with open(self.file_path, "r") as f:
                data = json.load(f)

            # Ajouter le nouveau diagnostic (c’est déjà une string JSON)
            new_entry = json.loads(msg.data)  
            data.append(new_entry)

            # Réécrire le fichier avec la nouvelle liste
            with open(self.file_path, "w") as f:
                json.dump(data, f, indent=4)

            self.get_logger().info(f"✅ Diagnostic sauvegardé dans {self.file_path}")

        except Exception as e:
            self.get_logger().error(f"Erreur sauvegarde diagnostic: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
