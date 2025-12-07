# Projet Robot – Surveillance Thermique des Bâtiments    
**Système autonome de monitoring thermique intégrant navigation ROS 2, analyse IoT et visualisation Grafana**

Ce dépôt contient la partie **Robot** du mon projet de fin d'étude "Exploitation et déploiement des données IoT pour la surveillance thermique des bâtiments". Le système assure la collecte, l'analyse et la visualisation en temps réel des données thermiques via un robot mobile autonome.

## Aperçu du Projet

Le robot assure :

- **Réception des données IoT** : Collecte des mesures de température via MQTT
- **Navigation autonome** : SLAM, localisation AMCL et planification de trajectoires avec Nav2
- **Analyse intelligente** : Diagnostic thermique et prédiction spatio-temporelle via ML
- **Stockage des données** : Envoi vers InfluxDB pour historique et analyse
- **Visualisation temps réel** : Intégration complète avec Grafana, Loki et Promtail
- **Détection d'anomalies** : Surveillance continue et alertes thermiques
---

#  Installation et Configuration
## Prérequis
- **ROS 2** (version Humble ou Iron recommandée)
- **Docker** et **Docker Compose**
- **Python 3.8+**
- **Mosquitto MQTT** (broker MQTT)

## Configuration initiale

1.  **Cloner le dépôt**
    ```bash
    git clone https://github.com/Takwa51/Takwa_ws_pfe.git
    cd Takwa_ws_pfe
    ```

2.  **Build du workspace ROS 2**
    ```bash
    colcon build
    source install/setup.bash
    ```
# Lancement complet du système
1.  **Lancer le robot (base mobile)**
    ```bash
    ros2 launch tbot_node pibot_launch.yaml
    ```
2.  **Lancer RViz**
    ```bash
    ros2 run rviz2 rviz2
    ```
3.  **Navigation Nav2**
   
    *AMCL (navigation avec carte déjà générée)*
    ```bash
    ros2 launch nav2_launch amcl_and_map.launch.py
    ```
    *Navigation autonome*
    ```bash
    ros2 launch nav2_launch navigation_launch.py
    ```
# Modules du système thermique
1.  **Souscription MQTT (températures depuis FiPy)**
    ```bash
    ros2 run mon_package mqtt_subscriber_node
    ```
2.  **Envoi vers InfluxDB Cloud**
    ```bash
    ros2 run mon_package influxdb_logger_node
    ```
3.  **Surveillance d'état (Checkpoints, anomalies)**
    ```bash
    ros2 run mon_package checkpoint_monitor_node
    ```
4.  **Suivi de waypoints**
    ```bash
    ros2 run follow_waypoints follow_waypoints
    ```
5.  **Logger les diagnostics dans un fichier**
    ```bash
    ros2 run mon_package diagnostic_file_logger_node
    ```
#  Logs & Grafana
Lancer la stack Grafana + Loki + Promtail
1.  **Aller dans le dossier**
    ```bash
    cd ros_grafana_stack
    ```
2.  **Lancer la stack**
    ```bash
    docker compose up -d
    ```
3.  **Vérifier que les conteneurs tournent**
    ```bash
    docker ps
    ```
4.  **Ouvrir Loki (pour les logs)**
    ```bash
    http://localhost:3100
    ```
5.  **Connexion Grafana**
    ```bash
    user : admin  
    password : 123456789
    ```
#  Machine Learning embarqué
Le robot embarque un modèle ML (RandomForest) :
    ```bash
    mon_package/models/best_random_forest_pipeline.zip
    ```



## 👥 Auteurs

*   **Takwa Arfani** - *Développement principal* -
    [GitHub](https://github.com/Takwa51) -
    [LinkedIn](https://www.linkedin.com/in/arfani-takwa/)

## 🙏 Remerciements
**M. Lala RAJAOARISOA** - Pour son encadrement professionnel exceptionnel

**M. Abdallah BOUCHAMA** - Pour ses précieux conseils et son accompagnement

Je remercie également l'ensemble de l'équipe de l'IMT Noed Europe CERI SN pour son soutien durant ce projet de fin d'études.
