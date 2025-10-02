import streamlit as st
import json
import time
import base64
from pathlib import Path

# --- Chemins des images ---
bg_path = Path("/home/samar/takwa_ws/src/mon_package/assets/batiment.jpg")
json_path = Path("/home/samar/takwa_ws/diagnostic.json")

# --- Fonction pour convertir une image locale en base64 ---
def get_base64_image(img_path):
    with open(img_path, "rb") as f:
        return base64.b64encode(f.read()).decode()

# --- Ajouter le background + style ---
bg_base64 = get_base64_image(bg_path)
st.markdown(
    f"""
    <style>
    .stApp {{
        background-image: url("data:image/jpg;base64,{bg_base64}");
        background-size: cover;
        background-position: center;
        background-attachment: fixed;
    }}

    .block-container {{
        padding-left: 0px !important;
        padding-right: 25px !important;
    }}

    .custom-title {{
        display: inline-block;
        background-color: rgba(41, 128, 185, 0.8);
        color: white;
        font-size: 38px;
        font-family: "Arial", sans-serif;
        font-weight: bold;
        padding: 20px 30px;
        border-radius: 20px;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.2);
    }}

    .card {{
        padding: 15px;
        border-radius: 10px;
        margin-bottom: 10px;
        color: black;
        margin-left: 0px;
    }}
    .normal {{ background-color: rgba(46, 204, 113, 0.9); }}
    .doute {{ background-color: rgba(230, 126, 34, 0.9); }}
    .alerte {{ background-color: rgba(231, 76, 60, 0.9); }}

    h3 {{
        margin: 5px 0;
    }}
    p {{
        margin: 3px 0;
    }}
    </style>
    """,
    unsafe_allow_html=True
)

# --- Titre du dashboard ---
st.markdown(
    """
    <div style="display: flex; align-items: center; gap: 15px;">
        <span class="custom-title">🏢 Dashboard de supervision - Bâtiment IMT Nord Europe</span>
    </div>
    """,
    unsafe_allow_html=True
)

# --- Options interactives ---
if "auto_update" not in st.session_state:
    st.session_state.auto_update = True
auto_update = st.checkbox("Mise à jour automatique", value=st.session_state.auto_update, key="auto_update_checkbox")
refresh_button = st.button("Rafraîchir maintenant", key="refresh_button")

# --- Placeholder pour contenu ---
placeholder = st.empty()

# --- Initialisation du timestamp ---
if "last_refresh" not in st.session_state:
    st.session_state.last_refresh = time.time()

# --- Fonction pour afficher le dashboard ---
def display_dashboard():
    if json_path.exists():
        with open(json_path, "r") as f:
            diagnostics = json.load(f)

        # --- Sélection du checkpoint ---
        checkpoints = [f"Checkpoint {d['checkpoint']} - {d['zone_type']}" for d in diagnostics]
        selected_cp = st.selectbox(
            "Sélectionnez un checkpoint :",
            checkpoints,
            key="checkpoint_selectbox"
        )
        idx = checkpoints.index(selected_cp)
        last_diag = diagnostics[idx]

        with placeholder.container():
            st.subheader(f"Diagnostics pour {selected_cp}")
            st.markdown(f"**Total messages reçus :** {last_diag['total_messages_received']}")

            # --- Filtre par protocole ---
            protocols_available = list(last_diag['protocols'].keys())
            selected_protocols = st.multiselect(
                "Afficher les protocoles :",
                protocols_available,
                default=protocols_available,
                key="protocols_multiselect"
            )

            for proto, data in last_diag['protocols'].items():
                if proto not in selected_protocols:
                    continue
                if data:
                    temp = data.get("temperature") or data.get("predicted_temperature")
                    status = data.get("status", "Normal")
                    rssi = data.get("rssi")
                    battery = data.get("battery_state")

                    # Choix de la couleur
                    if status == "Normal":
                        card_class = "normal"
                    elif "doute" in status.lower():
                        card_class = "doute"
                    else:
                        card_class = "alerte"

                    st.markdown(
                        f"""
                        <div class="card {card_class}">
                        <h3>🌡️ {proto} : {temp} °C</h3>
                        <p>📶 Signal : {rssi if rssi is not None else 'N/A'} dBm</p>
                        <p>🔋 Batterie : {battery if battery is not None else 'N/A'} %</p>
                        <p>✅ Status : {status}</p>
                        </div>
                        """,
                        unsafe_allow_html=True
                    )

            # --- Section anomalies ---
            if last_diag['anomalies']:
                with st.expander("⚠ Anomalies détectées"):
                    for i, anomaly in enumerate(last_diag['anomalies']):
                        st.markdown(
                            f"""
                            <div style="
                                background-color: rgba(255, 255, 255, 0.9);
                                color: black;
                                padding: 10px;
                                border-radius: 10px;
                                margin-bottom: 5px;
                            ">
                                <strong>{anomaly['protocol']}</strong> : {anomaly['type']}<br>
                                Cause probable : {anomaly['cause_probable']}<br>
                                Température : {anomaly['temperature']}°C
                            </div>
                            """,
                            unsafe_allow_html=True
                        )
    else:
        placeholder.info("Aucun diagnostic disponible pour le moment...")

# --- Affichage initial ---
display_dashboard()

# --- Rafraîchissement automatique toutes les 2 secondes ---
def try_rerun():
    try:
        from streamlit.runtime.scriptrunner import rerun
        rerun()
    except ImportError:
        # Pour anciennes versions
        st.experimental_rerun()

if auto_update or refresh_button:
    if time.time() - st.session_state.last_refresh > 2:
        st.session_state.last_refresh = time.time()
        try_rerun()
