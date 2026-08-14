import json
import os
import time
import yaml
import paho.mqtt.client as mqtt

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONFIG_DIR = os.path.join(ROOT, "resources", "config")
MQTT_CONFIG_PATH = os.path.join(CONFIG_DIR, "mqtt.yaml")
APP_CONFIG_PATH = os.path.join(CONFIG_DIR, "config.yaml")
RESTFUL_CONFIG_PATH = os.path.join(CONFIG_DIR, "restful.yaml")


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_config(
    mqtt_path=MQTT_CONFIG_PATH,
    app_path=APP_CONFIG_PATH,
    restful_path=RESTFUL_CONFIG_PATH,
):
    """Load resources/config/{mqtt,config,restful}.yaml — không dùng resources/mqtt/."""
    mqtt_cfg = _load_yaml(mqtt_path)
    app = _load_yaml(app_path)
    restful = _load_yaml(restful_path)

    company_id = mqtt_cfg.get("company_id", 1)
    ai_module = str(app.get("AI_MODULE") or mqtt_cfg.get("ai_modules") or "PLATE")

    camera_list = str(mqtt_cfg.get("camera_list", "smart_vms/cameras/company/{company_id}"))
    camera_list = camera_list.format(company_id=company_id)

    get_polygon = str(
        mqtt_cfg.get("get_polygon", "smart_vms/cameras/{camera_code}/zones")
    )
    # Wildcard form for subscribe helpers (replace camera_code → +)
    topic_zones = get_polygon.replace("{camera_code}", "+")

    pub_bbox = str(mqtt_cfg.get("pub_bbox", "smart_vms/ai/bbox/{camera_code}"))
    pub_event = str(mqtt_cfg.get("pub_event", "smart_vms/ai_events/{ai_modules}"))
    pub_event = pub_event.format(ai_modules=ai_module, ai_module=ai_module)

    snap = restful.get("snapshot_api") or {}
    if isinstance(snap, str):
        snap = {"endpoint": snap}
    upload_endpoint = str(snap.get("endpoint") or snap.get("path") or "/upload/file")
    if not upload_endpoint.startswith("/"):
        upload_endpoint = "/" + upload_endpoint

    return {
        "MQTT_USERNAME": str(mqtt_cfg.get("username", "")),
        "MQTT_PASSWORD": str(mqtt_cfg.get("password", "")),
        "BROKER_HOST": str(mqtt_cfg.get("host", "")),
        "BROKER_PORT": int(mqtt_cfg.get("port", 1883)),
        "KEEPALIVE": int(mqtt_cfg.get("keep_alive", 60)),
        "QOS_EVENTS": 1,
        "QOS_COMMANDS": 1,
        "TOPIC_AI_EVENTS": pub_event,
        "TOPIC_CAMERAS": camera_list,
        "TOPIC_CAMERA_ZONES": topic_zones,
        "TOPIC_AI_BBOX": pub_bbox,
        "AI_MODULE": ai_module,
        "base_url": str(snap.get("base_url", "")),
        "category": "vehicle",
        "upload_endpoint": upload_endpoint,
    }


def build_camera_list_topic(config):
    return config["TOPIC_CAMERAS"]


def normalize_ai_modules(value):
    """Chap nhan array, string "ZONE", hoac chuoi JSON "[\\"ZONE\\"]" tu VMS."""
    if isinstance(value, list):
        return [m for m in value if isinstance(m, str)]
    if not isinstance(value, str):
        return []
    text = value.strip()
    if text.startswith("["):
        try:
            parsed = json.loads(text)
            if isinstance(parsed, list):
                return [m for m in parsed if isinstance(m, str)]
        except json.JSONDecodeError:
            pass
    return [value]


def module_matches(field, ai_module):
    return ai_module in normalize_ai_modules(field)


def on_connect(client, userdata, flags, reason_code, properties=None):
    print("Connected, reason code:", reason_code)
    topic = userdata["topic"]
    qos = userdata.get("qos", 1)
    client.subscribe(topic, qos=qos)
    print("Subscribed to:", topic)


def on_message(client, userdata, msg):
    print("Received on:", msg.topic)
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except json.JSONDecodeError:
        print("Raw payload:", msg.payload)
        return

    cameras = payload if isinstance(payload, list) else payload.get("cameras", [])

    ai_module = userdata["ai_module"]
    filtered = [c for c in cameras if module_matches(c.get("ai_modules"), ai_module)]
    userdata["result"].extend(filtered)

    print(f"Total cameras: {len(cameras)}, matched ai_modules={ai_module}: {len(filtered)}")
    print(json.dumps(filtered, indent=2, ensure_ascii=False))


def subscribe_camera_list(config, timeout=10):
    topic = build_camera_list_topic(config)
    result = []

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        userdata={
            "topic": topic,
            "ai_module": config["AI_MODULE"],
            "qos": config["QOS_COMMANDS"],
            "result": result,
        },
    )
    client.username_pw_set(config["MQTT_USERNAME"], config["MQTT_PASSWORD"])

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(
        config["BROKER_HOST"], config["BROKER_PORT"], keepalive=config["KEEPALIVE"]
    )

    print(f"Listening on '{topic}' for {timeout}s...")
    client.loop_start()
    time.sleep(timeout)
    client.loop_stop()
    client.disconnect()

    return result


if __name__ == "__main__":
    cfg = load_config()
    print("Config loaded from resources/config/:")
    print(json.dumps({k: v for k, v in cfg.items() if k != "MQTT_PASSWORD"}, indent=2))
    subscribe_camera_list(cfg)
