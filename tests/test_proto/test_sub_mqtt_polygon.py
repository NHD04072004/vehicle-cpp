import json
import time
import paho.mqtt.client as mqtt

from test_sub_mqtt_cameras import load_config, subscribe_camera_list

ZONE_NAME = "PLATE"


def build_polygon_topic(config, camera_code):
    # TOPIC_CAMERA_ZONES: smart_vms/cameras/+/zones
    return config["TOPIC_CAMERA_ZONES"].replace("+", str(camera_code))


def find_zones_array(payload):
    if isinstance(payload, list):
        return payload
    if isinstance(payload, dict):
        for key in ("zones", "data", "polygons"):
            value = payload.get(key)
            if isinstance(value, list):
                return value
    return []


def filter_zones_by_name(payload, zone_name=ZONE_NAME):
    zones = find_zones_array(payload)
    return [
        z for z in zones
        if isinstance(z, dict) and (z.get("name") or z.get("zone_name")) == zone_name
    ]


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

    zone_name = userdata["zone_name"]
    matched = filter_zones_by_name(payload, zone_name)
    if not matched:
        print(f"No zone named '{zone_name}' in this payload, skipped.")
        return

    userdata["result"].append(matched)
    print(f"Matched zones (name={zone_name}):")
    print(json.dumps(matched, indent=2, ensure_ascii=False))


def subscribe_polygon(config, camera_code, timeout=10, zone_name=ZONE_NAME):
    topic = build_polygon_topic(config, camera_code)
    result = []

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        userdata={
            "topic": topic,
            "qos": config["QOS_COMMANDS"],
            "result": result,
            "zone_name": zone_name,
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


def pick_camera_with_zones(config, cameras, timeout=5, zone_name=ZONE_NAME):
    """Ưu tiên camera có zone tên `zone_name` trên MQTT (vd. camcaotoc), không lấy cam trống."""
    for camera in cameras:
        payloads = subscribe_polygon(
            config, camera["code"], timeout=timeout, zone_name=zone_name
        )
        if payloads:
            return camera, payloads
    return None, []


if __name__ == "__main__":
    cfg = load_config()

    cameras = subscribe_camera_list(cfg)
    if not cameras:
        raise SystemExit("No camera found for ai_modules=" + cfg["AI_MODULE"])

    camera, payloads = pick_camera_with_zones(cfg, cameras, zone_name=ZONE_NAME)
    if not camera:
        raise SystemExit(
            f"No zone named '{ZONE_NAME}' for any camera with ai_modules=" + cfg["AI_MODULE"]
        )
    print(f"Using camera {camera['code']} ({len(payloads)} zone='{ZONE_NAME}' message(s))")
