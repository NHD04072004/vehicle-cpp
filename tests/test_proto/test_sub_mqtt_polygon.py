import json
import time
import paho.mqtt.client as mqtt

from test_sub_mqtt_cameras import load_config, subscribe_camera_list


def build_polygon_topic(config, camera_code):
    # TOPIC_CAMERA_ZONES: smart_vms/cameras/+/zones
    return config["TOPIC_CAMERA_ZONES"].replace("+", str(camera_code))


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

    userdata["result"].append(payload)
    print(json.dumps(payload, indent=2, ensure_ascii=False))


def subscribe_polygon(config, camera_code, timeout=10):
    topic = build_polygon_topic(config, camera_code)
    result = []

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        userdata={
            "topic": topic,
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


def pick_camera_with_zones(config, cameras, timeout=5):
    """Ưu tiên camera có zones/lines trên MQTT (vd. camcaotoc), không lấy cam trống."""
    for camera in cameras:
        payloads = subscribe_polygon(config, camera["code"], timeout=timeout)
        if payloads:
            return camera, payloads
    return None, []


if __name__ == "__main__":
    cfg = load_config()

    cameras = subscribe_camera_list(cfg)
    if not cameras:
        raise SystemExit("No camera found for ai_modules=" + cfg["AI_MODULE"])

    camera, payloads = pick_camera_with_zones(cfg, cameras)
    if not camera:
        raise SystemExit(
            "No zone/line payload for any camera with ai_modules=" + cfg["AI_MODULE"]
        )
    print(f"Using camera {camera['code']} ({len(payloads)} polygon message(s))")
