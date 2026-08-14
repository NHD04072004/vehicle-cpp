import json
import time
from datetime import datetime
import paho.mqtt.client as mqtt

from test_sub_mqtt_cameras import load_config, subscribe_camera_list


def build_bbox_topic(config, camera_code):
    return config["TOPIC_AI_BBOX"].format(camera_code=camera_code)


def build_bbox_payload(config, camera_code):
    return {
        "camera_code": camera_code,
        "ai_modules": config["AI_MODULE"],
        "timestamp": datetime.now().timestamp(),
        "detections": [
            {
                "id": "vehicle_1",
                "class": "car",
                "confidence": 0.94,
                "bbox": [0.38, 0.22, 0.58, 0.68],
                "label": "car 30A12345",
                "color": "#00FF00",
            }
        ],
    }


def on_connect(client, userdata, flags, reason_code, properties=None):
    print("Connected, reason code:", reason_code)


def on_publish(client, userdata, mid, reason_code=None, properties=None):
    print("Published, mid:", mid)


def publish_bbox(config, payload):
    topic = build_bbox_topic(config, payload["camera_code"])

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(config["MQTT_USERNAME"], config["MQTT_PASSWORD"])

    client.on_connect = on_connect
    client.on_publish = on_publish

    client.connect(
        config["BROKER_HOST"], config["BROKER_PORT"], keepalive=config["KEEPALIVE"]
    )
    client.loop_start()

    print("Topic:", topic)
    print("Payload:")
    print(json.dumps(payload, indent=2, ensure_ascii=False))

    info = client.publish(
        topic, json.dumps(payload, ensure_ascii=False), qos=config["QOS_EVENTS"]
    )
    info.wait_for_publish(timeout=10)

    time.sleep(1)
    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    cfg = load_config()

    cameras = subscribe_camera_list(cfg)
    if not cameras:
        raise SystemExit("No camera found for ai_modules=" + cfg["AI_MODULE"])
    camera_code = cameras[0]["code"]

    payload = build_bbox_payload(cfg, camera_code)
    publish_bbox(cfg, payload)
