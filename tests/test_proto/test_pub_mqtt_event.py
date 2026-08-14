import json
import time
from datetime import datetime, timezone
import paho.mqtt.client as mqtt

from test_sub_mqtt_cameras import load_config, subscribe_camera_list
from test_api_upload import (
    VEHICLE_IMAGE_PATH,
    PLATE_IMAGE_PATH,
    VEHICLE_CATEGORY,
    PLATE_CATEGORY,
    upload_file,
)

# Schema khớp MqttEventBuilder (phatnguoi_mbf): snapshot_url = full-frame,
# snapshot_base64 = crop biển (object_key MinIO, không phải base64 thật).
ENTITY_TYPE = "VEHICLE"
PLATE_TEXT = "30A12345"


def build_event_topic(config):
    return config["TOPIC_AI_EVENTS"]


def now_iso():
    return datetime.now(timezone.utc).isoformat()


def build_event_in_payload(config, camera_id, snapshot_url, snapshot_base64):
    """Sự kiện xe VÀO (IN)."""
    return {
        "ai_modules": config["AI_MODULE"],
        "camera_id": camera_id,
        "event_time": now_iso(),
        "entity_type": ENTITY_TYPE,
        "entity_id": "vehicle_1042",
        "payload": {
            "direction": "IN",
            "vehicle": {
                "license_plate": {
                    "text": PLATE_TEXT,
                    "status": "DETECTED",
                    "plate_color": "1",
                },
                "vehicle_type": "Ô TÔ",
                "car_type": None,
                "manufacturer": "TOYOTA",
                "color": "RED",
                "total_number_of_seats": 5,
            },
        },
        "snapshot_url": snapshot_url,
        "snapshot_base64": snapshot_base64,
    }


def build_event_out_payload(config, camera_id, snapshot_url, snapshot_base64):
    """Sự kiện xe RA (OUT)."""
    return {
        "ai_modules": config["AI_MODULE"],
        "camera_id": camera_id,
        "event_time": now_iso(),
        "entity_type": ENTITY_TYPE,
        "entity_id": "vehicle_1042",
        "payload": {
            "direction": "OUT",
            "vehicle": {
                "license_plate": {
                    "text": "29H98765",
                    "status": "DETECTED",
                    "plate_color": "2",
                },
                "vehicle_type": "XE MÁY",
                "car_type": None,
                "manufacturer": "HONDA",
                "color": None,
                "total_number_of_seats": None,
            },
        },
        "snapshot_url": snapshot_url,
        "snapshot_base64": snapshot_base64,
    }


def on_connect(client, userdata, flags, reason_code, properties=None):
    print("Connected, reason code:", reason_code)


def on_publish(client, userdata, mid, reason_code=None, properties=None):
    print("Published, mid:", mid)


def publish_events(config, payloads):
    topic = build_event_topic(config)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(config["MQTT_USERNAME"], config["MQTT_PASSWORD"])

    client.on_connect = on_connect
    client.on_publish = on_publish

    client.connect(
        config["BROKER_HOST"], config["BROKER_PORT"], keepalive=config["KEEPALIVE"]
    )
    client.loop_start()

    print("Topic:", topic)
    for payload in payloads:
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
    camera = cameras[0]
    camera_id = camera["id"]

    # full-frame -> snapshot_url, crop biển -> snapshot_base64
    full_frame_key = upload_file(
        camera_id, VEHICLE_CATEGORY, VEHICLE_IMAGE_PATH, cfg
    )["object_key"]
    plate_crop_key = upload_file(
        camera_id, PLATE_CATEGORY, PLATE_IMAGE_PATH, cfg
    )["object_key"]

    payloads = [
        build_event_in_payload(cfg, camera_id, full_frame_key, plate_crop_key),
        build_event_out_payload(cfg, camera_id, full_frame_key, plate_crop_key),
    ]
    publish_events(cfg, payloads)
