import json
import os

from test_api_upload import (
    PLATE_CATEGORY,
    PLATE_IMAGE_PATH,
    VEHICLE_CATEGORY,
    VEHICLE_IMAGE_PATH,
    upload_file,
)
from test_pub_mqtt_event import ENTITY_TYPE, PLATE_TEXT, now_iso, publish_events
from test_sub_mqtt_cameras import load_config, subscribe_camera_list

# Schema khớp MqttEventBuilder.build_violation_event (phatnguoi_mbf):
# snapshot_url = full-frame, snapshot_base64 = crop biển (object_key MinIO).
ENTITY_ID = "vehicle_1042"


def build_violation_payload(
    config,
    camera_id,
    snapshot_url,
    snapshot_base64,
    violation_type_code="NO_HELMET",
):
    return {
        "ai_modules": config["AI_MODULE"],
        "camera_id": camera_id,
        "event_time": now_iso(),
        "entity_type": ENTITY_TYPE,
        "entity_id": ENTITY_ID,
        "payload": {
            "direction": "IN",
            "vehicle": {
                "license_plate": {
                    "text": PLATE_TEXT,
                    "status": "DETECTED",
                    "plate_color": "1",
                },
                "vehicle_type": "Ô tô",
                "car_type": None,
                "manufacturer": "TOYOTA",
                "color": "RED",
                "total_number_of_seats": 5,
            },
            "violation_type_code": violation_type_code,
            "violation_evidence": {"road_type": "highway"},
        },
        "snapshot_url": snapshot_url,
        "snapshot_base64": snapshot_base64,
    }


if __name__ == "__main__":
    cfg = load_config()

    camera_id = os.environ.get("CAMERA_ID")
    if not camera_id:
        cameras = subscribe_camera_list(cfg)
        if not cameras:
            raise SystemExit("No camera found for ai_modules=" + cfg["AI_MODULE"])
        camera_id = cameras[0]["id"]

    violation_type_code = os.environ.get("VIOLATION_TYPE_CODE", "NO_HELMET")

    # full-frame -> snapshot_url, crop biển -> snapshot_base64
    full_frame_key = upload_file(
        camera_id, VEHICLE_CATEGORY, VEHICLE_IMAGE_PATH, cfg
    )["object_key"]
    plate_crop_key = upload_file(
        camera_id, PLATE_CATEGORY, PLATE_IMAGE_PATH, cfg
    )["object_key"]

    payload = build_violation_payload(
        cfg, camera_id, full_frame_key, plate_crop_key, violation_type_code
    )
    print("Violation payload:")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    publish_events(cfg, [payload])
