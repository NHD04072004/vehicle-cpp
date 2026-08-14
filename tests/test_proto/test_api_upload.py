import os
import json
import requests

from test_sub_mqtt_cameras import load_config, subscribe_camera_list

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "data")
VEHICLE_IMAGE_PATH = os.path.abspath(os.path.join(DATA_DIR, "vehicle", "xe.jpg"))
PLATE_IMAGE_PATH = os.path.abspath(os.path.join(DATA_DIR, "plate", "bien.jpg"))

VEHICLE_CATEGORY = "vehicle"
PLATE_CATEGORY = "plate"


def upload_url(config):
    endpoint = config.get("upload_endpoint") or "/upload/file"
    return config["base_url"].rstrip("/") + endpoint


def upload_file(camera_id, category, file_path, config=None):
    url = upload_url(config) if config else upload_url(load_config())
    with open(file_path, "rb") as f:
        files = {"file": (os.path.basename(file_path), f, "image/jpeg")}
        data = {"camera_id": camera_id, "category": category}
        response = requests.post(url, files=files, data=data)

    result = response.json()
    print("Status:", response.status_code)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return result


if __name__ == "__main__":
    cfg = load_config()

    cameras = subscribe_camera_list(cfg)
    if not cameras:
        raise SystemExit("No camera found for ai_modules=" + cfg["AI_MODULE"])
    camera_id = cameras[0]["id"]

    upload_file(camera_id, VEHICLE_CATEGORY, VEHICLE_IMAGE_PATH, cfg)
    upload_file(camera_id, PLATE_CATEGORY, PLATE_IMAGE_PATH, cfg)
