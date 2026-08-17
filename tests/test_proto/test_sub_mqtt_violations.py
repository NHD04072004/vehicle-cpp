import json
import os
import time
import yaml
import paho.mqtt.client as mqtt

from test_sub_mqtt_cameras import (
    MQTT_CONFIG_PATH,
    load_config,
    subscribe_camera_list,
)


def load_violations_topic_template(mqtt_path=MQTT_CONFIG_PATH):
    """get_violations chua duoc load_config() expose → doc truc tiep tu mqtt.yaml."""
    with open(mqtt_path, "r", encoding="utf-8") as f:
        mqtt_cfg = yaml.safe_load(f) or {}
    return str(
        mqtt_cfg.get(
            "get_violations",
            "smart_vms/ai_config/state/{camera_id}/{ai_modules}/violations",
        )
    )


def build_violations_topic(config, camera_id, template=None):
    # smart_vms/ai_config/state/{camera_id}/{ai_modules}/violations
    tpl = template or load_violations_topic_template()
    return tpl.format(
        camera_id=str(camera_id),
        ai_modules=config["AI_MODULE"],
        ai_module=config["AI_MODULE"],
    )


def on_connect(client, userdata, flags, reason_code, properties=None):
    print("Connected, reason code:", reason_code)
    topic = userdata["topic"]
    qos = userdata.get("qos", 1)
    client.subscribe(topic, qos=qos)
    print("Subscribed to:", topic)


def normalize_violations(payload):
    """VMS tra ve dict co 'allowed_codes' — danh sach ma vi pham duoc bat.

    Payload thuc te (schema_version=1):
      {"camera_id", "camera_code", "module_code", "module_enabled",
       "configured", "allowed_codes": ["RED_LIGHT", ...], "revision", ...}
    Van chap nhan list tran / cac key khac de phong VMS doi format.
    """
    if isinstance(payload, list):
        return payload
    if isinstance(payload, dict):
        for key in ("allowed_codes", "violations", "data", "items"):
            value = payload.get(key)
            if isinstance(value, list):
                return value
    return []


def on_message(client, userdata, msg):
    print("Received on:", msg.topic)
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except json.JSONDecodeError:
        print("Raw payload:", msg.payload)
        return

    violations = normalize_violations(payload)
    userdata["result"].extend(violations)

    if isinstance(payload, dict):
        print(
            "camera_code={} module={} enabled={} configured={} revision={}".format(
                payload.get("camera_code"),
                payload.get("module_code"),
                payload.get("module_enabled"),
                payload.get("configured"),
                payload.get("revision"),
            )
        )
    print(f"Violations ({len(violations)}): {violations}")
    print(json.dumps(payload, indent=2, ensure_ascii=False))


def subscribe_violations(config, camera_id, timeout=10, template=None):
    topic = build_violations_topic(config, camera_id, template=template)
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


def pick_camera_with_violations(config, cameras, timeout=5, template=None):
    """Duyet cac camera cua AI_MODULE, tra ve cam dau tien co danh sach vi pham."""
    for camera in cameras:
        camera_id = camera.get("id") or camera.get("camera_id")
        if not camera_id:
            continue
        violations = subscribe_violations(
            config, camera_id, timeout=timeout, template=template
        )
        if violations:
            return camera, violations
    return None, []


if __name__ == "__main__":
    cfg = load_config()
    tpl = load_violations_topic_template()
    print("Violations topic template:", tpl)

    camera_id = os.environ.get("CAMERA_ID")
    if camera_id:
        violations = subscribe_violations(cfg, camera_id, timeout=10, template=tpl)
        if not violations:
            raise SystemExit(f"No violations for camera_id={camera_id}")
        print(f"Camera {camera_id}: {len(violations)} violation(s)")
        raise SystemExit(0)

    cameras = subscribe_camera_list(cfg)
    if not cameras:
        raise SystemExit("No camera found for ai_modules=" + cfg["AI_MODULE"])

    camera, violations = pick_camera_with_violations(cfg, cameras, template=tpl)
    if not camera:
        raise SystemExit(
            "No violation payload for any camera with ai_modules=" + cfg["AI_MODULE"]
        )
    print(
        f"Using camera {camera.get('code')} (id={camera.get('id')}): "
        f"{len(violations)} violation(s)"
    )
