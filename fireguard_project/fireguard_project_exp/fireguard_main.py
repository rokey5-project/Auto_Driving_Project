

import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from flask import (
    Flask, render_template, request, g, redirect, url_for,
    session, send_from_directory, Response, jsonify
)
import sqlite3
import time
import cv2
import threading
import numpy as np
from datetime import datetime

# =============================================================
# SHARED STATE
# =============================================================
BASE = os.path.dirname(os.path.abspath(__file__))
BRIDGE = os.path.join(BASE, "ros_bridge")
sys.path.insert(0, BASE)
sys.path.insert(0, BRIDGE)

from ros_bridge.utils import shared_state
from ros_bridge.fireguard_ros_bridge import start_ros_bridge

print("shared_state ID =", id(shared_state))


app = Flask(__name__)
app.secret_key = "fireguard_secret_key"

DB_PATH = "/home/rokey/fireguard_project_exp/fire_detection.db"

current_view = "multi"
zoom_target = None
current_decibel = 72


# =============================================================
# DB
# =============================================================
def get_db():
    if "db" not in g:
        g.db = sqlite3.connect(DB_PATH)
        g.db.row_factory = sqlite3.Row
    return g.db


def ensure_tables():
    db = get_db()
    cursor = db.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS cam_detect (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            topic TEXT NOT NULL,
            message TEXT,
            detect_time DATETIME DEFAULT CURRENT_TIMESTAMP
        );
    """)

    # 로봇 미션 로그 테이블
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS robot_mission_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            robot_id TEXT,
            start_time DATETIME,
            end_time DATETIME,
            status TEXT
        );
    """)

    db.commit()


@app.teardown_appcontext
def close_db(error):
    db = g.pop("db", None)
    if db:
        db.close()


# =============================================================
# ROBOT MISSION API (ROS 브리지에서 사용)
# =============================================================
@app.route("/api/robot/start", methods=["POST"])
def api_robot_start():
    data = request.get_json(force=True)
    robot_id = data.get("robot_id")

    db = get_db()

    row = db.execute("""
        SELECT detect_time FROM cam_detect
        WHERE topic = '/cam/fire_detection_status'
        ORDER BY detect_time DESC
        LIMIT 1
    """).fetchone()

    if row and row["detect_time"]:
        start_time = row["detect_time"]
    else:
        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    db.execute("""
        UPDATE robot_mission_log
        SET status = 'done'
        WHERE robot_id = ? AND status = 'running'
    """, (robot_id,))

    db.execute("""
        INSERT INTO robot_mission_log (robot_id, start_time, status)
        VALUES (?, ?, 'running')
    """, (robot_id, start_time))

    db.commit()

    return jsonify({"result": "started", "robot_id": robot_id, "start_time": start_time})


@app.route("/api/robot/dock", methods=["POST"])
def api_robot_dock():
    data = request.get_json(force=True)
    robot_id = data.get("robot_id")

    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    db = get_db()

    db.execute("""
        UPDATE robot_mission_log
        SET end_time = ?, status = 'done'
        WHERE id = (
            SELECT id FROM robot_mission_log
            WHERE robot_id = ?
            ORDER BY id DESC
            LIMIT 1
        )
    """, (now_str, robot_id))

    db.commit()

    return jsonify({
        "result": "docked",
        "robot_id": robot_id,
        "end_time": now_str
    })


# =============================================================
# LOGIN
# =============================================================
USERNAME = "admin"
PASSWORD = "rokey1234"


@app.route("/")
def home():
    if "username" in session:
        return redirect("/dashboard")
    return redirect("/login")


@app.route("/login", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        if (
            request.form.get("username") == USERNAME
            and request.form.get("password") == PASSWORD
        ):
            session["username"] = USERNAME
            return redirect("/dashboard")
        return "<script>alert('아이디 또는 비밀번호가 올바르지 않습니다.');location.href='/login'</script>"
    return render_template("login_center.html")


@app.route("/logout")
def logout():
    session.clear()
    return redirect("/login")


# =============================================================
# VIEW SWITCH
# =============================================================
@app.route("/change_main_view")
def change_main_view():
    global current_view, zoom_target
    mode = request.args.get("mode")

    if mode in ["slam", "logs", "multi"]:
        current_view = mode
        zoom_target = None
    return "ok"


@app.route("/zoom")
def zoom():
    global current_view, zoom_target
    zoom_target = request.args.get("target")
    current_view = "zoom"
    return "ok"


@app.route("/unzoom")
def unzoom():
    global current_view, zoom_target
    zoom_target = None
    current_view = "multi"
    return "ok"


# =============================================================
# ROBOT POSE API
# =============================================================
@app.route("/robot_pose")
def robot_pose():
    return shared_state["robot_pose"]


# =============================================================
# DASHBOARD
# =============================================================
@app.route("/dashboard")
def dashboard():
    if "username" not in session:
        return redirect("/login")

    db = get_db()

    row = db.execute("""
        SELECT message, detect_time FROM cam_detect
        WHERE topic='/cam/fire_detection_status'
        ORDER BY detect_time DESC LIMIT 1
    """).fetchone()

    if row:
        fire_message = row["message"]
        fire_detected = (fire_message != "No Detected")
        try:
            ts = time.mktime(time.strptime(row["detect_time"], "%Y-%m-%d %H:%M:%S"))
            flashing = (time.time() - ts <= 5)
        except Exception:
            flashing = True
    else:
        fire_detected = False
        flashing = False

    robots = [
        ("TB4A", "robot4"),
        ("TB4B", "robot6"),
    ]

    active_robots = []

    for display_id, key in robots:

        battery_raw = shared_state["battery"].get(key)
        battery_str = "N/A" if battery_raw is None else f"{battery_raw:.0f}%"

        camera_ok = bool(shared_state["camera"].get(key))

        pose = shared_state["robot_pose"].get(key, {})
        pose_x = pose.get("x", None)
        connected = (battery_raw is not None) or (pose_x is not None)

        active_robots.append({
            "id": display_id,
            "ros_id": key,
            "type": "TB4",
            "connected": connected,
            "camera_ok": camera_ok,
            "battery": battery_str,
        })

    connected_robots = [r for r in active_robots if r["connected"]]
    depth_A = shared_state["depth"].get("robot4")
    depth_B = shared_state["depth"].get("robot6")

    if isinstance(depth_A, str):
        depth_A = None
    if isinstance(depth_B, str):
        depth_B = None
    if connected_robots:
        key = connected_robots[0]["ros_id"]
        depth_val = shared_state["depth"].get(key)

        distance_remaining = depth_val
        
    else:
        distance_remaining = "N/A"

    # 전방 장애물 최소 거리: TB4A / TB4B
    depth_A_val = shared_state["depth"].get("robot4")
    depth_B_val = shared_state["depth"].get("robot6")

    depth_A = "N/A" if depth_A_val is None else f"{depth_A_val:.2f}"
    depth_B = "N/A" if depth_B_val is None else f"{depth_B_val:.2f}"
    # 

    page = int(request.args.get("page", 1))
    per_page = 10
    offset = per_page * (page - 1)

    rows = db.execute("""
        SELECT topic, message, detect_time
        FROM cam_detect
        WHERE topic='/cam/fire_detection_status'
        ORDER BY detect_time DESC LIMIT ? OFFSET ?
    """, (per_page, offset)).fetchall()

    total_rows = db.execute("""
        SELECT COUNT(*) FROM cam_detect
        WHERE topic='/cam/fire_detection_status'
    """).fetchone()[0]

    total_pages = total_rows // per_page + (1 if total_rows % per_page else 0)

    mission_rows = db.execute("""
        SELECT
            robot_id,
            start_time,
            end_time,
            status,
            CAST(
                CASE
                    WHEN end_time IS NOT NULL
                    THEN (julianday(end_time) - julianday(start_time)) * 86400
                    ELSE NULL
                END
            AS INTEGER) AS duration
        FROM robot_mission_log
        WHERE id IN (
            SELECT MAX(id) FROM robot_mission_log GROUP BY robot_id
        )
        ORDER BY robot_id
    """).fetchall()

    return render_template(
        "dashboard.html",
        fire_detected=fire_detected,
        flashing=flashing,
        active_robots=active_robots,
        connected_robots=connected_robots,
        current_view=current_view,
        zoom_target=zoom_target,
        zone_id="DepartmentStore",
        distance_remaining=distance_remaining,
        current_decibel=current_decibel,
        rows=rows,
        page=page,
        total_pages=total_pages,
        mission_rows=mission_rows,

        # depth 값 템플릿으로 전달
        depth_A=depth_A,
        depth_B=depth_B,
    )


# =============================================================
# VIDEO FEEDS
# =============================================================
@app.route("/video_feed/<robot_id>")
def video_feed(robot_id):

    key_map = {
        "A": "robot4",
        "4": "robot4",
        "TB4A": "robot4",
        "robot4": "robot4",

        "B": "robot6",
        "6": "robot6",
        "TB4B": "robot6",
        "robot6": "robot6",
    }
    internal_id = key_map.get(robot_id, robot_id)

    def generate():
        while True:
            frame = shared_state["latest_frame"].get(internal_id)

            if frame is None:
                time.sleep(0.03)
                continue

            if isinstance(frame, bytes):
                jpeg_bytes = frame
            else:
                ret, jpeg = cv2.imencode(".jpg", frame)
                if not ret:
                    time.sleep(0.03)
                    continue
                jpeg_bytes = jpeg.tobytes()

            yield (
                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                + jpeg_bytes
                + b"\r\n"
            )
            time.sleep(0.03)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# =============================================================
# cam1 / cam2 YOLO 스트림
# =============================================================
@app.route("/cam_feed/<int:cam_id>")
def cam_feed(cam_id):
    key = "cam1" if cam_id == 1 else "cam2"

    def generate():
        while True:
            img = shared_state["latest_frame"].get(key)
            if img is not None:
                ret, jpeg = cv2.imencode(".jpg", img)
                if ret:
                    yield (
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                        + jpeg.tobytes()
                        + b"\r\n"
                    )
            time.sleep(0.03)

    return Response(generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame")


# =============================================================
# SLAM MAP 기본 PNG
# =============================================================
@app.route("/slam_feed")
def slam_feed():
    return send_from_directory("static/img", "slam_map.png")


# =============================================================
# SLAM overlay
# =============================================================
@app.route("/slam_overlay")
def slam_overlay():

    base_path = os.path.join("static", "img", "slam_map.png")
    base_img = cv2.imread(base_path)

    if base_img is None:
        blank = np.zeros((300, 300, 3), dtype=np.uint8)
        ok, jpeg = cv2.imencode(".jpg", blank)
        return Response(jpeg.tobytes(), mimetype="image/jpeg")

    h, w = base_img.shape[:2]

    res = 0.05
    org_x = -2.892527692
    org_y = -0.975270054

    def world_to_map(wx, wy):
        mx = int((wx - org_x) / res)
        my = int(h - 1 - ((wy - org_y) / res))
        return mx, my

    dx6 = 0.7475
    dy6 = 2.375

    for (x, y) in (shared_state.get("pose_history_robot4") or []):
        mx, my = world_to_map(x, y)
        if 0 <= mx < w and 0 <= my < h:
            cv2.circle(base_img, (mx, my), 2, (0, 0, 255), -1)

    for (x, y) in (shared_state.get("pose_history_robot6") or []):
        x2 = x + dx6
        y2 = y + dy6
        mx, my = world_to_map(x2, y2)
        if 0 <= mx < w and 0 <= my < h:
            cv2.circle(base_img, (mx, my), 2, (255, 0, 0), -1)

    p4 = shared_state.get("robot_pose", {}).get("robot4", {})
    p6 = shared_state.get("robot_pose", {}).get("robot6", {})

    if p4.get("x") is not None:
        mx, my = world_to_map(p4["x"], p4["y"])
        if 0 <= mx < w and 0 <= my < h:
            cv2.circle(base_img, (mx, my), 6, (0, 0, 255), -1)

    if p6.get("x") is not None:
        x2 = p6["x"] + dx6
        y2 = p6["y"] + dy6
        mx, my = world_to_map(x2, y2)
        if 0 <= mx < w and 0 <= my < h:
            cv2.circle(base_img, (mx, my), 6, (255, 0, 0), -1)

    ok, jpeg = cv2.imencode(".jpg", base_img)
    if not ok:
        blank = np.zeros((300, 300, 3), dtype=np.uint8)
        ok, jpeg = cv2.imencode(".jpg", blank)

    return Response(jpeg.tobytes(), mimetype="image/jpeg")


# =============================================================
# DECIBEL
# =============================================================
@app.route("/boost_audio", methods=["POST"])
def boost_audio():
    global current_decibel
    current_decibel += 10
    shared_state["audio"]["target_gain"] = current_decibel
    shared_state["audio"]["should_beep"] = True
    return redirect("/dashboard")


@app.route("/reduce_audio", methods=["POST"])
def reduce_audio():
    global current_decibel
    current_decibel -= 10
    shared_state["audio"]["target_gain"] = current_decibel
    return redirect("/dashboard")




# ================================
# 실시간 로봇 상태 API
# ================================
@app.route("/api/realtime/robot_state")
def api_realtime_robot_state():
    robots = ["robot4", "robot6"]
    data = {}

    for key in robots:
        pose = shared_state.get("robot_pose", {}).get(key, {})
        data[key] = {
            "connected": bool(shared_state["camera"].get(key)),
            "battery": shared_state["battery"].get(key),
            "depth": shared_state["depth"].get(key),
            "x": pose.get("x"),
            "y": pose.get("y"),
        }

    return jsonify(data)


# ================================
# SLAM 실시간 오버레이 (캐시 방지)
# ================================
@app.route("/slam_overlay_live")
def slam_overlay_live():
    return redirect("/slam_overlay?t=" + str(time.time()))


# ================================
# 실시간 미션 상태 API
# ================================
@app.route("/api/realtime/mission_status")
def api_realtime_mission_status():
    db = get_db()

    rows = db.execute("""
        SELECT
            robot_id,
            start_time,
            end_time,
            status,
            CAST(
                CASE
                    WHEN end_time IS NOT NULL
                    THEN (julianday(end_time) - julianday(start_time)) * 86400
                    ELSE (julianday('now') - julianday(start_time)) * 86400
                END
            AS INTEGER) AS duration
        FROM robot_mission_log
        WHERE id IN (
            SELECT MAX(id) FROM robot_mission_log GROUP BY robot_id
        )
        ORDER BY robot_id
    """).fetchall()

    result = []
    for r in rows:
        result.append({
            "robot_id": r["robot_id"],
            "start_time": r["start_time"],
            "end_time": r["end_time"],
            "status": r["status"],
            "duration": r["duration"],
        })

    return jsonify(result)


# ================================
# 시스템 헬스 체크
# ================================
@app.route("/api/system/health")
def api_system_health():
    return jsonify({
        "ros_running": bool(shared_state),
        "robot4_connected": bool(shared_state["camera"].get("robot4")),
        "robot6_connected": bool(shared_state["camera"].get("robot6")),
        "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    })


# =============================================================
# MAIN
# =============================================================
if __name__ == "__main__":

    with app.app_context():
        ensure_tables()

    t = threading.Thread(target=start_ros_bridge, daemon=True)
    t.start()

    app.run(debug=False, threaded=True, use_reloader=False)
