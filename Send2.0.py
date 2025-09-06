import cv2 as cv
import numpy as np
import http.server as sr
import time, json, threading, asyncio
from contextlib import suppress
from sys import platform

from bleak import BleakScanner, BleakClient

# -----------------------------
# Camera & HSV configuration
# -----------------------------
camera = cv.VideoCapture(0)
colorFrame = None

try:
    with open("HSV.json", "r") as file:
        obj = json.load(file)
        orangeLow = obj["orangeLow"]
        orangeHigh = obj["orangeHigh"]
        temperature = obj["temperature"]

except Exception:
    orangeLow = [0, 0, 0]
    orangeHigh = [0, 0, 0]
    temperature = 0

orangeLowOpenCV  = np.array((orangeLow[0] / 2,  orangeLow[1] / 100 * 255, orangeLow[2] / 100 * 255), dtype=np.uint8, ndmin=1)
orangeHighOpenCV = np.array((orangeHigh[0] / 2, orangeHigh[1] / 100 * 255, orangeHigh[2] / 100 * 255), dtype=np.uint8, ndmin=1)

ip = "127.0.0.1"
port = 8000

ball_coords = []
latest_xy = None
lock = threading.Lock()

# -----------------------------
# BLE configuration (SPIKE)
# -----------------------------
HUB_NAME = "Phoenix"
CHAR_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"

ready_event = asyncio.Event()
_ble_client = None

async def _ble_connect():
    print("Scanning for SPIKE hub…")
    device = await BleakScanner.find_device_by_name(HUB_NAME)
    if not device:
        raise RuntimeError(f"Could not find hub named {HUB_NAME}")
    client = BleakClient(device)
    await client.connect()
    print("Connected to hub.")

    def handle_rx(_, data: bytearray):
        if not data:
            return
        if data[0] == 0x01:
            payload = data[1:]

            if payload.strip() == b"rdy":
                ready_event.set()

    await client.start_notify(CHAR_UUID, handle_rx)
    return client

async def _ble_loop():
    global _ble_client
    _ble_client = await _ble_connect()
    try:
        while True:
            await ready_event.wait()
            ready_event.clear()

            with lock:
                xy = latest_xy

            if xy is None:
                continue

            x, y = xy
            msg = f"{x},{y}\n".encode()
            payload = b"\x06" + msg
            try:
                await _ble_client.write_gatt_char(CHAR_UUID, payload, response=True)
            except Exception as e:
                print(f"BLE write failed: {e}")

                with suppress(Exception):
                    await _ble_client.disconnect()
                _ble_client = await _ble_connect()
    finally:
        with suppress(Exception):
            await _ble_client.disconnect()

def start_ble_thread():
    def runner():
        with suppress(asyncio.CancelledError):
            asyncio.run(_ble_loop())
    t = threading.Thread(target=runner, daemon=True)
    t.start()
    return t

# -----------------------------
# Simple HTTP server for HSV UI
# -----------------------------
class Server(sr.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.path = '/index.html'
            return sr.SimpleHTTPRequestHandler.do_GET(self)
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                try:
                    with lock:
                        frame = colorFrame
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    buffer = cv.imencode(".jpg", frame)[1]
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.end_headers()
                    self.wfile.write(bytes(buffer))
                    self.wfile.write(b'\r\n')
                    time.sleep(0.1)
                except Exception as e:
                    print(f"Stream exception: {e}")
                    break
        elif self.path == "/getHSV":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(bytes(json.dumps({"orangeLow": orangeLow, "orangeHigh": orangeHigh}), "UTF-8"))
        else:
            self.send_error(404, "File not found")

    def do_POST(self):
        global orangeLow, orangeHigh, orangeLowOpenCV, orangeHighOpenCV, temperature
        if self.path == '/':
            content_length = int(self.headers.get('Content-Length', "0"))
            post_data_dict = {}
            if content_length > 0:
                post_data_bytes = self.rfile.read(content_length)
                post_data_str = post_data_bytes.decode("UTF-8")
                for item in post_data_str.split('&'):
                    if '=' in item:
                        k, v = item.split('=', 1)
                        post_data_dict[k] = v
            if len(post_data_dict) >= 7:
                orangeLow = [
                    float(post_data_dict.get("H_low", 0)),
                    float(post_data_dict.get("S_low", 0)),
                    float(post_data_dict.get("V_low", 0)),
                ]
                orangeHigh = [
                    float(post_data_dict.get("H_high", 0)),
                    float(post_data_dict.get("S_high", 0)),
                    float(post_data_dict.get("V_high", 0)),
                ]
                orangeLowOpenCV  = np.array((orangeLow[0] / 2,  orangeLow[1] / 100 * 255, orangeLow[2] / 100 * 255), dtype=np.uint8, ndmin=1)
                orangeHighOpenCV = np.array((orangeHigh[0] / 2, orangeHigh[1] / 100 * 255, orangeHigh[2] / 100 * 255), dtype=np.uint8, ndmin=1)

                try:
                    temperature = int(post_data_dict.get("temperature", "0"))
                except Exception:
                    temperature = 0
                calibrate_camera(camera)

                with open("HSV.json", "w") as file:
                    json.dump({"orangeLow": orangeLow, "orangeHigh": orangeHigh, "temperature": temperature}, file)
            self.send_response(301)
            self.send_header("Location", "/")
            self.end_headers()

# -----------------------------
# FPS helper
# -----------------------------
numFrames = 0
lastFrameTimestamp = time.perf_counter()
def incrementFrames() -> None:
    global lastFrameTimestamp, numFrames
    numFrames += 1
    if time.perf_counter() - lastFrameTimestamp >= 1:
        print(f"FPS: {numFrames}")
        numFrames = 0
        lastFrameTimestamp = time.perf_counter()

# -----------------------------
# Camera controls
# -----------------------------
def calibrate_camera(cam: cv.VideoCapture) -> None:
    with lock:
        cam.set(cv.CAP_PROP_FRAME_WIDTH, 600)
        cam.set(cv.CAP_PROP_FRAME_HEIGHT, 600)
        cam.set(cv.CAP_PROP_AUTO_WB, 1)
        # cam.set(cv.CAP_PROP_AUTO_WB, 0)
        # cam.set(cv.CAP_PROP_WB_TEMPERATURE, temperature)
        cam.set(cv.CAP_PROP_SATURATION, 200)
        cam.set(cv.CAP_PROP_APERTURE, 8)

# -----------------------------
# Vision loop
# -----------------------------
def main() -> None:
    global colorFrame, ball_coords, latest_xy
    calibrate_camera(camera)

    while True:
        with lock:
            if not camera.isOpened():
                print("Cannot open camera")
                break
            ret, frame = camera.read()
            if not ret:
                continue

            frame = cv.GaussianBlur(frame, (17, 17), 1.3, sigmaY=0.0)
            hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            mask = cv.inRange(hsvFrame, orangeLowOpenCV, orangeHighOpenCV)
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            coords = []
            chosen = None
            for contour in contours:
                x, y, w, h = cv.boundingRect(contour)
                if w * h > 270:
                    cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    coords.append((x, y))
                    if chosen is None:
                        chosen = (x, y)

            colorFrame = frame
            ball_coords = coords
            latest_xy = chosen

        incrementFrames()

# -----------------------------
# Boot
# -----------------------------
if __name__ == "__main__":
    start_ble_thread()

    server = sr.HTTPServer((ip, port), Server)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"HTTP server on http://{ip}:{port}/")

    main()
