# # import asyncio
# # import cv2
# # import numpy as np
# # import time
# # from contextlib import suppress
# # from picamera2 import Picamera2
# # from bleak import BleakScanner, BleakClient
# #
# # # BLE Configuration
# # HUB_NAME = "Phoenix"
# # CHAR_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"
# # REAL_DIAMETER_CM = 4.0
# # FOCAL_LENGTH_PIXELS = 494
# # DETECTION_INTERVAL = 3
# # STOP_DISTANCE_CM = 5.0
# #
# # orange_lower = np.array([60, 80, 200], dtype="uint8")
# # orange_upper = np.array([120, 255, 255], dtype="uint8")
# #
# # ready_event = asyncio.Event()
# #
# # def handle_rx(_, data: bytearray):
# #     if data[0] == 0x01:
# #         payload = data[1:]
# #         try:
# #             text = payload.decode()
# #             print("Received:", text)
# #             if text.strip() == "rdy":
# #                 ready_event.set()
# #         except Exception:
# #             pass
# #
# # async def connect_to_spike():
# #     print("Connecting to Passer hub...")
# #     device = await BleakScanner.find_device_by_name(HUB_NAME)
# #     if not device:
# #         raise Exception(f"Could not find hub named {HUB_NAME}")
# #     client = BleakClient(device)
# #     await client.connect()
# #     await client.start_notify(CHAR_UUID, handle_rx)
# #     print("Connected.")
# #     return client
# #
# # async def send_distance(client, distance_cm):
# #     message = f"{distance_cm:.2f}\n"
# #     await ready_event.wait()
# #     ready_event.clear()
# #     await client.write_gatt_char(CHAR_UUID, b"\x06" + message.encode(), response=True)
# #     print(f"Sent: {message.strip()} cm")
# #
# # async def main():
# #     client = await connect_to_spike()
# #
# #     picam2 = Picamera2()
# #     config = picam2.create_preview_configuration(main={"size": (640, 480)})
# #     picam2.configure(config)
# #     picam2.start()
# #     time.sleep(1)
# #
# #     last_sent_time = 0
# #
# #     try:
# #         while True:
# #             frame = picam2.capture_array()
# #             frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
# #             frame = cv2.flip(frame, -1)
# #             blurred = cv2.GaussianBlur(frame, (11, 11), 0)
# #             hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
# #
# #             mask = cv2.inRange(hsv, orange_lower, orange_upper)
# #             mask = cv2.erode(mask, None, iterations=2)
# #             mask = cv2.dilate(mask, None, iterations=2)
# #
# #             cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# #             cnts = cnts[0] if len(cnts) == 2 else cnts[1]
# #
# #             for c in cnts:
# #                 ((x, y), radius) = cv2.minEnclosingCircle(c)
# #                 M = cv2.moments(c)
# #                 if M["m00"] > 0 and radius > 10:
# #                     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
# #                     distance_cm = (FOCAL_LENGTH_PIXELS * REAL_DIAMETER_CM) / (2 * radius)
# #
# #                     cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
# #                     cv2.circle(frame, center, 5, (0, 0, 255), -1)
# #                     cv2.putText(frame, f"{distance_cm:.1f} cm", (center[0] + 10, center[1] - 10),
# #                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
# #
# #                     if distance_cm <= STOP_DISTANCE_CM:
# #                         print(f"Distance {distance_cm:.2f} cm: sending lift command.")
# #                         await ready_event.wait()
# #                         ready_event.clear()
# #                         await client.write_gatt_char(CHAR_UUID, b"\x06lift\n", response=True)
# #                         await client.disconnect()
# #                         picam2.close()
# #                         cv2.destroyAllWindows()
# #                         return
# #
# #                     if last_sent_time == 0:
# #                         pass
# #                     else:
# #                         if time.time() - last_sent_time >= DETECTION_INTERVAL:
# #                             await send_distance(client, distance_cm)
# #                             last_sent_time = time.time()
# #
# #                     break
# #
# #             cv2.imshow("Ball Detection", frame)
# #             if cv2.waitKey(1) & 0xFF == ord('q'):
# #                 break
# #
# #     finally:
# #         await client.disconnect()
# #         picam2.close()
# #         cv2.destroyAllWindows()
# #
# # if __name__ == "__main__":
# #     with suppress(asyncio.CancelledError):
# #         asyncio.run(main())
#
# import asyncio
# import cv2
# import numpy as np
# import time
# from contextlib import suppress
# from picamera2 import Picamera2
# from bleak import BleakScanner, BleakClient
#
# # BLE constants
# HUB_NAME = "Phoenix"
# CHAR_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"
#
# # Ball and camera constants
# REAL_DIAMETER_CM = 4.0
# FOCAL_LENGTH_PIXELS = 494
# STOP_DISTANCE_CM = 5.0
# DETECTION_INTERVAL = 3  # seconds
#
# # HSV orange range
# orange_lower = np.array([60, 80, 200], dtype="uint8")
# orange_upper = np.array([120, 255, 255], dtype="uint8")
#
# ready_event = asyncio.Event()
#
# async def connect_to_spike():
#     device = await BleakScanner.find_device_by_name(HUB_NAME)
#     if not device:
#         raise Exception(f"Could not find hub: {HUB_NAME}")
#     client = BleakClient(device)
#     await client.connect()
#     print("Connected to hub.")
#     return client
#
# async def send_command(client, message: str):
#     await ready_event.wait()
#     ready_event.clear()
#     await client.write_gatt_char(CHAR_UUID, b"\x06" + message.encode() + b"\n", response=True)
#     print(f"Sent: {message}")
#
# async def main():
#     client = await connect_to_spike()
#
#     def handle_rx(_, data: bytearray):
#         if data[0] == 0x01:
#             payload = data[1:]
#             if payload == b"rdy":
#                 ready_event.set()
#
#     await client.start_notify(CHAR_UUID, handle_rx)
#
#     picam2 = Picamera2()
#     config = picam2.create_preview_configuration(main={"size": (640, 480)})
#     picam2.configure(config)
#     picam2.start()
#     time.sleep(1)
#
#     last_sent_time = 0
#     last_no_sent_time = 0
#
#     try:
#         while True:
#             frame = picam2.capture_array()
#             frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
#             frame = cv2.flip(frame, -1)
#             blurred = cv2.GaussianBlur(frame, (11, 11), 0)
#             hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
#
#             mask = cv2.inRange(hsv, orange_lower, orange_upper)
#             mask = cv2.erode(mask, None, iterations=2)
#             mask = cv2.dilate(mask, None, iterations=2)
#
#             cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             cnts = cnts[0] if len(cnts) == 2 else cnts[1]
#
#             ball_found = False
#
#             for c in cnts:
#                 ((x, y), radius) = cv2.minEnclosingCircle(c)
#                 M = cv2.moments(c)
#
#                 if M["m00"] > 0 and radius > 10:
#                     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
#                     distance_cm = (FOCAL_LENGTH_PIXELS * REAL_DIAMETER_CM) / (2 * radius)
#
#                     cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
#                     cv2.circle(frame, center, 5, (0, 0, 255), -1)
#                     cv2.putText(frame, f"{distance_cm:.1f} cm", (center[0] + 10, center[1] - 10),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#
#                     ball_found = True
#
#                     if time.time() - last_sent_time >= DETECTION_INTERVAL:
#                         if distance_cm <= STOP_DISTANCE_CM:
#                             await send_command(client, "lift")
#                             return  # Exit after sending lift
#                         else:
#                             await send_command(client, f"{distance_cm:.2f}")
#                             last_sent_time = time.time()
#                     break
#
#             if not ball_found:
#                 if time.time() - last_no_sent_time >= DETECTION_INTERVAL:
#                     await send_command(client, "no")
#                     last_no_sent_time = time.time()
#
#             cv2.imshow("Ball Detection", frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#
#     finally:
#         await client.disconnect()
#         picam2.close()
#         cv2.destroyAllWindows()
#
# if __name__ == "__main__":
#     with suppress(asyncio.CancelledError):
#         asyncio.run(main())
