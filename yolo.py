import os
import cv2
import time
import json
import paho.mqtt.client as mqtt
from ultralytics import YOLO

# MQTT Configuration - Local Mosquitto broker (no TLS, no auth)
print("=== MQTT Configuration ===")
MQTT_BROKER = input("Enter MQTT broker address (default: localhost): ").strip() or "localhost"
MQTT_PORT = 1883  # Native MQTT protocol (WebSocket on 9001 for browser clients)

# MQTT Topics
TOPIC_LED_CONTROL = "esp32/led/control"
TOPIC_BUZZER_CONTROL = "esp32/buzzer/control"
TOPIC_LED_STATE = "esp32/led/state"
TOPIC_BUZZER_STATE = "esp32/buzzer/state"
TOPIC_AI_STATUS = "ai/detection/status"  # AI detection status
TOPIC_AI_CONFIDENCE = "ai/detection/confidence"  # AI confidence level
TOPIC_AI_CONTROL = "ai/control"  # AI control toggle (ON/OFF)
TOPIC_AI_THRESHOLD = "ai/threshold"  # Confidence threshold (0-100)

# Global variables for AI control
ai_control_enabled = False
confidence_threshold = 0.5  # Default 50%

# Create MQTT client (no TLS, no authentication for local broker)
mqtt_client = mqtt.Client()

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"✓ Connected to Mosquitto broker ({MQTT_BROKER}:{MQTT_PORT}) successfully!")
        client.subscribe(TOPIC_LED_STATE)
        client.subscribe(TOPIC_BUZZER_STATE)
        client.subscribe(TOPIC_AI_CONTROL)
        client.subscribe(TOPIC_AI_THRESHOLD)
        print("✓ Subscribed to LED, Buzzer state, AI control, and threshold topics")
    else:
        print(f"✗ Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    global ai_control_enabled, confidence_threshold
    
    topic = msg.topic
    payload = msg.payload.decode()
    
    print(f"MQTT: {topic} = {payload}")
    
    # Handle AI control toggle
    if topic == TOPIC_AI_CONTROL:
        ai_control_enabled = (payload.upper() == "ON" or payload == "1")
        print(f"AI Control: {'ENABLED' if ai_control_enabled else 'DISABLED'}")
    
    # Handle confidence threshold
    elif topic == TOPIC_AI_THRESHOLD:
        try:
            threshold = float(payload) / 100.0  # Convert percentage to decimal (0-1)
            confidence_threshold = max(0.0, min(1.0, threshold))  # Clamp between 0 and 1
            print(f"Confidence Threshold: {confidence_threshold * 100:.1f}%")
        except ValueError:
            print(f"Invalid threshold value: {payload}")

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# Connect to MQTT broker
print(f"\nConnecting to {MQTT_BROKER}:{MQTT_PORT}...")
try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
    time.sleep(2)  # Wait for connection
except Exception as e:
    print(f"MQTT connection failed: {e}")
    exit(1)

# Load YOLOv8 model
model = YOLO('yolov8n.pt')

# List COCO classes and prompt the user to select one
print("\nAvailable COCO dataset classes:")
for class_id, class_name in model.names.items():
    print(f"{class_id}: {class_name}")

while True:
    try:
        selected_class_id = int(input("\nEnter the class ID you want to track: "))
        if selected_class_id in model.names:
            selected_class_name = model.names[selected_class_id]
            print(f"Tracking class: {selected_class_name}")
            break
        else:
            print("Invalid class ID. Please enter a valid ID from the list above.")
    except ValueError:
        print("Invalid input. Please enter a numerical class ID.")

# Video capture configuration
print("\n=== Video Configuration ===")
CAP_WIDTH = int(input("Enter capture width (default 1280): ").strip() or "1280")
CAP_HEIGHT = int(input("Enter capture height (default 720): ").strip() or "720")

# Open webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)

print(f"\n✓ Video capture initialized at {CAP_WIDTH}x{CAP_HEIGHT}")

object_detected = False
detection_cooldown = 5  # Number of consecutive frames for consistent detection
detection_counter = 0  # Counter to track consistent detections
current_confidence = 0.0

# FPS calculation variables
fps_start_time = time.time()
fps_frame_count = 0
fps = 0

print("\n=== Starting Detection ===")
print("Press 'q' to quit\n")

try:
    while cap.isOpened():
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            break

        # Calculate FPS
        fps_frame_count += 1
        if fps_frame_count >= 30:  # Update FPS every 30 frames
            fps_end_time = time.time()
            fps = fps_frame_count / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
            fps_frame_count = 0

        # Run YOLOv8 object detection
        results = model(frame, verbose=False)  # Disable verbose to speed up

        # Check if the selected class is detected and get max confidence
        detected = False
        max_confidence = 0.0
        
        for box in results[0].boxes:
            if int(box.cls) == selected_class_id:
                detected = True
                conf = float(box.conf)
                max_confidence = max(max_confidence, conf)
        
        current_confidence = max_confidence

        # Publish AI detection status and confidence to MQTT
        status = "DETECTED" if detected else "NOT_DETECTED"
        mqtt_client.publish(TOPIC_AI_STATUS, status)
        mqtt_client.publish(TOPIC_AI_CONFIDENCE, f"{current_confidence * 100:.2f}")

        # Update detection logic with cooldown mechanism
        if detected and current_confidence >= confidence_threshold:
            detection_counter += 1
        else:
            detection_counter -= 1

        # Clamp the counter within the cooldown range
        detection_counter = max(0, min(detection_cooldown, detection_counter))

        # Only control LED and buzzer if AI control is enabled
        if ai_control_enabled:
            # Trigger LED and Buzzer ON if the object is consistently detected above threshold
            if detection_counter == detection_cooldown and not object_detected:
                print(f"✓ {selected_class_name} detected (confidence: {current_confidence*100:.1f}%), turning LED and Buzzer ON.")
                mqtt_client.publish(TOPIC_LED_CONTROL, "ON")
                mqtt_client.publish(TOPIC_BUZZER_CONTROL, "ON")
                object_detected = True

            # Trigger LED and Buzzer OFF if the object is consistently not detected or below threshold
            elif detection_counter == 0 and object_detected:
                reason = "not detected" if not detected else f"confidence ({current_confidence*100:.1f}%) below threshold ({confidence_threshold*100:.1f}%)"
                print(f"✗ {selected_class_name} {reason}, turning LED and Buzzer OFF.")
                mqtt_client.publish(TOPIC_LED_CONTROL, "OFF")
                mqtt_client.publish(TOPIC_BUZZER_CONTROL, "OFF")
                object_detected = False

        # Display detections on the frame
        frame = results[0].plot()  # YOLOv8 method to plot detections on the frame

        # Draw FPS on top left corner
        fps_text = f"FPS: {fps:.1f}"
        cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (0, 255, 0), 2, cv2.LINE_AA)
        
        # Draw AI status and confidence on frame
        status_text = f"AI Control: {'ON' if ai_control_enabled else 'OFF'}"
        cv2.putText(frame, status_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (255, 255, 255), 2, cv2.LINE_AA)
        
        if detected:
            conf_text = f"Confidence: {current_confidence*100:.1f}% (Threshold: {confidence_threshold*100:.1f}%)"
            color = (0, 255, 0) if current_confidence >= confidence_threshold else (0, 165, 255)
            cv2.putText(frame, conf_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, color, 2, cv2.LINE_AA)

        # Show the frame
        cv2.imshow('YOLOv8 Detection', frame)

        # Break on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n\nKeyboardInterrupt detected. Exiting gracefully...")
    # Optionally, send an LED OFF signal before exiting
    if object_detected and ai_control_enabled:
        mqtt_client.publish(TOPIC_LED_CONTROL, "OFF")
        print("✓ LED turned OFF.")
        mqtt_client.publish(TOPIC_BUZZER_CONTROL, "OFF")
        print("✓ Buzzer turned OFF.")

finally:
    # Disconnect from MQTT
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("✓ Disconnected from MQTT broker")
    
    # Release the webcam and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    print("✓ Resources released. Goodbye!")
