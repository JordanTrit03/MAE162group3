# import useful libraries
import cv2
import subprocess
import os
from yolo_utils import *
from picamera2 import Picamera2

# video file names
temp_video = "temp_recording.avi"
output_video = "recording.mp4"

# check OpenCV + CUDA
print("OpenCV version :", cv2.__version__)
print("Available CUDA devices:", cv2.cuda.getCudaEnabledDeviceCount(), "\n")

# load class names
obj_file = './obj.names'
classNames = read_classes(obj_file)
print("Classes' names :", classNames, "\n")

# load YOLO model
modelConfig_path = './cfg/yolov4.cfg'
modelWeights_path = './weights/yolov4.weights'

neural_net = cv2.dnn.readNetFromDarknet(modelConfig_path, modelWeights_path)
neural_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
neural_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

confidenceThreshold = 0.5
nmsThreshold = 0.1

network = neural_net
height, width = 128, 128   # input size for network

# initialize Pi Camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={'size': (640, 480)}))
picam2.start()

# setup Video Writer (AVI first)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(temp_video, fourcc, 30.0, (640, 480))

print("[MAIN] Recording started... Press Ctrl+C to stop.")

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # object detection
        outputs = convert_to_blob(frame, network, height, width)
        bounding_boxes, class_objects, confidence_probs = object_detection(
            outputs, frame, confidenceThreshold)

        for i in range(len(bounding_boxes)):
            print(f"[Debug] Detected: Class={class_objects[i]}, Confidence={confidence_probs[i]:.2f}")
            # TODO: change the class number to the class number of traffic light in obj.names file
            if class_objects[i] == 3:
                # TODO: detect the color of the traffic light (red) by merging task 1
                # step 1: crop the bounding box area from the frame
                x, y, w, h = bounding_boxes[i]
                x, y, w, h = int(x), int(y), int(w), int(h)

                # Clamp to frame boundaries
                x = max(0, x)
                y = max(0, y)
                w = min(frame.shape[1] - x, w)
                h = min(frame.shape[0] - y, h)

                cropped = frame[y:y+h, x:x+w]

                # step 2: convert the cropped area to HSV color space
                hsv_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
                # step 3: create a mask for red color
                lower_red = np.array([0, 120, 70])
                upper_red = np.array([10, 255, 255])
                # step 4: check if there are enough contour areas in the mask to confirm the traffic light is red
                mask = cv2.inRange(hsv_cropped, lower_red, upper_red)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                red_detected = False
                # step 5: print a message if the traffic light is red (e.g., "Red light detected!")
                for contour in contours:
                    if cv2.contourArea(contour) > 300:  
                        red_detected = True
                        break
                if red_detected:
                    print("🚦 Red light detected!")
                    cv2.putText(frame, "RED LIGHT",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2)
        

        indices = nms_bbox(
            bounding_boxes,
            confidence_probs,
            confidenceThreshold,
            nmsThreshold
        )

        box_drawing(
            frame,
            indices,
            bounding_boxes,
            class_objects,
            confidence_probs,
            classNames,
            color=(0, 255, 255),
            thickness=2
        )

        # write frame to video file
        out.write(frame)

except KeyboardInterrupt:
    print("\n[MAIN] Stopping recording...")

# cleanup
out.release()
picam2.close()

print("[MAIN] Converting to MP4 using ffmpeg...")

subprocess.run(["ffmpeg", "-y", "-i", temp_video, "-vcodec", "libx264", "-preset", "ultrafast", "-crf", "23", "-pix_fmt", "yuv420p", output_video], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
os.remove(temp_video)

print("[MAIN] Video saved successfully as", output_video)
