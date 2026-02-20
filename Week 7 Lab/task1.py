from picamera2 import Picamera2
import time
picam2 = Picamera2()
config = picam2.create_still_configuration(
    main={"size": (2304, 1296)},
    buffer_count=2
)
picam2.configure(config)
picam2.start()
time.sleep(2) # let auto exposure settle
picam2.capture_file("task1.jpg")
picam2.stop()
print("Image saved successfully!")