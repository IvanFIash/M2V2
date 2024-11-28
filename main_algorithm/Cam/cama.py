from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()

# Configure the camera
picam2.configure(picam2.create_still_configuration())

# Start the camera and capture an image
picam2.start()
picam2.capture_file("photo.jpg")
picam2.stop()

print("Photo saved as photo.jpg")
