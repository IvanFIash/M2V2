from picamera2 import Picamera2
import subprocess
import time

def get_angle_from_accelerometer():
    result = subprocess.run(["./mpu6050-kalman"], capture_output=True, text=True)
    return result.stdout.strip().split()

# Inicializar la cámara
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"size": (1280, 720)}))

for i in range(10):
    start = time.time()
    # Capturar la imagen y calcular el ángulo
    picam2.start()
    image = picam2.capture_file("photo.jpg")  # Capturar y guardar la foto
    picam2.stop()

    print("Foto tomada y guardada como 'photo.jpg'.")

    # Calcular el ángulo usando el acelerómetro
    angles = get_angle_from_accelerometer()

    print(f"Roll: {float(angles[0])}, Pitch: {float(angles[1])}")
    end = time.time()
    print(end - start)
