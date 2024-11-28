from picamera2 import Picamera2
import subprocess

def get_angle_from_accelerometer():
    result = subprocess.run(["./mpu6050-kalman"], capture_output=True, text=True)
    print(result)
    return float(result.stdout.strip())

# Inicializar la cámara
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration())

# Capturar la imagen y calcular el ángulo
picam2.start()
image = picam2.capture_file("photo.jpg")  # Capturar y guardar la foto
picam2.stop()

print("Foto tomada y guardada como 'photo.jpg'.")

# Calcular el ángulo usando el acelerómetro
angle = get_angle_from_accelerometer()
