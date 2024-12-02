import cv2
import numpy as np

# Inicializar la webcam
cap = cv2.VideoCapture(0)  # 0 es el ID de la cámara predeterminada

ancho = 1280  # Cambia esto por el ancho deseado
alto = 720    # Cambia esto por la altura deseada
cap.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)

if not cap.isOpened():
    print("No se pudo acceder a la cámara.")
    exit()

ret, frame = cap.read()
if not ret:
    print("No se pudo leer el frame de la cámara.")
else:
    cv2.imwrite('captura.jpg', frame)

cap.release()
