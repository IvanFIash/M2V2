import cv2
import numpy as np
from pi5neo import Pi5Neo

# Initialize the Pi5Neo class with 10 LEDs and an SPI speed of 800kHz
neo = Pi5Neo('/dev/spidev0.0', 10, 800)

def threshold_rel(img, lo, hi):
    vmin = np.min(img)
    vmax = np.max(img)

    vlo = vmin + (vmax - vmin) * lo
    vhi = vmin + (vmax - vmin) * hi
    return np.uint8((img >= vlo) & (img <= vhi)) * 255

def threshold_abs(img, lo, hi):
    return np.uint8((img >= lo) & (img <= hi)) * 255

def hist(img):
    bottom_half = img[img.shape[0]//2:,:]
    return np.sum(bottom_half, axis=0)

def pixels_in_window(center, margin, height, nonzerox, nonzeroy):
    topleft = (center[0]-margin, center[1]-height//2)
    bottomright = (center[0]+margin, center[1]+height//2)

    condx = (topleft[0] <= nonzerox) & (nonzerox <= bottomright[0])
    condy = (topleft[1] <= nonzeroy) & (nonzeroy <= bottomright[1])
    return nonzerox[condx&condy], nonzeroy[condx&condy]

def find_lane_pixels(img, window_height, nwindows, nonzerox, nonzeroy, minpix):
    assert(len(img.shape) == 2)

    # Create an output image to draw on and visualize the result
    out_img = np.dstack((img, img, img))

    histogram = hist(img)
    midpoint = histogram.shape[0]//2
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Current position to be update later for each window in nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base
    y_current = img.shape[0] + window_height//2

    # Create empty lists to reveice left and right lane pixel
    leftx, lefty, rightx, righty = [], [], [], []

    # Step through the windows one by one
    for _ in range(nwindows):
        y_current -= window_height
        center_left = (leftx_current, y_current)
        center_right = (rightx_current, y_current)

        good_left_x, good_left_y = pixels_in_window(center_left, margin, window_height, nonzerox, nonzeroy)
        good_right_x, good_right_y = pixels_in_window(center_right, margin, window_height, nonzerox, nonzeroy)

        # Append these indices to the lists
        leftx.extend(good_left_x)
        lefty.extend(good_left_y)
        rightx.extend(good_right_x)
        righty.extend(good_right_y)

        if len(good_left_x) > minpix:
            leftx_current = np.int32(np.mean(good_left_x))
        if len(good_right_x) > minpix:
            rightx_current = np.int32(np.mean(good_right_x))

    return leftx, lefty, rightx, righty, out_img

def measure_curvature(left_f, right_f):
    ym = 30/720
    xm = 3.7/700

    left_fit = left_f.copy()
    right_fit = right_f.copy()
    y_eval = 700 * ym

    # Compute R_curve (radius of curvature)
    left_curveR =  ((1 + (2*left_fit[0] *y_eval + left_fit[1])**2)**1.5)  / np.absolute(2*left_fit[0])
    right_curveR = ((1 + (2*right_fit[0]*y_eval + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])

    xl = np.dot(left_fit, [700**2, 700, 1])
    xr = np.dot(right_fit, [700**2, 700, 1])
    pos = (1280//2 - (xl+xr)//2)*xm
    return left_curveR, right_curveR, pos

def calcular_buffer_led(pos):
    """
    Calcula el buffer de LED con base en la posición relativa `pos`.
    :param pos: Desplazamiento con respecto al centro (-8 a 8).
    :return: Lista de 10 valores (0 o 1) indicando cuáles LEDs encender.
    """
    # Inicializar buffer con ceros
    buffer = ['0'] * 10
    centro = 5  # Índice central del buffer (LED 5)

    if pos < -8 or pos > 8:
        # Si `pos` está fuera de rango, encender todos los LEDs
        buffer = ['1'] * 10
    else:
        # Convertir `pos` (-8 a 8) a un índice de LED relativo
        indice = int(round((pos / 8) * 4))  # Mapea de -8 a 8 a -4 a 4
        if indice < 0:
            buffer[centro + indice] = '1'  # Encender LED a la izquierda
        elif indice > 0:
            buffer[centro + indice] = '1'  # Encender LED a la derecha
        else:
            buffer[centro] = '1'  # Encender el LED central

    return buffer

def encender_leds(buffer):
    """
    Enciende los LEDs según el buffer generado.
    :param buffer: Lista de 10 valores (0 o 1).
    """
    for i, estado in enumerate(buffer):
        if estado == '1':
            neo.set_led_color(i, 255, 255, 255, 255)  # Blanco máximo (RGBW)
        else:
            neo.set_led_color(i, 0, 0, 0, 0)  # Apagado

    neo.update_strip()  # Actualizar la tira de LEDs

nwindows = 9
margin = 100
minpix = 50
left_fit = np.zeros(3)
right_fit = np.zeros(3)

# Inicializar la webcam
cap = cv2.VideoCapture(2)  # 0 es el ID de la cámara predeterminada

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

src = np.float32([(155, 0),     # top-left
                  (0, 720),     # bottom-left
                  (1175, 720),    # bottom-right
                  (980, 0)])    # top-right

dst = np.float32([(100, 0),
                  (100, 720),
                  (1100, 720),
                  (1100, 0)])

M = cv2.getPerspectiveTransform(src, dst)

per_img = cv2.warpPerspective(frame, M, (1280, 720), flags=cv2.INTER_LINEAR)

cv2.imwrite('per.jpg', per_img)

hls = cv2.cvtColor(per_img, cv2.COLOR_RGB2HLS)
hsv = cv2.cvtColor(per_img, cv2.COLOR_RGB2HSV)
h_channel = hls[:,:,0]
l_channel = hls[:,:,1]
s_channel = hls[:,:,2]
v_channel = hsv[:,:,2]

right_lane = threshold_rel(l_channel, 0.5, 1.0)
right_lane[:,:750] = 0

left_lane = threshold_abs(h_channel, 10, 30)
left_lane &= threshold_rel(v_channel, 0.4, 1.0)
left_lane[:,550:] = 0

rel_img = left_lane | right_lane

cv2.imwrite('rel.jpg', rel_img)

# Height of of windows - based on nwindows and image shape
window_height = int(rel_img.shape[0]//nwindows)

# Identify the x and y positions of all nonzero pixel in the image
nonzero = rel_img.nonzero()
nonzerox = np.array(nonzero[1])
nonzeroy = np.array(nonzero[0])

leftx, lefty, rightx, righty, out_img = find_lane_pixels(rel_img, window_height, nwindows, nonzerox, nonzeroy, minpix)

if len(lefty) > 1500:
    left_fit = np.polyfit(lefty, leftx, 2)
if len(righty) > 1500:
    right_fit = np.polyfit(righty, rightx, 2)

# Generate x and y values for plotting
maxy = rel_img.shape[0] - 1
miny = rel_img.shape[0] // 3
if len(lefty):
    maxy = max(maxy, np.max(lefty))
    miny = min(miny, np.min(lefty))

if len(righty):
    maxy = max(maxy, np.max(righty))
    miny = min(miny, np.min(righty))

ploty = np.linspace(miny, maxy, rel_img.shape[0])

left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

# Visualization
for i, y in enumerate(ploty):
    l = int(left_fitx[i])
    r = int(right_fitx[i])
    y = int(y)
    cv2.line(out_img, (l, y), (r, y), (0, 255, 0))

lR, rR, pos = measure_curvature(left_fit, right_fit)

cv2.imwrite('out.jpg', out_img)
print(pos)

buffer = calcular_buffer_led(pos)
print(f"Buffer generado: {''.join(buffer)}")  # Verificar buffer
encender_leds(buffer)

