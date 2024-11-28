import board
import neopixel
import sys

# Recibir los argumentos de la línea de comandos
arr = sys.argv

# Configuración de NeoPixel (pines y cantidad de LEDs)
pixels = neopixel.NeoPixel(board.D18, 10, brightness=0.5, pixel_order=neopixel.GRBW)

print(arr)

for i in range(len(arr)):
    if arr[i] == 1:
        pixels[i] = (0, 0, 0, 255)
    else:
        pixels[i] = (0, 0, 0, 0)
