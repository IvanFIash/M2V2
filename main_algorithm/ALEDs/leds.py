import board
import neopixel

# Configuración de NeoPixel (pines y cantidad de LEDs)
pixels = neopixel.NeoPixel(board.D18, 10, brightness=0.5, pixel_order=neopixel.GRBW)

# Encender un LED con color rojo puro
pixels[0] = (255, 0, 0, 0)  # (R, G, B, W)

# Encender un LED con blanco cálido
pixels[1] = (0, 0, 255, 0)
