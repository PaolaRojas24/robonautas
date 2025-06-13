import cv2
import numpy as np

cap = cv2.VideoCapture('/home/paoro/Documents/Clases/TE3002B/seguidor/parte2.mp4')
if not cap.isOpened():
    raise IOError("No se pudo abrir el video")

# Crear ventana para sliders
cv2.namedWindow("Calibrador Escala de Grises")

def nothing(x):
    pass

# Slider para el umbral
cv2.createTrackbar("Umbral", "Calibrador Escala de Grises", 60, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        continue

    #frame = cv2.resize(frame, (640, 480))

    # ROI solo parte superior
    roi = frame[0:550, :]

    # Conversión a escala de grises
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Leer valor del slider
    umbral = cv2.getTrackbarPos("Umbral", "Calibrador Escala de Grises")

    # Binarización inversa
    _, binary = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY_INV)

    # Mostrar resultados
    cv2.imshow("ROI original", roi)
    cv2.imshow("Grises", gray)
    cv2.imshow("Binarizada (INV)", binary)

    key = cv2.waitKey(150)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()