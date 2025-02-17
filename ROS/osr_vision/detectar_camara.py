import cv2
from ultralytics import YOLO
import os
from datetime import datetime

# Cargar el modelo entrenado
model = YOLO("./modelos/modelo_ConNumeros/weights/best.pt")  # Ajusta la ruta si es necesario

# Ejecutar el seguimiento con el modelo YOLO
results = model.track(source=0, tracker='bytetrack.yaml', show=True)


