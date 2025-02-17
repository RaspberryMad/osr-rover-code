from ultralytics import YOLO
import torch

# Verificar si hay GPU disponible
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Usando dispositivo: {device}")

# Ruta del archivo data.yaml (ajústala según sea necesario)
data_path = "./roboflow/SL/Perception.v1i.yolov8/data.yaml"

# Cargar el modelo YOLOv8 preentrenado
model = YOLO("yolov8n.pt")  # Puedes cambiarlo por yolov8s.pt o yolov8m.pt según tu hardware

# Configurar el entrenamiento
model.train(
    data=data_path,      # Dataset con imágenes anotadas
    epochs=100,          # Número de épocas (ajústalo según necesidad)
    imgsz=640,           # Tamaño de imagen
    batch=8,             # Tamaño de batch (ajústalo según la RAM de la GPU/CPU)
    device=device,       # Entrenar en GPU si está disponible
    workers=4,           # Número de procesos para cargar datos (ajústalo según hardware)
    name="train",        # Carpeta donde se guardará el modelo
    project="modelos",  # Directorio base para guardar resultados
    save=True,           # Guardar el mejor modelo
    patience=10          # Detener el entrenamiento si no mejora en 10 épocas
)

# Mensaje final
print("Entrenamiento completado. El mejor modelo está en:")
print("runs/detect/train/weights/best.pt")
    