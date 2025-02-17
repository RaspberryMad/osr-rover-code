import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO
import os
from datetime import datetime


nombre_foto = "IMG_6804"
# Cargar el modelo entrenado
model = YOLO("./modelos/modelo_ConNumeros/weights/best.pt")  # Ajusta la ruta si es necesario

# Ruta de la imagen a analizar
image_path = f"./dataset/cubos/{nombre_foto}.jpg"  # Cambia esto con tu imagen real
output_dir = "./dataset/cubos/resultados"

# Crear una carpeta con marca de tiempo para guardar los resultados
folder_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
os.makedirs(f"{output_dir}", exist_ok=True)

# Ejecutar la detección y guardar resultados
results = model.predict(source=image_path, save=True, conf=0.5, project=output_dir, name=folder_name)

# Cargar la imagen original
original_img = cv2.imread(image_path)

# Cargar la imagen con detecciones (generada por YOLO)
result_img_path = f"{output_dir}/{folder_name}/{nombre_foto}.jpg"  # YOLO suele guardarla con este nombre
result_img = cv2.imread(result_img_path)

# Convertir de BGR a RGB para Matplotlib
original_img_rgb = cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB)
result_img_rgb = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)

# Recortar el primer cubo detectado (si hay detecciones)
for result in results:
    if len(result.boxes) > 0:
        x1, y1, x2, y2 = map(int, result.boxes.xyxy[0])  # Coordenadas del primer objeto detectado
        cropped_cube = original_img[y1:y2, x1:x2]
        cropped_cube_path = f"{output_dir}/{folder_name}/cubo_recortado.jpg"
        cv2.imwrite(cropped_cube_path, cropped_cube)
        print(f"Cubo recortado guardado en: {cropped_cube_path}")

# Archivo de salida para guardar los textos detectados
detections_txt_path = f"{output_dir}/{folder_name}/detecciones.txt"

with open(detections_txt_path, "w") as f:
    for result in results:
        if len(result.boxes) > 0:
            for i, box in enumerate(result.boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Coordenadas del cuadro de detección
                label = result.names[int(box.cls[0])]  # Nombre de la clase detectada
                conf = float(box.conf[0])  # Confianza del modelo

                # Guardar en el archivo
                f.write(f"Detección {i+1}: {label} (Confianza: {conf:.2f}) Coordenadas: ({x1}, {y1}, {x2}, {y2})\n")        

# Crear una imagen comparativa (original vs detección)
fig, axes = plt.subplots(1, 2, figsize=(12, 6))
axes[0].imshow(original_img_rgb)
axes[0].set_title("Imagen Original")
axes[0].axis('off')

axes[1].imshow(result_img_rgb)
axes[1].set_title("Detección de Cubos")
axes[1].axis('off')

# Guardar la comparación
comparison_path = f"{output_dir}/{folder_name}/comparacion.png"
plt.savefig(comparison_path)
print(f"Imagen comparativa guardada en: {comparison_path}")

