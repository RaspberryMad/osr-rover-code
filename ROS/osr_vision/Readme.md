# Proyecto de Visión para el Rover

Este repositorio contiene el algoritmo de visión para el Rover, diseñado para la detección de objetos, números y colores utilizando YOLO y OpenCV.

---

## Descripción

El algoritmo utiliza:
- **YOLO**: Un modelo de detección de objetos en tiempo real.
- **OpenCV**: Librería para procesamiento de imágenes.

---

## Configuración del Entorno

Para configurar el entorno y ejecutar el proyecto correctamente, sigue los pasos a continuación:

### 1. Clonar el repositorio

    git clone <URL-del-repositorio>
    cd <nombre-del-repositorio>

### 2. Crear un entorno virtual

Crea un entorno virtual con el nombre .venv:

    python3 -m venv .venv

### 3. Activar el entorno virtual

Activa el entorno según tu sistema operativo:

- Linux/MacOS:

        source .venv/bin/activate

- Windows:

        .venv\Scripts\activate

### 4. Instalar las dependencias

Instala las dependencias necesarias usando el archivo requirements.txt:

    pip install -r requirements.txt

## Carpetas del Proyecto

 - **roboflow:** Contiene los datasets preparados en Roboflow, que se utilizan para entrenar el modelo de detección. 
 
 - **datasets:** Aquí se almacenan los resultados de las detecciones realizadas con imágenes. Cada vez que se ejecuta detectar.py, los resultados se guardan en esta carpeta.
 
 - **modelos:** En esta carpeta se guardan los modelos entrenados utilizando nuestro propio dataset. Los archivos aquí almacenados corresponden a los modelos YOLOv8 generados a partir de los entrenamientos realizados.

## Archivos del Proyecto

 - **entrenar.py:** Código para entrenar el modelo YOLOv8 con nuestro propio dataset. Este archivo se encarga de configurar y ejecutar el proceso de entrenamiento, generando un modelo ajustado a las necesidades del Rover.

 - **detectar.py:** Este archivo emplea el modelo entrenado para realizar detecciones sobre imágenes estáticas. Guarda los resultados de la detección en la carpeta dataset para su posterior análisis.

 - **detectar_camara.py:** Este archivo contiene el código que utiliza el modelo entrenado para realizar detecciones en tiempo real a través de la cámara. Muestra en pantalla lo que detecta, incluyendo objetos, números y colores.

