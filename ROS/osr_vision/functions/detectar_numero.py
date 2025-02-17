import cv2
import pytesseract
import numpy as np

def preprocesar_imagen(imagen_cubo):
    """Aplica preprocesamiento para mejorar la detección del número."""
    img = cv2.imread(imagen_cubo, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None, "Error: No se pudo cargar la imagen."

    # Aumentar contraste
    img = cv2.equalizeHist(img)

    # Aplicar filtro Gaussiano para suavizar
    img = cv2.GaussianBlur(img, (5, 5), 0)

    # Umbral adaptativo
    _, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # Dilatar y erosionar para mejorar detección de bordes
    kernel = np.ones((3,3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    return thresh, None

def detectar_numero(imagen_cubo):
    """Detecta y corrige la orientación del número en un cubo."""
    thresh, error = preprocesar_imagen(imagen_cubo)
    if error:
        return error

    # Encontrar contornos
    contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contornos:
        return "NO SE HA DETECTADO NINGÚN NÚMERO"

    # Tomar el contorno más grande
    c = max(contornos, key=cv2.contourArea)

    # Calcular caja de rotación
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.int32(box)

    # Corregir rotación
    angle = rect[-1]
    if angle < -45:
        angle += 90  

    h, w = thresh.shape
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(thresh, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)

    # Extraer el área del número
    x, y, w, h = cv2.boundingRect(c)
    numero_recortado = rotated[y:y+h, x:x+w]

    # Aplicar OCR con mejor configuración
    custom_config = r'--oem 3 --psm 6 -c tessedit_char_whitelist=0123456789'
    resultado = pytesseract.image_to_string(numero_recortado, config=custom_config).strip()

    # Filtrar caracteres no numéricos
    numero = "".join(filter(str.isdigit, resultado))

    if numero:
        return f"Número detectado: {numero}"
    else:
        return "NO SE HA DETECTADO NINGÚN NÚMERO"

# Prueba rápida
if __name__ == "__main__":
    imagen_prueba = r"./dataset/cubos/resultados/2025-02-05_08-58-10/cubo_recortado.jpg"
    print(detectar_numero(imagen_prueba))
