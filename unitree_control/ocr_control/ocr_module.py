import cv2
import numpy as np
import pytesseract

from unitree_control.core.control_modules import DogModule


class OCRModule(DogModule):
    """Handles optical character recognition"""

    def __init__(self):
        super().__init__("OCR")

        self.initialize()

    def initialize(self) -> None:
        if self._initialized:
            return

        self._initialized = True

    # followed this exactly: https://medium.com/@EnginDenizTangut/from-image-to-voice-building-an-ocr-tts-app-with-python-opencv-tesseract-5f5db8ea3b7b
    def extract_text_from_image(self, image: np.ndarray) -> tuple[str, np.ndarray]:
        """Extract text from an image"""
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.medianBlur(image, 3) # kernal to remove noise

        thresh = cv2.adaptiveThreshold(
            image, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 10
        )

        kernel = np.ones((2,2), np.uint8)
        image = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        image = cv2.GaussianBlur(image, (5, 5), 0)
        _, image = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        custom_config = r'--oem 3 --psm 6'
        text = pytesseract.image_to_string(image, lang='eng', config=custom_config)
        image = self._highlight_detected_result(image)

        return (text, image)
    

    def _highlight_detected_result(self, image: np.ndarray):
        ocr_data = pytesseract.image_to_data(image, output_type=pytesseract.Output.DICT)

        for i in range(len(ocr_data['text'])):
            if int(ocr_data['conf'][i]) > 60:
                x, y, w, h = (ocr_data["left"][i], ocr_data['top'][i], ocr_data['width'][i], ocr_data['height'][i])
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        return image 


    def shutdown(self) -> None:
        self._initialized = False
