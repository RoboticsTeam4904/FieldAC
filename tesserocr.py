from tesserocr import PyTessBaseAPI

images = ['TestImages/254_blur.jpg']#, 'sample2.jpg', 'sample3.jpg']

with PyTessBaseAPI() as api:
    for img in images:
        api.SetImageFile(img)
        print api.GetUTF8Text()
        print api.AllWordConfidences()
# api is automatically finalized when used in a with-statement (context manager).
# otherwise api.End() should be explicitly called when it's no longer needed.

import tesserocr
from PIL import Image

print tesserocr.tesseract_version()  # print tesseract-ocr version
print tesserocr.get_languages()  # prints tessdata path and list of available languages

image = Image.open('sample.jpg')
print tesserocr.image_to_text(image)  # print ocr text from image
# or
print tesserocr.file_to_text('sample.jpg')