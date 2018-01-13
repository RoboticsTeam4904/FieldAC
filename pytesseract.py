# import the necessary packages
from PIL import Image
import pytesseract
# import argparse
# import cv2
# import os
 
import cv2
folder = "TestImages"
image_name = "254_blur"
file_type = "png"
img = cv2.imread(folder + "/" + image_name + "." + file_type)

tessdata_dir_config = '--tessdata-dir "<replace_with_your_tessdata_dir_path>"'
# print(pytesseract.image_to_string(img))
# OR explicit beforehand converting
print(pytesseract.image_to_string(Image.fromarray(img)))

# # load the example image and convert it to grayscale
# image = cv2.imread(folder + "/" + image_name + "." + file_type)
# # img = copy.deepcopy(image)
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
 
# # check to see if we should apply thresholding to preprocess the
# # image
# # if args["preprocess"] == "thresh":
# gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
 
# # make a check to see if median blurring should be done to remove
# # noise
# # elif args["preprocess"] == "blur":
# # print type(gray), gray.shape
# # blur = cv2.medianBlur(gray, 2)
 
# # write the grayscale image to disk as a temporary file so we can
# # apply OCR to it
# # cv2.imshow("Image", image)
# # cv2.waitKey(0)
# filename = folder + "/" + image_name + "_processed.png"
# cv2.imwrite(filename, gray)

# # time.sleep(10)

# # load the image as a PIL/Pillow image, apply OCR, and then delete
# # the temporary file
# Img = Image.open(filename)
# text = pytesseract.image_to_string(Img)
# os.remove(filename)
# # print(text)
 
# # show the output images
# cv2.imshow("Image", image)
# cv2.imshow("Output", img)
# cv2.waitKey(0)
