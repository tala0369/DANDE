#IMAGE RESIZE ALGORITHM
import os
from PIL import Image

image_file = "test_image2.jpg"
#os.startfile(image_file)
im = Image.open(image_file)
[w,h] = im.size
print im.size

AR = w/float(h)
[wh,hh] = [float(h/2),w]
ARH = wh/float(hh)

if ARH>AR: #then crop the L/R edges - will lose data at centerline
    new_width = int(AR*hh)
    offset = (wh-new_width)/2
    resize1 = (0,offset,w,h/2-offset)
    resize2 = (0,h/2+offset,w,h-offset)
else: #then crop the top/bottom edges
    new_height = int(wh/AR)
    offset = (hh-new_height)/2
    resize1 = (offset,0,w-offset,h/2)
    resize2 = (offset,h/2,w-offset,h)

half1 = im.transform((h,w),Image.EXTENT,resize1, Image.BICUBIC)
half2 = im.transform((h,w),Image.EXTENT,resize2, Image.BICUBIC)

half1 = half1.transpose(Image.ROTATE_90).save("test_image2_half1.jpg","JPEG")
half2 = half2.transpose(Image.ROTATE_90).save("test_image2_half2.jpg","JPEG")

#os.startfile("test_image2_half1.jpg")
#os.startfile("test_image2_half2.jpg")