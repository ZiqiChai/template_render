import glob
import os
for bmp_filename in glob.glob('PVC_3hole_40mm/*.bmp'):
    basename = bmp_filename.split('.')[0]
    dpt_filename = basename + ".dpt"
    png_filename = basename + ".png"
    # print (bmp_filename)
    # print (dpt_filename)
    # print (png_filename)
    if not os.path.exists(dpt_filename):
        print (dpt_filename+" not exist")
    if not os.path.exists(png_filename):
        print (png_filename+" not exist")
