from __future__ import print_function, division
import os
import cv2

def main():
	max_height=0.0
	max_width=0.0
	number=0
	rootdir = './ico_template_m2'
	list = os.listdir(rootdir)
	for i in range(0,len(list)):
		path = os.path.join(rootdir,list[i])
		print("path:",path)
		if os.path.isfile(path):
			print("file_path:",path)
			if os.path.splitext(path)[1] == '.bmp':
				number=number+1
				print("bmp_file_path:",path)
				img=cv2.imread(path)
				cv2.imshow("img",img)
				size = img.shape
				print("height:",size[0]," width:",size[1]," channel:",size[2])
				#cv2.waitKey()
				if size[0]>max_height:
					max_height=size[0]
				if size[1]>max_width:
					max_width=size[1]
	print("total numbers:",number," max height:",max_height," max width:",max_width)

if __name__ == '__main__':
	main()
