# -*- coding: utf-8

import sys
from PIL import Image
import numpy as np
import huzy_stroke
import chinese_character_rec


threshold = 70


character_dir = {"0": "一", "1": "丁", "2": "七", "3": "万", "4": "丈", "5": "三", "6": "上", "7": "下", "8": "不", "9": "与", 
				"10": "丑", "11": "专", "12": "且", "13": "世", "14": "丘", "15": "丙", "16": "业", "17": "丛", "18": "东", "19": "丝"}

def trim(img):
	(row, col) = img.shape
	row_head = 0
	row_tail = 0
	col_head = 0
	col_tail = 0

	for i in range(0, row):
		if np.where(img[i] < threshold)[0].size > 3:
			row_head = i
			break

	for i in range(row - 1, 0, -1):
		if np.where(img[i] < threshold)[0].size > 3:
			row_tail = i
			break
	for i in range(0, col):
		if np.where(img[:, i] < threshold)[0].size > 3:
			col_head = i
			break
	for i in range(col - 1, 0, -1):
		if np.where(img[:, i] < threshold)[0].size > 3:
			col_tail = i
			break
	return (row_head, row_tail, col_head, col_tail)

def get_path(img_dir):
	image = Image.open(img_dir)
	temp_image_array = np.array(image.convert('L'))
	(row_head, row_tail, col_head, col_tail) = trim(temp_image_array)
	image_array = np.array(image)
	image_array = image_array[row_head:row_tail+1, col_head:col_tail+1, 0:3]
	im = Image.fromarray(image_array)
	im.save("out.png")


	pred = str(int(chinese_character_rec.inference()))
	result = huzy_stroke.get_strokes(pred)

	out = [[] for _ in range(len(result))]
	for i in range(len(result)):
		for j in result[i]:
			out[i].append([j["x"], j["y"]])
	print("predict: ", character_dir[pred])
	print(out)
	return out
