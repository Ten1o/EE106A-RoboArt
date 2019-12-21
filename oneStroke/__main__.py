# This Python file uses the following encoding: utf-8
from __future__ import division, absolute_import, print_function

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from oneStroke import *

img_path = sys.argv[1]

img = cv2.imread(img_path, 0)
# resize img
presetRatio = np.array([1/2, 9/16, 3/4, 1, 4/3, 16/9, 2])
presetSize = [(256, 512), (288, 512), (384, 512),
                (256, 256), (512, 384), (512, 288), (512, 256)]
ratio = img.shape[1] / img.shape[0]
ratioIndex = np.argmin(np.abs(ratio - presetRatio))
img_shape = presetSize[ratioIndex]
img = cv2.resize(img, img_shape)

# comment out the line below if no face
img = face_crop(img)
edge_points = edge_detect(img)
# ------ logging ------
x, y = edge_points.T
plt.cla()
plt.axis("equal")
plt.scatter(x, y, s=2)
plt.savefig("edge.png")
print("--- picture \"edge.png\" saved ---")
# ---------------------

path_index = dfs(mst(adj_matrix(edge_points)))
path = edge_points[path_index]
# ------ logging ------
x, y = path.T
plt.cla()
plt.axis("equal")
plt.plot(x, y, linewidth=2.56)
plt.savefig("path.png")
print("--- picture \"path.png\" saved ---")
# ---------------------

downsampled_path = rdp(path)
print(len(downsampled_path))
# ------ logging ------
x, y = downsampled_path.T
plt.cla()
plt.axis("equal")
plt.plot(x, y, linewidth=2.56)
plt.savefig("rdp.png")
print("--- picture \"rdp.png\" saved ---")
# ---------------------

with open('path.txt', 'w') as f:
    f.write('[[')
    for i in downsampled_path:
        f.write('[{},{}],'.format(i[1], 256-i[0]))
    f.write(']]')
