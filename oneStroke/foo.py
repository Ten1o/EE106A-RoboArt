# This Python file uses the following encoding: utf-8
from __future__ import division, absolute_import, print_function

import cv2
import numpy as np
from tensorflow.keras.models import model_from_json, load_model
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import minimum_spanning_tree, depth_first_order
from oneStroke import HAARCASCADE_MODEL_DIR, DARK_JSON_DIR, DARK_H5_DIR


def face_crop(img):
    """
    Find the face & back ground reduction.

    Parameters
    ----------
    `img` : 2d numpy array 
        grayscale image

    Returns
    -------
    `face_crop` : 2d numpy array 
        grayscale image resized to 256x256

    """
    # human matting
    with open(DARK_JSON_DIR, 'r') as f:
        json_string = f.read()
    model = model_from_json(json_string)
    model.load_weights(DARK_H5_DIR)
    res = np.argmax(model.predict(img[np.newaxis,:,:,np.newaxis].astype(np.float32)), axis=3)[0]
    img[res == 0] = 0

    # ------ debugging ------
    cv2.imwrite("net.png", img)
    print("--- picture \"net.png\" saved ---")
    # -----------------------

    # crop face
    face_cascade = cv2.CascadeClassifier(HAARCASCADE_MODEL_DIR)
    faces = face_cascade.detectMultiScale(img, 1.3, 5)
    if faces == ():
        print("No face founded\nexit with code 0")
        exit(0)
    (x, y, w, h) = faces[np.argmax(faces[:,-1])]
    height, width = img.shape
    if int(y-0.15*h) < 0 or int(y+1.15*h) >= height or int(x-0.15*w) < 0 or int(x+1.15*w) >= width:
        print("Face not complete\nexit with code 0")
        exit(0)
    face_crop = cv2.resize(img[int(y-0.15*h):int(y+1.15*h), int(x-0.15*w):int(x+1.15*w)], (256, 256))

    # ------ debugging ------
    cv2.imwrite("face.png", face_crop)
    print("--- picture \"face.png\" saved ---")
    # -----------------------

    return face_crop


def edge_detect(img):
    """
    Find the edge points of an image.

    Parameters
    ----------
    `img` : 2d numpy array 
        grayscale image

    Returns
    -------
    `edge_points` : 2d numpy array 
        a set of (x, y) coordinates

    """
    # image enhancement
    # contrast limited adaptive histogram equalization
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    img = clahe.apply(img)

    # ------ debugging ------
    cv2.imwrite("clahe.png", img)
    print("--- picture \"clahe.png\" saved ---")
    # -----------------------

    blur_img = cv2.bilateralFilter(img, 7, 50, 50)
    # Auto Canny
    _, thr = cv2.threshold(blur_img, 0, 1, cv2.THRESH_OTSU)
    lowerThreshold = np.mean(blur_img[thr==0])
    upperThreshold = np.mean(blur_img[thr==1])
    edge_img = cv2.Canny(blur_img, lowerThreshold, upperThreshold)

    edge_indexes = np.argwhere(edge_img != 0)
    edge_points = np.zeros(edge_indexes.shape)
    edge_points[:,0] = edge_indexes[:,1]
    edge_points[:,1] = img.shape[0] - edge_indexes[:,0]
    return edge_points


def adj_matrix(vertices):
    """
    Find the adjacency matrix of a complete graph defined by a set of vertices.

    Parameters
    ----------
    `vertices` : 2d numpy array 
        a set of (x, y) coordinates

    Returns
    -------
    `adjMat` : scipy sparse matrix 
        adjacency matrix of a complete graph defined by `vertices`

    """
    n = vertices.shape[0]
    adjMat = np.zeros((n, n))
    for i in range(n):
        adjMat[i, :] = np.linalg.norm(vertices - vertices[i], axis=1)
    adjMat = csr_matrix(adjMat)
    return adjMat


def mst(adjMat):
    """
    Find the minimum spanning tree of a graph defined by adjacency matrix.

    Parameters
    ----------
    `adjMat` : scipy sparse matrix
        adjacency matrix of a graph

    Returns
    -------
    `adjMat_of_mst` : scipy sparse matrix 
        adjacency matrix of minimum spanning tree of the graph

    """
    adjMat_of_mst = minimum_spanning_tree(adjMat)
    return adjMat_of_mst


def dfs(adjMat):
    """
    Find the depth first search order of a graph defined by adjacency matrix.

    Parameters
    ----------
    `adjMat` : scipy sparse matrix
        adjacency matrix of a graph

    Returns
    -------
    `target_path_index` : 1d numpy array 
        a path of indices of points that walk through the graph in depth first order

    """
    i_start = divmod(np.argmax(adjMat), adjMat.shape[0])[0]
    path_index, predecessors = depth_first_order(adjMat, i_start, directed=False)
    target_path_index = []
    for i in range(len(path_index) - 1):
        curVertex = path_index[i]
        target_path_index.append(curVertex)
        nextVertex = path_index[i+1]
        while predecessors[nextVertex] != curVertex:
            curVertex = predecessors[curVertex]
            target_path_index.append(curVertex)
    target_path_index.append(path_index[-1])
    target_path_ = np.array(target_path_index)
    return target_path_index


def rdp(path, epsilon=2.56):
    """
    Use [Ramer–Douglas–Peucker algorithm](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm)
    to downsample points on path.

    Parameters
    ----------
    `path` : 2d numpy array 
        a set of (x, y) coordinates
    `epsilon` : positive real number
        maximum tolerance of error

    Returns
    -------
    Downsampled path

    Note
    ------
    Sometimes, if the path goes back and forth, the implementation may not work properly.
    Say if the path starts at one point, goes wherever it want and ends at the exact
    point it starts, the implementation would just give you that point.

    """
    start = 0
    end = len(path) - 1
    vec1 = (path[end] - path[start]).reshape(-1,1)
    vecs = path - path[start]
    projMat = vec1.dot(vec1.T) / np.linalg.norm(vec1) ** 2
    errVec = (np.eye(2) - projMat).dot(vecs.T)
    errNorm = np.linalg.norm(errVec.T, axis=1)
    imax = np.argmax(errNorm)
    dmax = errNorm[imax]
    if dmax >= epsilon:
        return np.vstack((rdp(path[start:imax+1], epsilon), 
                          rdp(path[imax:end+1], epsilon)[1:]))
    else:
        return np.vstack((path[start], path[end]))
