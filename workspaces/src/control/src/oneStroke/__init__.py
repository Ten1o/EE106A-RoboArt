# This Python file uses the following encoding: utf-8
from __future__ import division, absolute_import, print_function

import os

dirname = os.path.dirname(os.path.abspath(__file__))

HAARCASCADE_MODEL_DIR = os.path.join(dirname, 'haarcascade_frontalface_default.xml')
if not os.path.exists(HAARCASCADE_MODEL_DIR):
    msg = """
Face detection model <haarcascade_frontalface_default.xml> does not exist.
Fail to import oneStroke.
    """
    raise ImportError(msg)

DARK_JSON_DIR = os.path.join(dirname, 'dark.json')
if not os.path.exists(DARK_JSON_DIR):
    msg = """
Human matting model <dark.json> does not exist.
Fail to import oneStroke.
    """
    raise ImportError(msg)

DARK_H5_DIR = os.path.join(dirname, 'dark.h5')
if not os.path.exists(DARK_H5_DIR):
    msg = """
Human matting model <dark.h5> does not exist.
Fail to import oneStroke.
    """
    raise ImportError(msg)

del os

from .foo import face_crop, edge_detect, adj_matrix, mst, dfs, rdp
__all__ = ['face_crop', 'edge_detect', 'adj_matrix', 'mst', 'dfs', 'rdp']
