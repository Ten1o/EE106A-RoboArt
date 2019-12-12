# -*- coding: utf-8
import pickle
from time import sleep
import turtle
import sys

def get_strokes(word):
    name = "chin"
    if 'Chinese_character_recognition_final' in sys.path:
        name = 'Chinese_character_recognition_final/chin'
    with open(name, 'rb') as f:
        data = pickle.load(f)
    strokes = data[word]

    return strokes
