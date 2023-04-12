from vpython import *
import time
scene.range = 5
IMU = box(length = 1, width = 1.5, height = 0.05, color = color.blue)
Xarrow = arrow(axis = vector(1,0,0), length=2,
               shaftwidth=0.1, color = color.red, opacity=0.3);
Yarrow = arrow(axis = vector(0,1,0), length=2,
               shaftwidth=0.1, color = color.green, opacity=0.3);
Zarrow = arrow(axis = vector(0,0,1), length=2,
               shaftwidth=0.1, color = color.blue, opacity=0.3);

