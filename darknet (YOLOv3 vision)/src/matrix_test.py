#!/usr/bin/env python


import numpy as np

A = np.array([[2, 0, -1],
               [5, 1, -2],
               [0, 1, -3]])

B = np.array([[-2],[-4],[-3]])

A_inverse = np.linalg.inv(A)

constants = np.matmul(A_inverse,B)

ray = [1,2,3]

coordinates = ray * constants[2] 
print(coordinates)


A = np.array([[1, 2, 0],
               [2, 5, 1],
               [3, 0, 1]])
A_inverse = np.linalg.inv(A)
B = np.array([[1-2],[2-4],[3-3]])

constants = np.matmul(A_inverse,B)
ray = [1,2,3]

coordinates = ray * (1 - constants[2])
print(coordinates)