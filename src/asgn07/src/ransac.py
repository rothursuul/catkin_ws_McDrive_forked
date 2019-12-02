import numpy as np
s = 2
sigma = sqrt(mean(abs(x - x.mean())**2))
t = sqrt(3.84*sigma)
n = 0

def get_point_lib(A):
    res = []
    for x in range(len(A-1)):
        for y in range (len(A[x]-1)):
            if A[x,y] == 255:
                res.append([x,y])
    return np.array(res)

def ransac(s, t, n):
    # randomly choose 2 points
    np.random.randint()