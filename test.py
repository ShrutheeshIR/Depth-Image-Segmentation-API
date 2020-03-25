import cv2
import numpy as np
import sys
sys.path.append('build')
import bv
import time


cammat = np.array([[525.0,0,320], [0,525.0,240],[0,0,1]])

for i in range(300,500):
    t1= time.time()
    im = cv2.imread('/home/olorin/Desktop/IISc/TSDF/tsdf-fusion-python/data/frame-000' + str(i) + '.depth.png', 2|4)

    siz, imm = bv.segmentdepthimage(im, cammat)
    print(time.time()-t1)
    print(siz)
    cv2.imshow('IMA', imm*25)

    for i in range(siz-1):
        maskim = imm.copy()
        maskim[maskim!=i+1] = 0
        cv2.imshow(str(i+1), (maskim-i)*250)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
