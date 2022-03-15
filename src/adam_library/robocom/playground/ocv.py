import cv2

minHessian = 400
detector=cv2.SIFT_create(nfeatures=minHessian)
detector=cv2.xfeatures2d_SIFT.create(nfeatures=minHessian)