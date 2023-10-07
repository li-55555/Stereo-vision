import numpy as np
import cv2 as cv
import torch as torch

# 边缘识别函数，利用canny方法和高斯模糊实现物体的边识别
def edge_detection(img,canny_a,canny_b):
    img0 = cv.GaussianBlur(img, (5, 5), 0)
    edges=cv.Canny(img0,canny_a,canny_b)
    _,edges=cv.threshold(edges,127,255,cv.THRESH_BINARY)
    contours,hierarchy=cv.findContours(edges,cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    return contours,hierarchy,edges

# 颜色识别函数，利用颜色对物体进行识别
def color_detection(img,l1,l2,l3,u1,u2,u3):
    lower_range=np.array([l1,l2,l3])
    upper_range=np.array([u1,u2,u3])
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv_img, lower_range, upper_range)
    target=cv.bitwise_and(img,img,mask=mask)
    return target
    
# def contour_matches(contours1,contours2,target):
#     for cnt2 in contours2:
#         best_match=None
#         best_match_score=float('inf')
#         for cnt1 in contours1:
#             match_score=cv.matchShapes(cnt1,cnt2,cv.CONTOURS_MATCH_I1,0.0)
#             if match_score<best_match_score:
#                 best_match_score=match_score
#                 best_match=cnt1
#         cv.drawContours(target,[cnt2],-1,(0,0,0),1)
#         # cv.drawContours(target,[best_match],-1,(0,0,255),1)
#     return best_match

if __name__ == '__main__':
    img =cv.imread('1.png')
    img2=cv.imread('2.png')
    # contours,_,edges=edge_detection(img,45,105)
    # contours2,_,edges2=edge_detection(img,45,105)
    # contour_matches(contours,contours2,img2)
    target=color_detection(img,12,17,55,45,255,255)
    c,_,edges=edge_detection(target,45,105)
    # cv.drawContours(img,contours,-1,(0,0,0),1)
    # 显示图像种类集
    # cv.imshow('result',img)
    # cv.imshow('edges',edges)
    # cv.imshow('best match',img2)
    cv.imshow('target',target)
    cv.waitKey(0)
    cv.destroyAllWindows()
