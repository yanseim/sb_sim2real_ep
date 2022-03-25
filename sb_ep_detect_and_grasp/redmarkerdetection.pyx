import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
cimport numpy as np
cimport cython
import sys

DTYPE = np.uint8
ctypedef np.uint8_t DTYPE_t
ctypedef np.int DTYPE_i

cdef unsigned char absSub(unsigned char v1, unsigned char v2):
    return v1-v2 if v1>v2 else v2-v1

@cython.boundscheck(False)
@cython.wraparound(False)


# instruction:
# python3 setup.py build_ext --inplace
# python3 detector_python.py

def red_segmentation(np.ndarray[DTYPE_t, ndim=2] image,np.ndarray[DTYPE_t, ndim=3] hsv_image,np.ndarray[DTYPE_t, ndim=1] seg_papram):

    cdef int height, width, i, j
    height = image.shape[0]
    width = image.shape[1]

    hsv_0 = 0
    hsv_1 = 0
    hsv_2 = 0

    for i in range(height):
        for j in range(width):
            hsv_0 = hsv_image[i,j,0]
            hsv_1 = hsv_image[i,j,1]
            hsv_2 = hsv_image[i,j,2]

            if (not((((hsv_0 >= seg_papram[0]) and (hsv_0 <= seg_papram[1]))
                 or (hsv_0 >= seg_papram[2]) and (hsv_0 <= seg_papram[3])) 
                 and (hsv_2>=seg_papram[4]) and (hsv_1>=seg_papram[5]))):
                image[i,j] = 0
            else:
                image[i,j] = 255
                
def img_sum(np.ndarray[DTYPE_t, ndim=2] image):
    cdef int height, width, i, j
    height = image.shape[0]
    width = image.shape[1]
    cdef int sum = 0
    for i in range(height):
        for j in range(width):
            sum += image[i,j]
    return sum

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def cross(x1,y1,x2,y2):
    return x1*y2-x2*y1

def compare(a=point(0,0),b=point(0,0),c=point(0,0)):
    return cross((b.x-a.x),(b.y-a.y),(c.x-a.x),(c.y-a.y))

def cmp2(a,b):
    c = point(0,0)
    if(compare(c,a,b)==0):
        return a.x<b.x
    else:
        return compare(c,a,b)>0

def sort_contour(cnt):

    if not len(cnt) == 4:
        return None

    cx = (cnt[0,0,0]+cnt[1,0,0]+cnt[2,0,0]+cnt[3,0,0])/4
    cy = (cnt[0,0,1]+cnt[1,0,1]+cnt[2,0,1]+cnt[3,0,1])/4

    cnt_norm = cnt.copy()
    for i in range(4):
        cnt_norm[i,0,0] = cnt[i,0,0] - cx
        cnt_norm[i,0,1] = cnt[i,0,1] - cy

    for t in range(10):
        for i in range(3):
            p1 = point(cnt_norm[i,0,0],cnt_norm[i,0,1])
            p2 = point(cnt_norm[i+1,0,0],cnt_norm[i+1,0,1])
            if cmp2(p1,p2):
                cnt_norm[i,0,0],cnt_norm[i+1,0,0] = cnt_norm[i+1,0,0],cnt_norm[i,0,0]
                cnt_norm[i,0,1],cnt_norm[i+1,0,1] = cnt_norm[i+1,0,1],cnt_norm[i,0,1]

    for i in range(4):
        cnt_norm[i,0,0] = cnt_norm[i,0,0] + cx
        cnt_norm[i,0,1] = cnt_norm[i,0,1] + cy

    return cnt_norm

templates = []

def load_template():
    tpl_path = sys.path[0]+"/tpl/"
    for i in range(8):
        tpl = cv2.imread(tpl_path + str(i) + ".png", 0)
        print(tpl.shape)
        templates.append(tpl)

def marker_detection(np.ndarray[DTYPE_t, ndim=3] frame,np.ndarray[DTYPE_t, ndim=1] seg_papram):

    r = 0.045
    model_object = np.array([(0-0.5*r,0-0.5*r, 0.0),
                            (r-0.5*r, 0-0.5*r, 0.0),
                            (r-0.5*r, r-0.5*r, 0.0),
                            (0-0.5*r, r-0.5*r, 0.0)])
    camera_matrix = np.array([
                 (617.3054000792732, 0.0, 424.0),
                 (0.0, 617.3054000792732, 240.0),
                 (0,0,1)],
				 dtype="double")
    dist_coeffs = np.array([[0,0,0,0]], dtype="double")

    hsvImg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    match_threshold = 150000

    red_segmentation(grayImg,hsvImg,seg_papram)

    contours, hierarchy = cv2.findContours(grayImg,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)  

    quads = []
    quads_f = []
    quads_prj = []

    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        bbox = cv2.boundingRect(cnt)
        if area >=30 :
            approx = cv2.approxPolyDP(cnt,15,True)
            if len(approx) == 4:
                approx_sort = sort_contour(approx)
                quads.append(approx_sort)
                quads_f.append(approx_sort.astype(float))

    rvec_list = []
    tvec_list = []
    for i in range(len(quads_f)):
        model_image = np.array([(quads_f[i][0,0,0],quads_f[i][0,0,1]),
                                (quads_f[i][1,0,0],quads_f[i][1,0,1]),
                                (quads_f[i][2,0,0],quads_f[i][2,0,1]),
                                (quads_f[i][3,0,0],quads_f[i][3,0,1])])
        ret, rvec, tvec = cv2.solvePnP(model_object, model_image, camera_matrix, dist_coeffs)
        projectedPoints,_ = cv2.projectPoints(model_object, rvec, tvec, camera_matrix, dist_coeffs)

        err = 0
        for t in range(len(projectedPoints)):
            err += np.linalg.norm(projectedPoints[t]-model_image[t])

        area = cv2.contourArea(quads[i])
        if err/area < 0.005 and tvec[2]<1.55:
            quads_prj.append(projectedPoints.astype(int))
            rvec_list.append(rvec)
            tvec_list.append(tvec)

    # print("model_image",model_image)
    # print("model_object",model_object)
    # print("quads[0]",quads[0])
    # ('model_image', array([[419., 226.],
    #    [418., 304.],
    #    [497., 307.],
    #    [498., 227.]]))
    # ('model_object', array([[-0.0225, -0.0225,  0.    ],
    #    [ 0.0225, -0.0225,  0.    ],
    #    [ 0.0225,  0.0225,  0.    ],
    #    [-0.0225,  0.0225,  0.    ]]))
    # ('quads[0]', array([[[609, 325]],
    #     [[608, 413]],
    #     [[694, 413]],
    #     [[694, 325]]], dtype=int32))

    quads_prj_draw = []
    quads_ID = []
    quads_prj_draw_4_showing_target = []
    # print("quads_prj",quads_prj)
    for i in range(len(quads_prj)):# this loop is the most important, number detection!!
        points_src = np.array([[(quads_prj[i][0,0,0],quads_prj[i][0,0,1])],
                                [(quads_prj[i][1,0,0],quads_prj[i][1,0,1])],
                                [(quads_prj[i][2,0,0],quads_prj[i][2,0,1])],
                                [(quads_prj[i][3,0,0],quads_prj[i][3,0,1])]],dtype = "float32")# the 4 points of the quad in the camera
        points_dst = np.array([[50,0],
                                [0,0],
                                [0,50],
                                [50,50]],dtype = "float32")# 50*50
        M = cv2.getPerspectiveTransform(points_src,points_dst)
        out_img = cv2.warpPerspective(frame,M,(50,50))
        out_img = cv2.cvtColor(out_img, cv2.COLOR_BGR2GRAY)
        out_img = cv2.threshold(out_img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

        my_out_img = np.zeros((50,50),dtype="uint8")
        my_out_img[7:43,7:43] = out_img[7:43,7:43]

        match_candidate = []
        match_candidate.append(my_out_img)
        match_candidate.append(cv2.rotate(my_out_img, cv2.ROTATE_180))
        match_candidate.append(cv2.rotate(my_out_img, cv2.ROTATE_90_CLOCKWISE))
        match_candidate.append(cv2.rotate(my_out_img, cv2.ROTATE_90_COUNTERCLOCKWISE))

        min_diff = 100000000000
        min_diff_target = 0


        for t in range(8):
            for tt in range(4):
                diff_img = cv2.absdiff(templates[t], match_candidate[tt])
                sum = img_sum(diff_img)
                if min_diff > sum:
                    min_diff = sum
                    min_diff_target = t
                # if t in [2,4]:
                #     print("t = %d"%t)
                #     print("min_diff",min_diff)
                #     print("sum",sum)

        # for avoiding misdetecting 3 to O=================
        if min_diff_target==6:
            sum = 0
            center_start = 21
            center_end = 28
            for tt in range(4):
                img_center = match_candidate[tt][center_start:center_end,center_start:center_end]
                sum += img_sum(img_center)
            if sum>(center_end-center_start)*(center_end-center_start)*0.6*255:
                min_diff_target = 2
        
        # =================================================

        if min_diff < match_threshold:
            quads_ID.append(min_diff_target)
            quads_prj_draw.append(quads_prj[i])
            quads_prj_draw_4_showing_target.append(quads_prj[i])
        else:
            quads_ID.append(-1) # bcz this line,quads_ID has the same length with quads_prj!! however quads_prj_draw does not
            quads_prj_draw_4_showing_target.append(quads_prj[i])# quads_prj_draw_4_showing_target has the same length with quads_prj
            
        
        # if min_diff_target==4:
        #     cv2.imshow('tem0', templates[min_diff_target])
        #     cv2.imshow('tem3', templates[2])
        #     cv2.imshow('match candidate', out_img)
        #     cv2.waitKey(1)
    
    # print("quads_ID",quads_ID)
# ===============================================================

    # choose which plane to publish
    idx_chosen_to_pub = -1

    # min_y = 1
    # for i in range(len(quads_ID)):
    #     if quads_ID[i] == 0 or quads_ID[i] == 1 or quads_ID[i] == 2 or quads_ID[i] == 6  or quads_ID[i] == 7 and tvec_list[i][1]>-0.2:
    #         if min_y>tvec_list[i][1]:
    #             min_y = tvec_list[i][1]
    # print("min_y",min_y)

    min_norm = 10
    
    # print("quads_prj_draw",quads_prj_draw)
    for i in range(len(quads_ID)):
        if quads_ID[i] == 0 or quads_ID[i] == 1 or quads_ID[i] == 2 or quads_ID[i] == 6  or quads_ID[i] == 7:
            if tvec_list[i][0] < 0.5 and tvec_list[i][0] > -0.5 and \
                tvec_list[i][1] < 0.03 and tvec_list[i][1] > -0.15 and tvec_list[i][1] and \
                tvec_list[i][2] < 1.5:
                i_norm = np.linalg.norm( R.from_rotvec(np.reshape(rvec_list[i],(3,))).as_matrix()-np.array([[0,1,0],[1,0,0],[0,0,-1]]) )
                # print("in loop",R.from_rotvec(np.reshape(rvec_list[i],(3,))).as_matrix())
                if min_norm > i_norm:
                    min_norm = i_norm
                    idx_chosen_to_pub = i
    # print("R.from_rotvec(np.reshape(rvec_list[i],(3,))).as_matrix()",R.from_rotvec(np.reshape(rvec_list[idx_chosen_to_pub],(3,))).as_matrix())


    # norm_epsilon = 2
    # while len(id_list_chosen)>1:
    #     for i in range(len(id_list_chosen)):
    #         rot = R.from_rotvec(np.reshape(rvec_list_chosen[i],(3,)))
    #         R1 = rot.as_matrix()
    #         Ry1 = np.array([[0,0,-1],[0,1,0],[1,0,0]])
    #         Ry2 = np.array([[0,0,1],[0,1,0],[-1,0,0]])
    #         for k in range(len(id_list_chosen)):
    #             min_norm = 10
    #             if i != k:
    #                 R2 = R.from_rotvec(np.reshape(rvec_list_chosen[k],(3,))).as_matrix()
    #                 if min_norm > min(np.linalg.norm(Ry1@R1-R2),np.linalg.norm(Ry2@R1-R2)):
    #                     min_norm = min(np.linalg.norm(Ry1@R1-R2),np.linalg.norm(Ry2@R1-R2))
    #                     k_min = k
    #         print("min_norm",min_norm)
    #         if min_norm<norm_epsilon:
    #             delete_one = i if np.linalg.norm(R1-np.eye(3))>np.linalg.norm(R.from_rotvec(np.reshape(rvec_list_chosen[k],(3,))).as_matrix()-np.eye(3)) else k
    #             del id_list_chosen[delete_one]
    #             del tvec_list_chosen[delete_one]
    #             del rvec_list_chosen[delete_one]
    #             break


    for i in range(len(quads_prj)):
        bbox = cv2.boundingRect(quads_prj[i])
        if quads_ID[i] == 0:
            cv2.putText(frame, '1', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 1:
            cv2.putText(frame, '2', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 2:
            cv2.putText(frame, '3', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 3:
            cv2.putText(frame, '4', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 4:
            cv2.putText(frame, '5', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 5:
            cv2.putText(frame, 'B', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 6:
            cv2.putText(frame, 'O', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 7:
            cv2.putText(frame, 'X', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)

    cv2.drawContours(frame,quads_prj_draw,-1,(0,255,0),1)
    if idx_chosen_to_pub != -1:
        cv2.drawContours(frame,quads_prj_draw_4_showing_target[idx_chosen_to_pub],-1,(255,0,0),3)
    
    return quads_ID,tvec_list,rvec_list,idx_chosen_to_pub