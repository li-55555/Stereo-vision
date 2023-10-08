import sys
import cv2
import numpy as np
import stereoconfig
import open3d as o3d
import sys


# 预处理
def preprocess(img1, img2):
    # 彩色图->灰度图
    if (img1.ndim == 3):
        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)  # 通过OpenCV加载的图像通道顺序是BGR
    if (img2.ndim == 3):
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # 直方图均衡 减轻光照不均的影响
    img1 = cv2.equalizeHist(img1)
    img2 = cv2.equalizeHist(img2)

    # 高斯滤波 在图片质量不好时使用
    # kernel_size = (3, 3)  # 内核大小
    # img1 = cv2.GaussianBlur(img1, kernel_size,0)
    # img2 = cv2.GaussianBlur(img2, kernel_size,0)

    # 中值滤波 消除孤立的噪点
    img1 = cv2.medianBlur(img1, 3)
    img2 = cv2.medianBlur(img2, 3)
    return img1, img2


# 消除畸变
def undistortion(image, camera_matrix, dist_coeff):
    undistortion_image = cv2.undistort(image, camera_matrix, dist_coeff)

    return undistortion_image


# 获取畸变校正和立体校正的映射变换矩阵、重投影矩阵
# @param：config是一个类，存储着双目标定的参数:config = stereoconfig.stereoCamera()
def getRectifyTransform(height, width, config):
    # 读取内参和外参
    left_K = config.cam_matrix_left
    right_K = config.cam_matrix_right
    left_distortion = config.distortion_l
    right_distortion = config.distortion_r
    R = config.R
    T = config.T

    # 计算校正变换
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(left_K, left_distortion, right_K, right_distortion,
                                                      (width, height), R, T, alpha=0)

    map1x, map1y = cv2.initUndistortRectifyMap(left_K, left_distortion, R1, P1, (width, height), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(right_K, right_distortion, R2, P2, (width, height), cv2.CV_32FC1)

    return map1x, map1y, map2x, map2y, Q


# 畸变校正和立体校正
def rectifyImage(image1, image2, map1x, map1y, map2x, map2y):
    rectifyed_img1 = cv2.remap(image1, map1x, map1y, cv2.INTER_AREA)
    rectifyed_img2 = cv2.remap(image2, map2x, map2y, cv2.INTER_AREA)

    return rectifyed_img1, rectifyed_img2


# 立体校正检验----画线
def draw_line(image1, image2):
    # 建立输出图像
    height = max(image1.shape[0], image2.shape[0])
    width = image1.shape[1] + image2.shape[1]

    output = np.zeros((height, width, 3), dtype=np.uint8)
    output[0:image1.shape[0], 0:image1.shape[1]] = image1
    output[0:image2.shape[0], image1.shape[1]:] = image2

    # 绘制等间距平行线
    line_interval = 50  # 直线间隔：50
    for k in range(height // line_interval):
        cv2.line(output, (0, line_interval * (k + 1)), (2 * width, line_interval * (k + 1)), (0, 255, 0), thickness=2,
                 lineType=cv2.LINE_AA)

    return output


# 视差计算
def stereoMatchSGBM(left_image, right_image, down_scale=False):
    # SGBM匹配参数设置
    if left_image.ndim == 2:
        img_channels = 1
    else:
        img_channels = 3
    blockSize = 5
    paraml = {'minDisparity': 0,
             'numDisparities': 320,#SGBM认为图像左侧前numDisparities列是不能估计视差的，所以会看到视差图左侧有一块是黑色
             'blockSize': blockSize,
             'P1': 64 * img_channels * blockSize ** 2,
             'P2': 256 * img_channels * blockSize ** 2,
             'disp12MaxDiff': 1,
             'preFilterCap': 63,
             'uniquenessRatio': 15,
             'speckleWindowSize': 100,
             'speckleRange': 1,
             'mode': cv2.STEREO_SGBM_MODE_SGBM_3WAY
             }
 
    # 构建SGBM对象
    left_matcher = cv2.StereoSGBM_create(**paraml)
    paramr = paraml
    paramr['minDisparity'] = -paraml['numDisparities']
    right_matcher = cv2.StereoSGBM_create(**paramr)
 
    # 计算视差图
    size = (left_image.shape[1], left_image.shape[0])
    if down_scale == False:
        disparity_left = left_matcher.compute(left_image, right_image)
        disparity_right = right_matcher.compute(right_image, left_image)
 
    else:# 降采样并插值处理图像，提高计算效率，并使图像更加平滑，减少图像噪声
        left_image_down = cv2.pyrDown(left_image)
        right_image_down = cv2.pyrDown(right_image)
        factor = left_image.shape[1] / left_image_down.shape[1]
 
        disparity_left_half = left_matcher.compute(left_image_down, right_image_down)
        disparity_right_half = right_matcher.compute(right_image_down, left_image_down)
        disparity_left = cv2.resize(disparity_left_half, size, interpolation=cv2.INTER_AREA)
        disparity_right = cv2.resize(disparity_right_half, size, interpolation=cv2.INTER_AREA)
        disparity_left = factor * disparity_left
        disparity_right = factor * disparity_right
 
    # 真实视差（因为SGBM算法得到的视差是×16的）
    trueDisp_left = disparity_left.astype(np.float32) / 16.
    trueDisp_right = disparity_right.astype(np.float32) / 16.
 
    return trueDisp_left, trueDisp_right

def getDepthMapWithQ(disparityMap: np.ndarray, Q: np.ndarray) -> np.ndarray:
    points_3d = cv2.reprojectImageTo3D(disparityMap, Q)
    depthMap = points_3d[:, :, 2]
    reset_index = np.where(np.logical_or(depthMap < 0.0, depthMap > 65535.0))
    depthMap[reset_index] = 0

    return depthMap.astype(np.float32)


def getDepthMapWithConfig(disparityMap: np.ndarray, config: stereoconfig.stereoCamera) -> np.ndarray:
    fb = config.cam_matrix_left[0, 0] * (-config.T[0])
    doffs = config.doffs
    depthMap = np.divide(fb, disparityMap + doffs)
    reset_index = np.where(np.logical_or(depthMap < 0.0, depthMap > 65535.0))
    depthMap[reset_index] = 0
    reset_index2 = np.where(disparityMap < 0.0)
    depthMap[reset_index2] = 0
    return depthMap.astype(np.float32)

def WLS_SGBM(left_image, right_image):#利用WLS滤波器改善视差图质量，一般只在视差图质量不好时使用
    if left_image.ndim == 2:
        img_channels = 1
    else:
        img_channels = 3
    blockSize = 5
    size = (left_image.shape[1], left_image.shape[0])
    paraml = {'minDisparity': 0,
             'numDisparities': 320,#SGBM认为图像左侧前numDisparities列是不能估计视差的，所以会看到视差图左侧有一块是黑色
             'blockSize': blockSize,
             'P1': 64 * img_channels * blockSize ** 2,
             'P2': 256 * img_channels * blockSize ** 2,
             'disp12MaxDiff': 1,
             'preFilterCap': 63,
             'uniquenessRatio': 15,
             'speckleWindowSize': 100,
             'speckleRange': 1,
             'mode': cv2.STEREO_SGBM_MODE_SGBM_3WAY
             }
    left_matcher = cv2.StereoSGBM_create(**paraml)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    # WLS滤波器参数
    lmbda = 80000
    sigma = 1.3
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)

    left_image_down = cv2.pyrDown(left_image)
    right_image_down = cv2.pyrDown(right_image)
    factor = left_image.shape[1] / left_image_down.shape[1]
    disparity_left_half = left_matcher.compute(left_image_down, right_image_down)
    disparity_right_half = right_matcher.compute(right_image_down, left_image_down)
    disparity_left = cv2.resize(disparity_left_half, size, interpolation=cv2.INTER_AREA)
    disparity_right = cv2.resize(disparity_right_half, size, interpolation=cv2.INTER_AREA)
    displ = factor * disparity_left
    dispr = factor * disparity_right

    displ = displ.astype(np.float32) / 16.
    dispr = dispr.astype(np.float32) / 16.
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filteredImg = wls_filter.filter(displ, left_image, None, dispr)
    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
    filteredImg = np.uint8(filteredImg)
    return filteredImg

if __name__ == '__main__':

    # print(sys.argv[0])
    # print(sys.argv[1])
    # print(sys.argv[2])

    # 与相机前端联动时，使用下面的代码
    iml = cv2.imread(sys.argv[1], 1)  # 左图
    imr = cv2.imread(sys.argv[2], 1)  # 右图

    # 直接读取图片时，使用下面的代码
    # iml = cv2.imread("right1.bmp", 1)  # 左图
    # imr = cv2.imread("left1.bmp", 1)  # 右图
    if (iml is None) or (imr is None):
        print("Error: Images are empty, please check your image's path!")
        sys.exit(0)
    height, width = iml.shape[0:2]

    # 读取相机内参和外参
    # 使用之前先将标定得到的内外参数填写到stereoconfig.py中的StereoCamera类中
    config = stereoconfig.stereoCamera()
    print(config.cam_matrix_left)

    # 立体校正
    map1x, map1y, map2x, map2y, Q = getRectifyTransform(height, width, config)  # 获取用于畸变校正和立体校正的映射矩阵以及用于计算像素空间坐标的重投影矩阵
    iml_rectified, imr_rectified = rectifyImage(iml, imr, map1x, map1y, map2x, map2y)
    print(Q)

    # 绘制等间距平行线，检查立体校正的效果
    line = draw_line(iml_rectified, imr_rectified)
    cv2.imwrite('check_rectification.png', line)

    # 立体匹配
    iml_, imr_ = preprocess(iml_rectified, imr_rectified)  # 预处理
    disp,_ = stereoMatchSGBM(iml_, imr_, True)  
    # disp = WLS_SGBM(iml_,imr_) 利用WLS滤波器改善视差图质量，一般只在视差图质量不好时使用
    disp1 = cv2.normalize(disp,None,alpha=0,beta=255,norm_type=cv2.NORM_MINMAX,dtype=cv2.CV_8U)
    disp1 = cv2.applyColorMap(disp1*8,2)
    cv2.imwrite('disaprity.png', disp1[0:1080, 512:1920])

    # 计算深度图
    #depthMap = getDepthMapWithQ(disp, Q)
    depthMap = getDepthMapWithConfig(disp, config)
    minDepth = np.min(depthMap)
    maxDepth = np.max(depthMap)
    print(minDepth, maxDepth)
    depthMapVis = (255.0 * (depthMap - minDepth)) / (maxDepth - minDepth)
    depthMapVis = depthMapVis.astype(np.uint8)

    def callbackFunc(e, x, y, f, p):
        if e == cv2.EVENT_LBUTTONDOWN:
            print('目标的深度距离为 %2f mm' % depthMap[y][x])

    cv2.namedWindow('DepthMap', 0)
    cv2.setMouseCallback("DepthMap", callbackFunc, None)
    cv2.imshow("DepthMap", depthMapVis)
    cv2.waitKey(0)

    # 使用open3d库绘制点云
    iml = cv2.cvtColor(iml_rectified, cv2.COLOR_BGR2RGB)
    colorImage = o3d.geometry.Image(iml)
    depthImage = o3d.geometry.Image(depthMap)
    rgbdImage = o3d.geometry.RGBDImage.create_from_color_and_depth(colorImage, depthImage, depth_scale=1000.0,
                                                                     depth_trunc=0.6,convert_rgb_to_intensity=False)
    intrinsics = o3d.camera.PinholeCameraIntrinsic()
    fx = config.cam_matrix_left[0, 0]
    fy = config.cam_matrix_left[1, 1]
    cx = config.cam_matrix_left[0, 2]
    cy = config.cam_matrix_left[1, 2]
    print(fx, fy, cx, cy)
    intrinsics.set_intrinsics(width, height, fx=fx, fy=fy, cx=cx, cy=cy)
    extrinsics = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.],
                           [0., 0., 1., 0.],
                           [0., 0., 0., 1.]])
    pointcloud = o3d.geometry.PointCloud().create_from_rgbd_image(rgbdImage, intrinsic=intrinsics, extrinsic=extrinsics)

    # 计算像素点的3D坐标（左相机坐标系下）
    points_3d = cv2.reprojectImageTo3D(disp, Q)  # 参数中的Q就是由getRectifyTransform()函数得到的重投影矩阵

    # 构建点云--Point_XYZRGBA格式
    o3d.io.write_point_cloud("PointCloud.pcd", pointcloud=pointcloud)
    o3d.visualization.draw_geometries([pointcloud])

    # 进一步去除干扰背景（可选）
    # max_depth = 0.6
    # min_depth = 0.1
    # keep_indices = []
    # for i in range(len(pointcloud.points)):
    #     if pointcloud.points[i][2] < max_depth and pointcloud.points[i][2] > min_depth:
    #         keep_indices.append(i)

    # pcd = pointcloud.select_by_index(keep_indices)

    # o3d.visualization.draw_geometries([pcd])

    # 平面分割
    plane_model, inliers = pointcloud.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)

    # 可视化
    inlier_cloud = pointcloud.select_by_index(inliers)
    outlier_cloud = pointcloud.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud.paint_uniform_color([0, 1.0, 0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])