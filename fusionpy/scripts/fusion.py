#! /usr/bin/env python
import json
import numpy as np
import yaml
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from pcl_msgs.msg import pcl_msgs

import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
# from autoware_msgs.msg import DetectedObjectArray as DetectedObjectArray
# from autoware_msgs.msg import ImageObj as ImageObj
import os
from matplotlib.colors import Normalize
import matplotlib.cm as cm

color_map = {0: (255, 120, 180)}
line_seq = [(0, 1), (0, 3), (0, 4), (1, 2), (1, 5), (3, 2), (3, 7), (4, 5), (4, 7), (2, 6), (5, 6), (6, 7)]
cmap = cm.get_cmap('jet')  # heatmap的颜色种类设为rainbow
norm = Normalize(vmin=0, vmax=60)  # 归一化，将vmin和vmax分别映射到0和1
calib_file='/home/cqjtu/Documents/catktin_ros/src/cfg/lidar_camera_calibration65.yaml'

# def load_calib(calib_file):
#     f = open(calib_file, "r", encoding='utf-8')
#     calib = json.load(f)

#     return calib
def load_calib(yaml_file):
    cv_file = cv2.FileStorage(yaml_file, cv2.FILE_STORAGE_READ) 
    calib=dict()
    calib["Extrinsic"]=cv_file.getNode('CameraExtrinsicMat').mat()
    calib["Intrinsic"]=cv_file.getNode('CameraMat').mat()
    cv_file.release()
    return calib



def render_in_img(img, lidar, bboxes, labels, calib,  thickness=2):
    canvas = img.copy()
    canvas = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)

    l2c = np.array(calib["Extrinsic"]).reshape(4,4)
    l2c = np.linalg.inv(l2c)
    intri = np.array(calib["Intrinsic"])
    intrinisic = np.eye(4)
    intrinisic[:3, :3] = intri

    if lidar is not None:
        lidar = np.concatenate((lidar, np.ones((lidar.shape[0], 1))), axis=1)
        coords = l2c @ lidar.T
        coords = intrinisic @ coords
        coords = coords.reshape(4, -1)

        indices = coords[2] >0  

        coords = coords.T[indices]
        
        coords[:, 2] = np.clip(coords[:, 2], a_min=1e-5, a_max=1e5)
        coords[:, 0] /= coords[:, 2]
        coords[:, 1] /= coords[:, 2]

        kept = (coords.T[0] > 0) & (coords.T[1] > 0) & (coords.T[0] < 1600) & (coords.T[1] < 1200)

        coords = coords[kept]
        depth = coords[:,2]

        coords = coords[:, :2].astype(int)
        if len(coords)>0:
            print(f'coords:{len(coords)}')
            for coord,dep in zip(coords,depth):
                rgba = cmap(norm(dep))
                rgb_r = int(255 * rgba[0])
                rgb_g = int(255 * rgba[1])
                rgb_b = int(255 * rgba[2])

                bgr_color = (rgb_b, rgb_g, rgb_r)

                cv2.circle(canvas, (coord), 0, bgr_color, 3)

    if bboxes is not None and len(bboxes) > 0:
        coords = np.concatenate(
            [bboxes.reshape(-1, 3), np.ones((8, 1))], axis=-1
        )
        coords = l2c @ coords.T
        coords = intrinisic @ coords

        coords = coords.reshape(-1, 4, 8)

        indices = np.all(coords[:, 2] > 0, axis=1)
        coords = coords[indices]
        labels = labels[indices]

        indices = np.argsort(-np.min(coords[..., 2], axis=1))
        coords = coords[indices]
        labels = labels[indices]

        coords = coords.reshape(-1, 8).T
        coords[:, 2] = np.clip(coords[:, 2], a_min=1e-5, a_max=1e5)
        coords[:, 0] /= coords[:, 2]
        coords[:, 1] /= coords[:, 2]

        coords = coords[..., :2].reshape(-1, 8, 2)
        for index in range(coords.shape[0]):
            for start, end in line_seq:
                cv2.line(
                    canvas,
                    coords[index, start].astype(np.int),
                    coords[index, end].astype(np.int),
                    color_map[labels[index]],
                    thickness,
                    cv2.LINE_AA,
                )

    canvas = canvas.astype(np.uint8)
    canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return canvas


def callback(img,lidar):

  
    try:
        bridge = CvBridge()
    except CvBridgeError as e:
        print(e)
    det_image = bridge.imgmsg_to_cv2(img, 'bgr8')
    print(f"image:{type(det_image)}")
    # cv2.imshow('color_image',det_image)
    # cv2.waitKey(1)
    rospy.loginfo("get info")

    
    calib=load_calib(calib_file)
    msg = PointCloud2()
    pc=pc2.read_points(lidar,field_names=("x","y","z"),skip_nans=True)
    pc_list=np.array(list(pc))
    # print(pc_list)
    # print(len(pc_list))
    # pc_list=pc_list[pc_list[:,1]>=0]
    # print(len(pc_list))
    print(pc_list.shape)

    det_image=render_in_img(det_image,pc_list,None,None,calib,1)

    image_msg=bridge.cv2_to_imgmsg(det_image)
    imagepub.publish(image_msg)



if __name__ == '__main__':

    rospy.init_node('get_image', anonymous=True)
    cloudpub = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=5)
    imagepub=rospy.Publisher("image_topic",Image,queue_size=1)

    color = message_filters.Subscriber("/hik_cam_node/hik_camera", Image)
    depth = message_filters.Subscriber("/points_cluster", PointCloud2)
    # color_depth = message_filters.ApproximateTimeSynchronizer([color, depth], 10, 1, allow_headerless=True)  # 接近时间同步
    color_depth = message_filters.ApproximateTimeSynchronizer([color, depth], 1,slop=1,allow_headerless=True)  # 绝对时间同步
    color_depth.registerCallback(callback)  
    
    #同时订阅/camera/color/image_raw和/camera/aligned_depth_to_color/image_raw话题，并利用message_filters实现话题同步，共同调用callback
    rospy.spin()

    