#! /usr/bin/env python
import json
import numpy as np
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
from autoware_msgs.msg import DetectedObjectArray ,DetectedObject
from autoware_msgs.msg import ImageObj as ImageObj
from autoware_msgs.msg import ImageRect 
import os
from matplotlib.colors import Normalize
import matplotlib.cm as cm

color_map = {0: (255, 120, 180)}
line_seq = [(0, 1), (0, 3), (0, 4), (1, 2), (1, 5), (3, 2), (3, 7), (4, 5), (4, 7), (2, 6), (5, 6), (6, 7)]
cmap = cm.get_cmap('jet')  # heatmap的颜色种类设为rainbow
norm = Normalize(vmin=0, vmax=60)  # 归一化，将vmin和vmax分别映射到0和1
calib_file='/home/cqjtu/Documents/catktin_ros/src/cfg/lidar_camera_calibration65.yaml'
adjust_x=0
overlap_threshold_=0.3
class Rectangle:
    def __init__(self,x=0,y=0,wid=0,height=0):
        self.x=x
        self.y=y
        
        self.width=wid
        self.height=height
    def get_area(self):
    
        m=self.height*self.width
    
        return m
    def show_all(self):
        print('长',self.length,'宽',self.width,'面积',self.get_area())
def calculate_IoU(predicted_bound, ground_truth_bound):

    x1min, y1min =  predicted_bound.x,predicted_bound.y
    x1max, y1max=   predicted_bound.x+predicted_bound.width,predicted_bound.y+predicted_bound.height
    # print("预测框P的坐标是：({}, {}, {}, {})".format(pxmin, pymin, pxmax, pymax))
    x2min, y2min =  ground_truth_bound.x,ground_truth_bound.y
    x2max, y2max=ground_truth_bound.x+ground_truth_bound.width,ground_truth_bound.y+ground_truth_bound.height
    # print("原标记框G的坐标是：({}, {}, {}, {})".format(gxmin, gymin, gxmax, gymax))
    parea = predicted_bound.get_area()
    garea = ground_truth_bound.get_area()

    # 计算相交矩形框的坐标
    xmin = np.maximum(x1min, x2min)  # 左上角的横坐标
    ymin = np.maximum(y1min, y2min)  # 左上角的纵坐标
    xmax = np.minimum(x1max, x2max)  # 右下角的横坐标
    ymax = np.minimum(y1max, y2max)  # 右下角的纵坐标

    inter_h = np.maximum(ymax - ymin, 0.)
    inter_w = np.maximum(xmax - xmin, 0.)
    intersection = inter_h * inter_w
    # area = max(0, xmax - xmin) * max(0, ymax - ymin)  # 可以用一行代码算出来相交矩形的面积
    # print("G∩P的面积是:{}".format(intersection))
    IoU = intersection / parea 
    
    return IoU

def merge_RecA_RecB(predicted_bound,ground_truth_bound):
    x1min, y1min =  predicted_bound.x,predicted_bound.y
    x1max, y1max=   predicted_bound.x+predicted_bound.width,predicted_bound.y+predicted_bound.height
    # print("预测框P的坐标是：({}, {}, {}, {})".format(pxmin, pymin, pxmax, pymax))
    x2min, y2min =  ground_truth_bound.x,ground_truth_bound.y
    x2max, y2max=ground_truth_bound.x+ground_truth_bound.width,ground_truth_bound.y+ground_truth_bound.height

    # 计算外接框
    xmin = min(x1min, x2min)
    ymin = min(y1min, y2min)
    xmax = max(x1max, x2max)
    ymax = max(y1max, y2max)
    width,height=xmax-xmin,ymax-ymin
    return Rectangle(xmin,ymin,width,height)

def load_calib(yaml_file):
    cv_file = cv2.FileStorage(yaml_file, cv2.FILE_STORAGE_READ) 
    calib=dict()
    calib["Extrinsic"]=cv_file.getNode('CameraExtrinsicMat').mat()
    calib["Intrinsic"]=cv_file.getNode('CameraMat').mat()
    cv_file.release()
    return calib
def drawrectangleIOU(raw_img, Rect, class_id, y,iou):
    
    cv2.rectangle(raw_img, (Rect.x,Rect.y),(Rect.x+Rect.width,Rect.y+Rect.height),
                            (255, 0,  0),
                              2)
    labels = " IOU:{:.1f} y:{:.1f}".format(iou,y)
    labels = "{}".format( class_id) + labels
    print_size=0.5
    thickness=1
    color=(0,0,0)
    (width,height),baseline=cv2.getTextSize(labels,cv2.FONT_HERSHEY_SIMPLEX,fontScale=print_size, thickness=thickness)
    top = max(Rect.y, height)
    cv2.rectangle(raw_img, (Rect.x, top - height),
              (Rect.x + width, top + baseline),
              (255, 255, 255), 
              cv2.FILLED)
    cv2.putText(raw_img, labels, (Rect.x, top),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=print_size,color=color,thickness=thickness)
    return raw_img

def ObjectInImage(image,pc_list):
    
    calib=load_calib(calib_file)

    l2c = np.array(calib["Extrinsic"]).reshape(4,4)
    l2c = np.linalg.inv(l2c)
    intri = np.array(calib["Intrinsic"])
    intrinisic = np.eye(4)
    intrinisic[:3, :3] = intri
    lidar = np.concatenate((pc_list, np.ones((pc_list.shape[0], 1))), axis=1)

    coords = l2c @ lidar.T
    coords = intrinisic @ coords
    coords = coords.reshape(4, -1)

    indices = coords[2] > 0

    coords = coords.T[indices]

    coords[:, 2] = np.clip(coords[:, 2], a_min=1e-5, a_max=1e5)
    coords[:, 0] /= coords[:, 2]
    coords[:, 1] /= coords[:, 2]

    kept = (coords.T[0] > 0) & (coords.T[1] > 0) & (coords.T[0] < 1600) & (coords.T[1] < 1200)

    coords = coords[kept]
    depth = coords[:,2]

    coords = coords[:, :2].astype(int)
    rect=Rectangle()
    (rows,cols,channels) = image.shape
    for point in coords:
        x,y=point
        if 0<=x<=cols and 0<=y<=rows:
            cv2.circle(image, (point), 0, (0,255,255), 3)
            get_point=True
        else:
            get_point=False
    if len(coords)>0:
        print(f'coords:{len(coords)}')
        minx=np.min(coords[:,0])+adjust_x
        maxx=np.max(coords[:,0])+adjust_x
        miny=np.min(coords[:,1])+adjust_x
        maxy=np.max(coords[:,1])+adjust_x
        minx= 1 if minx== 0 else minx
        maxy=1 if maxy==0 else maxy
        width= 1 if maxx-minx==0 else maxx-minx
        height= 1 if maxy-miny==0 else maxy-miny
        rect=Rectangle(minx,miny,width,height)
        cv2.rectangle(image,(minx,miny),(maxx,maxy),(0,255,0),2)
    return rect,image

def callback(ImageObj,DetectedObjects):

    img = ImageObj.roi_image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
    except CvBridgeError as e:
        print(e)
    if isinstance(cv_image,cv2.Mat):
        print("mat")
    # cv2.imshow('color_image',det_image)
    # cv2.waitKey(1)
    rospy.loginfo("get info")
    objects =DetectedObjects.objects
    if len(objects)<1:
        rospy.loginfo(f"objects size: {len(objects)} ")
    # calib_file='/home/cqjtu/Documents/catktin_ros/src/cfg/65.json'
    # calib=load_calib(calib_file)
    msg = DetectedObjectArray()
    msg.header=DetectedObjects.header
    msg.image=bridge.cv2_to_imgmsg(cv_image)
    fused_objects=[]
    for img_obj in ImageObj.obj:
        fused_object=DetectedObject()
        rect_img=Rectangle(img_obj.x,img_obj.y,img_obj.width,img_obj.height)
        # rect_img_s=rect_img.get_area()
        fused_object.x=rect_img.x
        fused_object.y=rect_img.y
        fused_object.width=rect_img.width
        fused_object.height=rect_img.height
        # print("image rect{} {} {} {}".format(img_obj.x,img_obj.y,img_obj.width,img_obj.height))
        # cv2.rectangle(cv_image,(rect_img.x,rect_img.y),(rect_img.x+rect_img.width,rect_img.y+rect_img.height),(0,0,255),2)
        fused_object.score=img_obj.score
        fused_object.id =img_obj.class_id

        for obj in objects:
            pc=pc2.read_points(obj.pointcloud,field_names=("x","y","z"),skip_nans=True)
            pc_list=np.array(list(pc))
            print(f'cluster points:{len(pc_list)}')
            rect,cv_image=ObjectInImage(cv_image,pc_list)
            if rect.get_area()<1:
                print("image rect{} {} {} {}".format(rect.x,rect.y,rect.width,rect.height))
                print(f"obj behided back ..")
                continue
            if obj.pose.position.y <1.5:
                print(f"too close .....{obj.pose.position.y}")
                continue
            if obj.pose.position.x>9.5:
                print(f"too far alway...{obj.pose.position.x}")
                continue
            #iou
            Rect_final=Rectangle()
            iou=calculate_IoU(rect,rect_img)
            overlap=merge_RecA_RecB(rect,rect_img)
            if iou>0:
                print(f"iou:{iou}")
            if iou>overlap_threshold_:
                fused_object.pose.position = obj.pose.position
                fused_object.dimensions = obj.dimensions
                fused_object.velocity = obj.velocity
                # Rect_final.x = min(rect.x ,rect_img.x) 
                # Rect_final.y = min(rect.y , rect_img.y) 
                # Rect_final.width = int(rect.width + rect_img.width - overlap.width)
                # Rect_final.height = int(rect.height + rect_img.height - overlap.height)
                Rect_final=rect_img
                print(f"fiw:{Rect_final.width} liw:{rect.width } imgw:{rect_img.width} ovre{overlap.width}")
                cv_image=drawrectangleIOU(cv_image, Rect_final, fused_object.id, obj.pose.position.y ,iou)
        fused_objects.append(fused_object)
    msg.objects=DetectedObjects
    

    image_msg=bridge.cv2_to_imgmsg(cv_image)
    imagepub.publish(image_msg)
    fusedpub.publish(msg)


if __name__ == '__main__':
    

    rospy.init_node('get_image', anonymous=True)
    fusedpub = rospy.Publisher('fuse_msg_topic', DetectedObjectArray, queue_size=5)
    imagepub=rospy.Publisher("image_topic",Image,queue_size=1)

    color = message_filters.Subscriber("/detect_msg_image", ImageObj)
    depth = message_filters.Subscriber("/detection/lidar_detector/objects", DetectedObjectArray)
    # color_depth = message_filters.ApproximateTimeSynchronizer([color, depth], 10, 1, allow_headerless=True)  # 接近时间同步
    color_depth = message_filters.ApproximateTimeSynchronizer([color, depth], 1,slop=0.1)  # 绝对时间同步
    color_depth.registerCallback(callback)  
    
    #同时订阅/camera/color/image_raw和/camera/aligned_depth_to_color/image_raw话题，并利用message_filters实现话题同步，共同调用callback
    rospy.spin()
