import cv2
import numpy as np
from media_pipe import get_points
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from wpb_home_behaviors.msg import Coord
import math

_cv_bridge = CvBridge()

depth = None
lenth = 0.6

pixel_x = 212
norm_pixel_x = 0

pixel_y = 216
z = 1000
d_angle = 0
lamda = 0.2
tan = 1 / lenth
alpha = 10


def get_depth(x, y):
    global depth
    index_x = int(x)
    index_y = int(y)
    if isinstance(depth, np.ndarray):
        return depth[index_y - alpha:index_y + alpha, index_x - alpha:index_x + alpha].mean()
    else:
        return -1


def media_pipe(image):
    # here
    key_point = get_points(image, show=True)
    if key_point is not None:
        key_point[0] *= 424
        key_point[1] *= 512
    return key_point


def getPositionCallback(image_msg):
    global pixel_x, norm_pixel_x, pixel_y, z, d_angle
    assert isinstance(image_msg, Image)
    # transfer to numpy array
    image = _cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    key_points = media_pipe(image)

    if key_points is not None:
        pixel_x = (1 - lamda) * key_points[0] + lamda * pixel_x
        pixel_y = (1 - lamda) * key_points[1] + lamda * pixel_y - 20
        z = (1 - lamda) * get_depth(pixel_x, pixel_y) + lamda * z
        if math.isnan(pixel_x) or math.isnan(pixel_y):
            return
    elif norm_pixel_x > 180:
        pixel_x = (1 - lamda) * 212 + lamda * pixel_x
    elif norm_pixel_x < -180:
        pixel_x = (1 - lamda) * 212 + lamda * pixel_x
    else:
        return
    z_actual = z.item() / 1000
    norm_pixel_x = pixel_x - 212
    d_angle = - math.atan(norm_pixel_x / 256 / tan) if abs(norm_pixel_x) > 10 else 0
    x = (norm_pixel_x - 212) / 256 * z_actual / tan
    pose_msg = Coord()
    pose_msg.name = 'human'
    pose_msg.x = [x]
    # pose_msg.y = [y]
    pose_msg.z = [z_actual]
    pose_msg.probability = [d_angle]
    print(pose_msg)
    # rospy.loginfo("Publsh person message[%s]", pose_msg.name)
    human_point_pub.publish(pose_msg)
    rate.sleep()


def show(image_msg):
    global depth
    image = _cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    _, depth = cv2.threshold(image, 4000, 4000, cv2.THRESH_TRUNC)
    # depth = cv2.resize(image, (106, 128))
    # Xmin = np.min(image)
    # Xmax = np.max(image)
    # # 将数据映射到[-1,1]区间 即a=-1，b=1
    # a = 0
    # b = 255
    # image = a + (b - a) / (Xmax - Xmin) * (image - Xmin)
    # cv2.imshow('hh', image)
    # cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('Image_process', anonymous=True)
    rate = rospy.Rate(500)

    human_point_pub = rospy.Publisher('/Human_pose', Coord, queue_size=1)
    rospy.Subscriber("/kinect2/sd/image_color_rect", Image, getPositionCallback)
    rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, show)
    #
    # cap = cv2.VideoCapture(0)
    # while cap.isOpened():
    #     success, image = cap.read()
    #     if not success:
    #         print("Ignoring empty camera frame.")
    #         # If loading a video, use 'break' instead of 'continue'.
    #         cv2.destroyAllWindows()
    #         cap.release()
    #         break
    #     # cv2.imshow('w',image)
    #     # cv2.waitKey(1)
    #     key_points = media_pipe(image)
    #     pose_msg = Coord()
    #     if key_points is not None:
    #         pose_msg = Coord()
    #         pose_msg.name = 'human'
    #         pose_msg.x = [key_points[0]]
    #         pose_msg.y = [key_points[1]]
    #         pose_msg.z = [key_points[2]]
    #         pose_msg.probability = [key_points[3]]
    #         print('x； ', pose_msg.x, end="； ")
    #         print('y； ', pose_msg.y, end="; ")
    #         print('z； ', pose_msg.z)
    #         human_point_pub.publish(pose_msg)
    #         # rospy.loginfo("Publsh person message[%s]", pose_msg.name)

    rospy.spin()
