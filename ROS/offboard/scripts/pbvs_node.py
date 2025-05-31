#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
from vision_utils import compute_homography, recover_from_homography, estimate_homography, rodriguez, pbvs_controller

# Variables globales
ref_image = None
current_image = None
bridge = CvBridge()

REF_IMAGE_NAME = "/home/noe/catkin_ws/src/offboard/image_reference/reference_image.jpg"

def image_callback(msg):
    global current_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return
    current_image = cv_image

def main():
    global ref_image, current_image

    rospy.init_node('pbvs_node', anonymous=True)

    # Cargar la imagen de referencia
    ref_image = cv2.imread(REF_IMAGE_NAME, cv2.IMREAD_COLOR)
    if ref_image is None:
        rospy.logerr("No se pudo cargar la imagen de referencia desde: " + REF_IMAGE_NAME)
        return

    # Suscribirse al tópico de la cámara
    image_sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, image_callback)

    # Publicadores
    homography_pub = rospy.Publisher("/homograpy_numerical", Float64MultiArray, queue_size=10)
    ck_t_ct_pub = rospy.Publisher("/ck_t_ct", Float64MultiArray, queue_size=10)
    n_pub = rospy.Publisher("/n_plane_vector", Float64MultiArray, queue_size=10)
    Uv_pub = rospy.Publisher("/Uv", Float64MultiArray, queue_size=10)
    Uw_pub = rospy.Publisher("/Uw", Float64MultiArray, queue_size=10)
    d_pub = rospy.Publisher("/d_value", Float64, queue_size=10)
    

    # Inicializar la matriz de calibración de la cámara y otras matrices
    K = np.array([[554.382713, 0.0, 320.0],
                  [0.0, 554.382713, 240.0],
                  [0.0, 0.0, 1.0]])  # Matriz de calibración de la cámara
    H = np.eye(3)  # Matriz de homografía
    R = np.eye(3)  # Rotación entre el marco actual y el marco de referencia
    t = np.zeros((3, 1))  # Traslación entre el marco actual y el marco de referencia
    n = np.zeros(3)
    d = 0.0
    counter = 0
    homography_solution = None

    # Obtener los argumentos desde la terminal
    #lambdav = rospy.get_param('~lambdav', 1.0)
    #lambdaw = rospy.get_param('~lambdaw', 1.0)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if current_image is not None:
            H = estimate_homography(ref_image, current_image, K, counter)
            if H is not None:
                R, t, n, d, homography_solution = recover_from_homography(H, K, counter, homography_solution)
                u = rodriguez(R)
                Uv, Uw = pbvs_controller(R, t, u, lambdav=0.5, lambdaw=0.5)

                # Publicar los resultados
                homography_pub.publish(Float64MultiArray(data=H.flatten()))
                ck_t_ct_pub.publish(Float64MultiArray(data=t.flatten()))
                n_pub.publish(Float64MultiArray(data=n.flatten()))
                d_pub.publish(Float64(data=d))
                Uv_pub.publish(Float64MultiArray(data=Uv.flatten()))
                Uw_pub.publish(Float64MultiArray(data=Uw.flatten()))

        counter += 1
        #rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception caught")