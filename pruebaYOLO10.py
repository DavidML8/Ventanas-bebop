#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cvzone
import math
import numpy as np

controlImagen = 0
levantar = 1
rotacional = 0
controlRotacional = 0
checarRotacion = 0
buscar = 0

# Kx = (1 / 655)
Kx = (1 / 900)
Ky = (1 / 300)
Kz = (1 / 250)
# 480x856
centerX, centerY = int(856 // 2), int(480 // 2)
ancho = 260

centrar = [0] * 4
centradoHorizontal = False
centradoVertical = False
centradoFrente = False
centrado = False

model = YOLO('orange50n.pt')

classNames = ["window"]

numVentanas = 5
ventanasPasadas = 0

direccionGiro = [[0, 0, 0, 0], [0.2, 1.4, 0, 0], [0.2, 0.7, 0, 0], [0.2, 0.7, 0, 0], [0.2, 0.7, 0, 0], [0.2, 0.7, 0, 0], [0.2, 0.7, 0, 0], [0.2, 0.7, 0, 0], [0.2, 0.7, 0, 0], [0.2, 0.7, 0, 0]]
direccionBuscar  = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


class BebopController:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("bebop/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=2)
        self.takeoff_pub = rospy.Publisher("bebop/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("bebop/land", Empty, queue_size=1)
        self.twist_msg = Twist()
        self.takeoff_msg = Empty()
        self.land_msg = Empty()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            global controlImagen
            global imagen
            imagen = cv_image.copy()
            controlImagen = 1
        except CvBridgeError as e:
            print(e)

        # cv2.imshow("Bebop Camera", cv_image)
        # cv2.waitKey(3)

    def move(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        self.twist_msg.linear.x = linear_x
        self.twist_msg.linear.y = linear_y
        self.twist_msg.linear.z = linear_z
        self.twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(self.twist_msg)

    def takeoff(self):
        self.takeoff_pub.publish(self.takeoff_msg)
        global levantar
        levantar = 1

    def land(self):
        self.land_pub.publish(self.land_msg)

    def lanzame(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0, tiempo=0.0):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < tiempo:
            self.twist_msg.linear.x = linear_x
            self.twist_msg.linear.y = linear_y
            self.twist_msg.linear.z = linear_z
            self.twist_msg.angular.z = angular_z
            self.cmd_vel_pub.publish(self.twist_msg)

    def giro(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0, tiempoGiro=0.0):
        giro_time = rospy.get_time()
        while (rospy.get_time() - giro_time) < tiempoGiro:
            self.twist_msg.angular.z = angular_z
            self.twist_msg.linear.x = linear_x
            self.twist_msg.linear.y = linear_y
            self.twist_msg.linear.z = linear_z
            self.cmd_vel_pub.publish(self.twist_msg)


if __name__ == "__main__":
    rospy.init_node("bebop_controller")
    controller = BebopController()

    controller.move(linear_z=0)
    rospy.sleep(1)
    # controller.takeoff()
    rospy.sleep(2)
    controller.lanzame(linear_z=0.7, tiempo=1.7)
    rospy.sleep(2)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if controlImagen == 1:
            results = model(imagen, stream=True)
            for r in results:
                print("num Res: ", len(r))
                boxes = r.boxes
                print(boxes)
                print("num Box: ", len(boxes))
                if checarRotacion == 1:
                    if  len(r) == 0:
                        buscar = 1
                        print("Girar")
                    if buscar == 1:
                        print("Buscando")
                else:
                    for box in boxes:
                        # x1, x2, y1, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = box.xyxy[0]
                        # x, y, w, h = box.xywh[0]
                        # bbox = int(x), int(y), int(w), int(h)
                        x1, x2, y1, y2 = int(x1), int(x2), int(y1), int(y2)
                        w, h = int(x2 - x1), int(y2 - y1)
                        aspectRatio = w / h
                        print("x1; ", x1, " x2: ", x2, " y1: ", y1, " y2: ", y2, " w: ", w, " h: ", h)
                        # cvzone.cornerRect(img, bbox)
                        conf = math.ceil(box.conf[0] * 100) / 100
                        cls = int(box.cls[0])
                        if conf > 0.80:
                            cVx, cVy = (x1 + (w // 2)), (y1 + (h // 2))
                            cVy = cVy + 100
                            # cv2.rectangle(imagen, (x1, y1), (x2, y2), (255, 0, 255), 3)
                            # cvzone.putTextRect(imagen, f'{conf}', (max(0, x1), max(35, y1 - 20)), scale=0.8, thickness=1)
                            # cv2.circle(imagen, ((x1 + (w // 2)), ((y1 + (h // 2)) + 100)), 8, (0, 0, 255), -1)
                            # ROI = imagen[y1:y2, x1:x2]
                            # cv2.imshow("ROI", ROI)
                            if levantar == 1:
                                errorPixeles = 30
                                errorAncho = 10
                                # cv2.rectangle(imagen, ((cVx - errorPixeles), (cVy - errorPixeles)), ((cVx + errorPixeles), (cVy + errorPixeles)), (0, 255, 0), 2)
                                # cv2.line(cv_image, (cVx, cVy), (centerX, centerY), (255, 0, 0), 2)
                                dX, dY, dW = int(centerX - cVx), int(centerY - cVy), int(ancho - w)
                                # print("diferencia X: ", dX, " diferencia Y: ", dY)
                                velocidadX = Kx * dX
                                velocidadY = Ky * dY
                                velocidadZ = Kz * dW
                                velocidadX = np.clip(velocidadX, -0.28, 0.28)
                                velocidadY = np.clip(velocidadY, -0.35, 0.35)
                                velocidadZ = np.clip(velocidadZ, -0.3, 0.3)
                                velocidadX = round(velocidadX, 3)
                                velocidadY = round(velocidadY, 3)
                                velocidadZ = round(velocidadZ, 3)
                                print("VelX: ", velocidadX, "     VelY: ", velocidadY)
                                # cvzone.putTextRect(imagen, f'{dX} {velocidadX}', ((cVx - 40), (cVy - 15)), scale=0.8, thickness=1)
                                # cvzone.putTextRect(imagen, f'{dY} {velocidadY}', ((cVx + 20), (cVy + 15)), scale=0.8, thickness=1)
                                # cvzone.putTextRect(imagen, f'{dW} {velocidadZ}', ((cVx - 40), (cVy + 50)), scale=0.8, thickness=1)
                                if (cVx + errorPixeles) > centerX > (cVx - errorPixeles):
                                    controller.move(linear_y=0)
                                    rate.sleep()
                                    centradoHorizontal = True
                                    print("horizontal")
                                    # 480x856
                                    # cv2.circle(cv_image, (50, 50), 20, (0, 255, 0), -1)
                                    # cv2.putText(cv_image, "X", (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)
                                else:
                                    controller.move(linear_y=velocidadX)
                                    rate.sleep()
                                    centradoHorizontal = False
                                    # cv2.circle(cv_image, (50, 50), 20, (0, 0, 255), -1)
                                    # cv2.putText(cv_image, "X", (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)
                                if (cVy + errorPixeles) > centerY > (cVy - errorPixeles):
                                    controller.move(linear_z=0)
                                    rate.sleep()
                                    centradoVertical = True
                                    print("vertical")
                                    # cv2.circle(cv_image, (100, 50), 20, (0, 255, 0), -1)
                                    # cv2.putText(cv_image, "Y", (90, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)
                                else:
                                    controller.move(linear_z=velocidadY)
                                    rate.sleep()
                                    centradoVertical = False
                                if (ancho + errorAncho) > w > (ancho - errorAncho):
                                    controller.move(linear_x=0)
                                    rate.sleep()
                                    centradoFrente = True
                                    # cv2.circle(cv_image, (100, 50), 20, (0, 255, 0), -1)
                                    # cv2.putText(cv_image, "Y", (90, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)
                                else:
                                    controller.move(linear_x=velocidadZ)
                                    rate.sleep()
                                    centradoFrente = False
                                    # cv2.circle(cv_image, (100, 50), 20, (0, 0, 255), -1)
                                    # cv2.putText(cv_image, "Y", (90, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)
                                if centradoHorizontal == True and centradoVertical == True and centradoFrente:
                                    centrado = True
                                else:
                                    centrado = False

                                centrar.insert(0, centrado)
                                centrar.pop()

                                if all(historialCentro is True for historialCentro in centrar):
                                    print("Centradisimoooooooo")
                                    controller.lanzame(linear_x=0.5, tiempo=1.7)
                                    rospy.sleep(2)
                                    controller.lanzame(linear_y=direccionGiro[ventanasPasadas][2], tiempo=direccionGiro[ventanasPasadas][3])
                                    rospy.sleep(2)
                                    controller.giro(angular_z=direccionGiro[ventanasPasadas][0], tiempoGiro=direccionGiro[ventanasPasadas][1])
                                    # 1.8
                                    rospy.sleep(2)
                                    if ventanasPasadas == 0:
                                        checarRotacion = 1
                                    ventanasPasadas += 1
                                if ventanasPasadas == numVentanas:
                                    print('land')
                                    controller.land()
                                    rospy.sleep(1)
                                    print('hasta la próxima [música de skrillex suena]')
                            else:
                                cVx, cVy = centerX, centerY
                        else:
                            print("Giroooooo Zepelli")
            cv2.imshow("imagen", imagen)
            cv2.waitKey(1)
