#!/usr/bin/env python3
import argparse
import time
import socket,os,struct, time
import numpy as np
import cv2

import rospy
from geometry_msgs.msg import Twist

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

def rx_bytes(size):
  data = bytearray()
  while len(data) < size:
    data.extend(client_socket.recv(size-len(data)))
  return data


##Configuration :

parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n",  default="192.168.0.200", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

deck_port = args.p
deck_ip = args.n

print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((deck_ip, deck_port))
print("Socket connected")

imgdata = None
data_buffer = bytearray()

#Configuration aruco
dict_aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dict_aruco, parameters)

#Configuration matrice de distorsion caméra
mtx = np.array([[131.6433353, 0, 163.69754607],[0,139.72452379,167.05022501],[0, 0, 1]])
dist = np.array([[-0.30337915,0.14980387,0.001463824,-0.3813348,-0.05521002]])

#mtx = np.array([[178.33085351, 0, 209.27829488],[0,224.23659555,148.61063435],[0, 0, 1]])
#dist = np.array([[-1.15166742,0.22755013,0.00245517,-0.31471369,0.12224225]])

#Début programme
start = time.time()
count = 0



if __name__ == "__main__":
    rospy.init_node("Aruco_detector")
    rospy.loginfo("Aruco detector has been started")

    pub = rospy.Publisher("Aruco_Pose", Twist, latch=True, queue_size=1)

    compteur = 0

    while not rospy.is_shutdown():
        twist = Twist()

        # First get the info
        packetInfoRaw = rx_bytes(4)
        #print(packetInfoRaw)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
        #print("Length is {}".format(length))
        #print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
        #print("Function is 0x{:02X}".format(function))

        imgHeader = rx_bytes(length - 2)
        #print(imgHeader)
        #print("Length of data is {}".format(len(imgHeader)))
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        if magic == 0xBC:
        #print("Magic is good")
        #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
        #print("Image format is {}".format(format))
        #print("Image size is {} bytes".format(size))

        # Now we start rx the image, this will be split up in packages of some size
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = rx_bytes(4)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = rx_bytes(length - 2)
                imgStream.extend(chunk)
            
            count = count + 1
            meanTimePerImage = (time.time()-start) / count
            #print("{}".format(meanTimePerImage))
            #print("{}".format(1/meanTimePerImage))

            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
                bayer_img.shape = (244, 324)
                #color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                
                if args.save:
                    cv2.imwrite(f"stream_out/raw/img_{count:06d}.png", bayer_img)
                    #cv2.imwrite(f"stream_out/debayer/img_{count:06d}.png", color_img)

                markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(bayer_img)
                if not markerIds is None:

                    rvecs, tvecs, trash = my_estimatePoseSingleMarkers(markerCorners, 12.4, mtx, dist)
                    #cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
                    for idx in range(len(markerIds)):

                        cv2.drawFrameAxes(bayer_img,mtx,dist,rvecs[idx],tvecs[idx],5)
                        print('marker id:%d, pos_x = %f,pos_y = %f, pos_z = %f' % (markerIds[idx],tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]))  

                        if compteur<1:
                        #Actualisation du msg de pose du Marker :
                            twist.linear.x = tvecs[idx][0]
                            twist.linear.y = tvecs[idx][1]
                            twist.linear.z = tvecs[idx][2]

                            twist.angular.x = rvecs[idx][0]
                            twist.angular.y = rvecs[idx][1]
                            twist.angular.z = rvecs[idx][2]

                            pub.publish(twist)
                            compteur += 1


                cv2.aruco.drawDetectedMarkers(bayer_img, markerCorners, markerIds)
                cv2.imshow('Raw', bayer_img)
                #cv2.imshow('Color', color_img)
                cv2.waitKey(1)
                
            else:
                with open("img.jpeg", "wb") as f:
                    f.write(imgStream)
                nparr = np.frombuffer(imgStream, np.uint8)
                decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                cv2.imshow('JPEG', decoded)
                cv2.waitKey(1)



