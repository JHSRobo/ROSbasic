#!/usr/bin/env python
# USAGE
# needs ros support but no one wants to do it.

# import the necessary packages
import imutils
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from crack_finder.srv import crack_finder
from std_msgs.msg import Header
import cv2

class ShapeDetector:
	def __init__(self):
		self.s1 = self.s2 = self.s3 = self.s4 = self.s5 = 0
		pass

	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)

		if len(approx) == 4:
			# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h)

			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
			if 0.75 <= ar <= 1.35:
				shape = "square"
				self.s3 = self.s3 + 1
			else:
				shape = "rectangle"
				self.s4 = self.s4 + 1

		# return the name of the shape
		return shape

def find_the_shape():
	msg = crack_finder()

	header = Header()

	# update message headers
	header.stamp = rospy.Time.now()
	header.frame_id = 'humidity_data'
	msg.header = header

    try:
        data = rospy.wait_for_message("/rov/image_raw", Image, timeout=5)
    except rospy.ROSException:
        msg.length = None
		return msg
    else:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data.data, desired_encoding="passthrough")
        while True:
        	# Capture frame-by-frame
        	resized = imutils.resize(frame, width=640)
			ratio = frame.shape[0] / float(resized.shape[0])
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			#blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
			lower_red = np.array([103,140,50])
			upper_red = np.array([130,255,255])
			k = cv2.waitKey(33)
			if k==97:
				#if you dare to touch the A key it will stop the program if it doesnt stop automatically
				break
			elif k==255:
				continue
			mask = cv2.inRange(hsv, lower_red, upper_red)
			res = cv2.bitwise_and(frame,frame, mask= mask)
			#thresh = cv2.adaptiveThreshold(frame,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,11,2)
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cnts = cnts[0] if imutils.is_cv2() else cnts[1]
			sd = ShapeDetector()
			idx = 0
			for c in cnts:
				# compute the center of the contour, then detect the name of the
				# shape using only the contour
				M = cv2.moments(c)
				cX = int((M["m10"] / (M["m00"] + 1e-7)) * ratio)
				cY = int((M["m01"] / (M["m00"] + 1e-7)) * ratio)
				shape = sd.detect(c)
				x,y,w,h = cv2.boundingRect(c)
				if w > 4 and h > 4:
					idx+=1
					new_img = frame[y:y+h,x:x+w]
					cv2.imshow('croppo',new_img)
					cv2.imwrite('croppo.png', new_img)
				if shape == "rectangle":
					cv2.imwrite("is-it-a-rectange.png", mask)
					sqtest = cv2.imread("is-it-a-rectange.png", 0)
					y = 110
					x = 120
					h = 285
					w = 380
					crop = sqtest[y:y+h, x:x+w]
					thresh = cv2.adaptiveThreshold(crop,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,11,2)
					cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
					cnts = cnts[0] if imutils.is_cv2() else cnts[1]
					sd = ShapeDetector()
					for c in cnts:
						# compute the center of the contour, then detect the name of the
						# shape using only the contour
						M = cv2.moments(c)
						cX = int((M["m10"] / (M["m00"] + 1e-7)) * ratio)
						cY = int((M["m01"] / (M["m00"] + 1e-7)) * ratio)
						shape = sd.detect(c)
						if shape == "rectangle":
							c = c.astype("float")
							c *= ratio
							c = c.astype("int")
							#cv2.drawContours(thresh, [c], -1, (0, 255, 0), 2)
							#cv2.putText(thresh, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 2)
							cv2.imshow('thresh',thresh)
							cv2.imwrite("foundthecrack.png", frame)
							#cv2.imwrite("foundthecrack.png", frame)
							image4 = frame
							#gray3=cv2.cvtColor(image4,cv2.COLOR_BGR2GRAY)
							#edged = cv2.Canny(image4, 10, 250)
							lower = np.array([0, 0, 0])
							upper = np.array([40, 40, 40])
							thresh100 = cv2.inRange(image4, lower, upper)
							#blurred3 = cv2.GaussianBlur(gray3, (5, 5), 0)
							#thresh3 = cv2.threshold(blurred3, 60, 255, cv2.THRESH_BINARY)
							cnts = cv2.findContours(thresh100.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
							cnts = cnts[0] if imutils.is_cv2() else cnts[1]
							cv2.imshow('blackcrop', thresh100)
							for c in cnts:
								x,y,w,h = cv2.boundingRect(c)
								if w > 200 and h > 200:
									new_img1=image4[y:y+h,x:x+w]
									cv2.imshow('blackcrop.png', new_img1)
									cv2.imwrite('blackcrop.png', new_img1)
							heightblk = np.size(new_img1, 0)
							widthblk = np.size(new_img1, 1)
							heightblu = np.size(new_img, 0)
							widthblu = np.size(new_img, 1)
							actualheightblk = 31.0/heightblk
							actualwidthblk = 31.0/widthblk
							actualheightblu = actualheightblk * heightblu
							actualwidthblu = actualwidthblk * widthblu

							msg.length = max(actualheightblu, actualwidthblu)
							return msg
					# multiply the contour (x, y)-coordinates by the resize ratio,
					# then draw the contours and the name of the shape on the image
					c = c.astype("float")
					c *= ratio
					c = c.astype("int")
					#cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
					#cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 2)

				# multiply the contour (x, y)-coordinates by the resize ratio,
				# then draw the contours and the name of the shape on the image
				c = c.astype("float")
				c *= ratio
				c = c.astype("int")
	msg.length = None
	return msg


def listener():
    rospy.init_node("crack_finder")
    rospy.Service('crack_finder', shape_detect, find_the_shape)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
