# import the necessary packages
import cv2
import numpy as np


class ShapeDetector:
	def __init__(self):
		self.s1 = self.s2 = self.s3 = self.s4 = self.s5 = 0
		pass

	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)

		# if the shape is a triangle, it will have 3 vertices
		# if len(approx) == 2:
		# shape = "line"
		# s1 = s1 + 1
		if len(approx) == 3:
			shape = "triangle"
			self.s2 = self.s2 + 1

		# if the shape has 4 vertices, it is either a square or
		# a rectangle
		elif len(approx) == 4:
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

		# if the shape is a pentagon, it will have 5 vertices
		# elif len(approx) == 5:
			# shape = "pentagon"

		# otherwise, we assume the shape is a circle
		else:
			shape = "circle"
			self.s5 += 1

		# return the name of the shape
		print(self.s1, "lines,", self.s2, "triangles,", self.s3, "Squares,", self.s4, "rectangles,", self.s5, "circles")
		c1 = str(self.s1)
		c2 = str(self.s2)
		c3 = str(self.s3)
		c4 = str(self.s4)
		c5 = str(self.s5)
		blackdrop = np.zeros((1024, 512, 3), np.uint8)
		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(blackdrop,c2,(10,120), font, 4,(255,255,255),2,cv2.LINE_AA)
		cv2.putText(blackdrop,c3,(10,230), font, 4,(255,255,255),2,cv2.LINE_AA)
		p1 = (155, 40)
		p2 = (120, 120)
		p3 = (170, 120)
		# Drawing the triangle with the help of lines
		#  on the black window With given points
		# cv2.line is the inbuilt function in opencv library
		cv2.line(blackdrop, p1, p2, (255, 0, 0), 3)
		cv2.line(blackdrop, p2, p3, (255, 0, 0), 3)
		cv2.line(blackdrop, p1, p3, (255, 0, 0), 3)
		cv2.rectangle(blackdrop,(125,155),(195,225),(0,255,0),-1)
		cv2.putText(blackdrop,c4,(10,340), font, 4,(255,255,255),2,cv2.LINE_AA)
		cv2.rectangle(blackdrop,(155,265),(165,335),(255,255,0),-1)
		cv2.putText(blackdrop,c5,(10,450), font, 4,(255,255,255),2,cv2.LINE_AA)
		cv2.circle(blackdrop,(160,410), 40, (0,0,255), -1)
		cv2.imshow("blackdrop", blackdrop)
		return shape
