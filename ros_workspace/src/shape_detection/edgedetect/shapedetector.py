# import the necessary packages
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
			x, y, w, h = cv2.boundingRect(approx)
			ar = w / float(h)

			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
			if 0.75 <= ar <= 1.35:
				shape = "square"
				self.s3 += 1
			else:
				shape = "rectangle"
				self.s4 += 1

		print(self.s1, "lines,", self.s2, "triangles,", self.s3, "Squares,", self.s4, "rectangles,", self.s5, "circles")
		# return the name of the shape
		return shape
