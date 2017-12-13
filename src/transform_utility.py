# Transform Utility

import numpy as np

# Project v onto w
def projectOnto(v, w):
	normw = np.linalg.norm(w)
	alongW = (np.dot(v,w)/(normw**2))*w
	normalToW = v - alongW

	return alongW, normalToW

# Calling projectOnto(windowCenter, windowNormal)
# splits the drone's position from the window into the perpendicular
# component (normalToW) pointing away from the drone toward the
# windowNormal line, and the component along the windowNormal line
# pointing from the drone to the window plane
# Assuming windowCenter, windowNormal are in the drone's frame

def getNormalFromWindowPts(pt1, pt2, pt3):
	pass
