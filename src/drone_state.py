#/usr/bin/env python

# A list of drone states because I'm tired of defining them in every program

class DroneStatus(object):
	Emergency = 0
	Init	  = 1
	Landed	  = 2
	Flying	  = 3
	Hover	  = 4
	Test	  = 5
	Takeoff	  = 6
	GoToHover = 7
	Landing	  = 8
	Looping	  = 9
