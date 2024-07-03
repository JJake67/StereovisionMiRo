#!/usr/bin/python3
#
# TAKEN FROM MIRO EXAMPLES, LETS YOU CONTROL HIM WITH WASD
# WONT BE ABLE TO MOVE FORWARD SOMETIMES UNLESS YOU DO SOME FLAG STUFF

import numpy as np
import sys, os, time
import curses
import argparse

import miro2 as miro

################################################################

class client:

	def loop(self, stdscr):

		# turn off delay
		stdscr.nodelay(1)

		# output
		dtheta = 2.0
		dy = 0.2
		timeout = 0.5

		# use getch() once so that print works (why?)
		c = stdscr.getch()

		# instructions
		print ("Use arrow keys to control MIRO directly.\r")
		print ("Press ESCAPE to exit\r")

		# loop
		while self.interf.is_active():

			# this is a terrible way of getting the keydown
			# state, but it seems that it's a challenge to
			# do it well in Python for some reason.
			c = stdscr.getch()

			if c != -1:

				if c == 260:
					self.interf.set_vel(0, dtheta, timeout)
				elif c == 261:
					self.interf.set_vel(0, -dtheta, timeout)
				elif c == 259:
					self.interf.set_vel(dy, 0, timeout)
				elif c == 258:
					self.interf.set_vel(-dy, 0, timeout)
				elif c == 32:
					self.interf.set_vel(0, 0, timeout)
				elif c == 27:
					self.interf.term()
				else:
					print (c)
					pass

			# flush buffer
			while c != -1:
				c = stdscr.getch()

			# yield
			time.sleep(0.1)

		# finalise report
		print ("\n\nexit...")
		print (" ")

	def __init__(self):

		# args
		parser = argparse.ArgumentParser(description='manually control your robot')
		parser.add_argument('--name', dest='robot_name', default=None,
				help='specify the robot to connect to')
		args = parser.parse_args()

		# get interface to robot
		self.interf = miro.lib.RobotInterface(robot_name=args.robot_name)

		# run loop
		curses.wrapper(self.loop)

		# disconnect interface
		self.interf.disconnect()

################################################################

if __name__ == "__main__":

	# normal singular invocation
	main = client()
