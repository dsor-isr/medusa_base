#!/usr/bin/env python

""" 
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
"""
import rospy
from medusa_bringup_algorithms.MedusaSetup import MedusaSetup
from std_msgs.msg import Int8, Bool

class MedusaBringupVxNode():
	def __init__(self):
		"""
		Constructor for ros node
		"""

		"""
		###########################################################################################
		@.@ Init node
		###########################################################################################
		"""
		rospy.init_node('medusa_bringup_vx_node')

		"""
		###########################################################################################
		@.@ Handy Variables
		###########################################################################################
		# Declare here some variables you might think usefull -> example: self.fiic = true

		"""
		self.loadParams()
		MedusaSetup(self.name, self.config_package, self.folder, self.namespace)


	"""
	###########################################################################################
	@.@ Member Helper function to set up parameters; 
	###########################################################################################
	"""
	def loadParams(self):
		# self.node_frequency = rospy.get_param('~node_frequency', 10)
		# self.real = rospy.get_param('~real', False)
		self.name = rospy.get_param('~name', "mred")
		self.config_package = rospy.get_param('~type', "medusa_bringup")
		self.folder = rospy.get_param('~folder', "simulation")
		self.namespace = rospy.get_param('~namespace', "false")

		#self.bags = rospy.get_param('~bags', False)
		#self.pf_controller = rospy.get_param('~path_following', "Lapierre")
		#self.waypoint = rospy.get_param('~waypoint', "standard")


def main():

	medusaBringupVx = MedusaBringupVxNode()
	
	# +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	rospy.spin()

if __name__ == '__main__':
	main()
