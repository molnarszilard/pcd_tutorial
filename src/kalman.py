#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class Kalman(object):
	"""docstring for Kalman"""
	def __init__(self, n_states, n_sensors):
		super(Kalman, self).__init__()
		self.n_states = n_states
		self.n_sensors = n_sensors

		self.x = np.matrix(np.zeros(shape=(n_states,1)))
		self.P = np.matrix(np.identity(n_states)) 
		self.F = np.matrix(np.identity(n_states))
		self.u = np.matrix(np.zeros(shape=(n_states,1)))
		self.H = np.matrix(np.identity(n_states)) #np.matrix(np.zeros(shape=(n_sensors, n_states)))
		self.R = np.matrix(np.identity(n_sensors))
		self.I = np.matrix(np.identity(n_states))
		self.Q = np.matrix(np.identity(n_states)) * 0.1

		self.first = True

	def update(self, Z):
		'''Z: new sensor values as numpy matrix'''

		w = Z - self.H * self.x
		S = self.H * self.P * self.H.getT() + self.R
		K = self.P * self.H.getT() * S.getI()
		self.x = self.x + K * w
		self.P = (self.I - K * self.H) * self.P

	def predict(self):
		self.x = self.F * self.x + self.u
		self.P = self.F * self.P * self.F.getT() + self.Q

class Subscriber(object):
	"""docstring for Subscriber"""
	def __init__(self):
		super(Subscriber, self).__init__()
		rospy.init_node('imu_conv', anonymous=True)

		self.kalman = Kalman(n_states = 3, n_sensors = 3)
		self.kalman.H = np.matrix(np.identity(self.kalman.n_states))/2
		#self.kalman.P *= 0.1
		self.kalman.R *= 1.0

		self.pub_accel = rospy.Publisher('filtered_imu', Imu)

		rospy.Subscriber('/pico_zense/imu', Imu, self.callback_accel)
		rospy.spin()
		
	def callback_accel(self, msg):
		# print "received data: ", data
		Z = np.matrix([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]).getT()

		if self.kalman.first:
			self.kalman.x = Z
			self.kalman.first = False

		self.kalman.update(Z)
		self.kalman.predict()

		vec = Imu()
		vec.linear_acceleration.x = -self.kalman.x[1]
		vec.linear_acceleration.y = -self.kalman.x[0]
		vec.linear_acceleration.z = self.kalman.x[2]
		vec.header.frame_id = 'pico_zense_depth_frame'
		vec.header.stamp = rospy.Time.now()

		self.pub_accel.publish(vec)



if __name__ == '__main__':
	subscriber = Subscriber()
