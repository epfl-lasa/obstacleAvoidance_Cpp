import rospy

import tf

from sensor_msgs.msg import Joy


def joy_callback(msg, calib3d):

	if msg.buttons[0] == 1: #position/orientation mode switch
		for i in range(3):
			calib3d.relative_position_incr[i] = 0.0
			calib3d.relative_orientation_incr[i] = 0.0
		calib3d.orientation_flag = not calib3d.orientation_flag

	if msg.axes[2] > 0.5: #no activation
		for i in range(3):
			calib3d.relative_position_incr[i] = 0.0
			calib3d.relative_orientation_incr[i] = 0.0
		return

	if calib3d.orientation_flag: #orientation mode
		gain = 0.001
		calib3d.relative_orientation_incr[0] = gain*msg.axes[0]
		calib3d.relative_orientation_incr[1] = gain*msg.axes[1]
		calib3d.relative_orientation_incr[2] = gain*msg.axes[3]

	else: #position mode
		gain = 0.001
		calib3d.relative_position_incr[0] = gain*msg.axes[0]
		calib3d.relative_position_incr[1] = gain*msg.axes[1]
		calib3d.relative_position_incr[2] = gain*msg.axes[3]

class Calib3D:
	def __init__(self):
		rospy.init_node('calib3d')
		#self.parent_tf_listener = tf.TransformListener()
		#.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))

		self.relative_position = [0.1592201791561404, -0.05646253176161607, 0.5543856122517523]#[-0.05, 0.11, -0.11] #[0.0, 0.0, 0.0]
		self.relative_orientation = [0.014, -0.429, -3.06]#[0.0, 0.0, 0.0]# [-1.5707, 0.7853, 1.5707] #[0.0, 0.0, 0.0]

		self.relative_position_incr = [0.0, 0.0, 0.0]
		self.relative_orientation_incr = [0.0, 0.0, 0.0]
		self.orientation_flag = False

		self.subscriber = rospy.Subscriber('/joy',
			Joy, joy_callback, self)

		self.continuouslyIncrementAndBroadcastTf()

	def continuouslyIncrementAndBroadcastTf(self):
		br = tf.TransformBroadcaster()
		rate = rospy.Rate(100.0)
		while not rospy.is_shutdown():
			# increment
			for i in range(3):
				self.relative_position[i] += self.relative_position_incr[i]
				self.relative_orientation[i] += self.relative_orientation_incr[i]
			#print ("P:", self.relative_position[0],self.relative_position[1],self.relative_position[2])
			#print ("R:", self.relative_orientation[0],self.relative_orientation[1],self.relative_orientation[2])

			# broadcast
			br.sendTransform((self.relative_position[0],
				self.relative_position[1],
				self.relative_position[2]),
				tf.transformations.quaternion_from_euler(
					self.relative_orientation[0],
					self.relative_orientation[1],
					self.relative_orientation[2]),
				rospy.Time.now(),
				"velodyne_link",
				"camera_link")
			rate.sleep()

if __name__ == '__main__':
	c = Calib3D()
