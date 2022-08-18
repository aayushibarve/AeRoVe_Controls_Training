import rospy
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix
from time import sleep
import math
import matplotlib.pyplot as plt
import numpy as np
from numpy import sin as S 
from numpy import cos as C 
from numpy import tan as T
import pickle

class DroneIn3D:
	
	def __init__(self):
		self.X=np.array([
			# x0, y1, z2, phi3, theta4, psi5, 
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			# x_dot6, y_dot7, z_dot8
			0.0, 0.0, 0.0])       
		self.g = 9.81
		self.gps_lat=0
		self.gps_long=0

		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS
		self.current_p=rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.loc_pose)
		rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_pose)
		self.get_linear_vel=rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel)
		self.loc=Point()
		self.glob=Point()
		

		#PUBLISHERS
		self.pub_acc = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped,queue_size=1)

	def offboard(self):
		rate = rospy.Rate(10)
		sp = PoseStamped()
		sp.pose.position.x = 0.0
		sp.pose.position.y = 0.0
		sp.pose.position.z = 7.0
		for i in range(10):
			self.pub.publish(sp)
			rate.sleep()
		print('We are good to go!!')
		self.setmode("GUIDED")

	def loc_pose(self, data):

		self.X[0] = data.pose.position.x
		self.X[1] = data.pose.position.y
		self.X[2] = data.pose.position.z

	def global_pose(self, data):
		self.glob.x = data.latitude 
		self.glob.y = data.longitude  
		self.glob.z = data.altitude 



	def setmode(self,md):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			response = mode(0,md)
			response.mode_sent
		except rospy.ServiceException as e:
			print ("Service call failed: %s"%e)

	def takeoff(self, alt):
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			mode = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
			response = mode(0,0, self.glob.x, self.glob.y, alt)
			response.success
		except rospy.ServiceException as e:
			print ("Service call failed: %s"%e)

	def setarm(self,av): # input: 1=arm, 0=disarm
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			response = arming(av)
			response.success
		except rospy.ServiceException as e:
			print ("Service call failed: %s" %e)

	def gotopose(self, x, y ,z):
		rate = rospy.Rate(20)
		self.sp = PoseStamped()
		self.sp.pose.position.x = x
		self.sp.pose.position.y = y
		self.sp.pose.position.z = z
		dist = np.sqrt(((self.X[0]-x)**2) + ((self.X[1]-y)**2) + ((self.X[2]-z)**2))
		while(dist > 0.18):
			self.pub.publish(self.sp)
			dist = np.sqrt(((self.X[0]-x)**2) + ((self.X[1]-y)**2) + ((self.X[2]-z)**2))
			rate.sleep()
		


	def get_vel(self,vel_data):
		self.X[6]=	vel_data.twist.linear.x
		self.X[7]=	vel_data.twist.linear.y
		self.X[8]=	vel_data.twist.linear.z





class SMController:

	def __init__(self):
		self.drag_coeffs=np.diag([1,2,3]) #dp matrix
		self.a1=1 #alpha1
		self.a2=2 #alpha2
		self.b=1  #beta for control signal
		self.m=2
		self.pos_i_term=0
		self.sgn=np.array(([0],[0],[0]))
		self.p_error=0
		self.v_error=0 

	def calculate_errors(self,x,x_dot,x_dot_dot,y,y_dot,y_dot_dot,z,z_dot,z_dot_dot,yaw,cx,cy,cz,cvx,cvy,cvz):
		self.p_error=np.array(([x-cx],[y-cy],[z-cz]))
		self.v_error=np.array(([x_dot-cvx],[y_dot-cvy],[z_dot-cvz]))
		self.ad=np.array(([x_dot_dot],[y_dot_dot],[z_dot_dot]))
		


	def sgn_s0(self):
		#Calculation of value of s0 at current position and velocity
		#Integration term
		dt=0.1
		self.pos_i_term+=self.p_error*dt
		#S0 using alpha1 and alpha2
		s0=self.a1*self.pos_i_term + self.a2*self.p_error + self.v_error
		self.sgn=np.sign(s0)




	def ecap_and_ut(self,cvx,cvy,cvz):

		current_v=np.array(([cvx],[cvy],[cvz]))
		#Calculation of ecap and its components
		ecap=self.a1*self.p_error + self.a2*self.v_error + self.ad + np.array(([0],[0],[9.81])) + (1/self.m)*np.matmul(self.drag_coeffs,current_v) + self.b*self.sgn
		#Calculation of acceleration inputs
		return ecap


def control(x,x_dot,x_dot_dot,y,y_dot,y_dot_dot,z,z_dot,z_dot_dot,yaw,drone,sm1):
	cx=drone.X[0]
	cy=drone.X[1]
	cz=drone.X[2]
	cvx=drone.X[6]
	cvy=drone.X[7]
	cvz=drone.X[8]
	sm1.calculate_errors(x,x_dot,x_dot_dot,y,y_dot,y_dot_dot,z,z_dot,z_dot_dot,yaw,cx,cy,cz,cvx,cvy,cvz)
	sm1.sgn_s0()
	acc=sm1.ecap_and_ut(cvx,cvy,cvz)
	Accel=Vector3Stamped()
	Accel.vector.x=acc[0]
	Accel.vector.y=acc[1]
	Accel.vector.z=acc[2]
	drone.pub_acc.publish(Accel)


if __name__ == '__main__':
	pck=open('pickled_traj2.txt','rb')
	traj=pickle.load(pck)
	x=traj[0,:]
	x_dot=traj[1,:]
	x_dot_dot=traj[2,:]
	y=traj[3,:]
	y_dot=traj[4,:]
	y_dot_dot=traj[5,:]
	z=traj[6,:]
	z_dot=traj[7,:]
	z_dot_dot=traj[8,:]
	yaw=traj[9,:]
	num=len(x)
	#print(x)
	print("File lloaded")

	drone=DroneIn3D()
	sm1=SMController()
	drone.setarm(1)
	sleep(3)
	print("armed")
	drone.setmode("GUIDED")
	sleep(2)
	drone.takeoff(7)
	sleep(3) 
	print("trakeoff")
	rate=rospy.Rate(10)


	for i in range(num):
		control(x[i],x_dot[i],x_dot_dot[i],y[i],y_dot[i],y_dot_dot[i],z[i],z_dot[i],z_dot_dot[i],yaw[i],drone,sm1)
		print("in loop")

		rate.sleep()






	







		
		
