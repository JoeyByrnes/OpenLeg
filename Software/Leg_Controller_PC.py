##################################################
## OpenLeg Python Controller using OdriveTool
##
## Authors: Joseph Byrnes & Kanyon Edvall
##################################################

import odrive
import time
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd



cur_x = 0
cur_y = 360

safe_mode = True
linear = False
fast = False
slow = False
record = False
xpos = 0
ypos = -200

start_time = 0

setx = open("x_set.txt","w+")
sety = open("y_set.txt","w+")
kr = open("knee_reading.txt","w+")
ks = open("knee_setpoint.txt","w+")
hr = open("hip_reading.txt","w+")
hs = open("hip_setpoint.txt","w+")
tm = open("time.txt","w+")
crk = open("currentk.txt","w+")
crh = open("currenth.txt","w+")

def moveJoint(dest_x, dest_y):
	xpos = dest_x
	ypos = dest_y
	q2 = -math.acos((xpos*xpos + ypos*ypos - a1*a1 - a2*a2)/(2*a1*a2))
	q1 = math.atan2(ypos, xpos) + math.atan2(a2*math.sin(q2), a1 + a2*math.cos(q2)) - q2

	#print("Generated q2: " + str(q2) + ", q1: " + str(q1))

	knee_set = q2*rad_to_cpr*knee_ratio + zero_pos_knee
	hip_set = -q1*rad_to_cpr*hip_ratio + zero_pos_hip

	knee_pos = od.axis0.encoder.pos_estimate
	hip_pos = od.axis1.encoder.pos_estimate

	time_read = time.time() - start_time
	if record:
		kr.write("%f\r\n" % knee_pos)
		hr.write("%f\r\n" % hip_pos)
		ks.write("%f\r\n" % knee_set)
		hs.write("%f\r\n" % hip_set)
		tm.write("%f\r\n" % time_read)
		setx.write("%f\r\n" % dest_x)
		sety.write("%f\r\n" % dest_y)

		currentk = math.fabs(od.axis0.motor.current_meas_phC) + math.fabs(od.axis0.motor.current_meas_phB)
		crk.write("%f\r\n" % currentk)

		currenth = math.fabs(od.axis1.motor.current_meas_phC) + math.fabs(od.axis1.motor.current_meas_phB)
		crh.write("%f\r\n" % currenth)


	# if fast:
	knee.pos_setpoint = int(knee_set)
	hip.pos_setpoint = int(hip_set)
	# else:
	# 	knee.move_to_pos(int(knee_set))
	# 	hip.move_to_pos(int(hip_set))

def moveLinear(dest_x, dest_y):
	global cur_x
	global cur_y
	resolution = 10
	dist = ((dest_x - cur_x)**2 + (dest_y - cur_y) ** 2)**0.5

	dist_y = math.fabs(cur_y - dest_y)
	dist_x = math.fabs(cur_x - dest_x)

	num_points = dist/resolution
	dx = math.copysign(dist_x/num_points, dest_x-cur_x)
	dy = math.copysign(dist_y/num_points, dest_y-cur_y)

	for i in range(int(num_points)):
		moveJoint(cur_x+dx, cur_y+dy)
		cur_x = cur_x+dx
		cur_y = cur_y+dy
		# time.sleep(0.0000001)


def moveLinearSlow(dest_x, dest_y):
	global cur_x
	global cur_y
	resolution = 0.1
	dist = ((dest_x - cur_x)**2 + (dest_y - cur_y) ** 2)**0.5

	dist_y = math.fabs(cur_y - dest_y)
	dist_x = math.fabs(cur_x - dest_x)

	num_points = dist/resolution
	dx = math.copysign(dist_x/num_points, dest_x-cur_x)
	dy = math.copysign(dist_y/num_points, dest_y-cur_y)

	for i in range(int(num_points)):
		moveJoint(cur_x+dx, cur_y+dy)
		cur_x = cur_x+dx
		cur_y = cur_y+dy
		# time.sleep(0.0000001)


def moveLinearadj(dest_x, dest_y, res):
	global cur_x
	global cur_y
	resolution = res
	dist = ((dest_x - cur_x)**2 + (dest_y - cur_y) ** 2)**0.5

	dist_y = math.fabs(cur_y - dest_y)
	dist_x = math.fabs(cur_x - dest_x)

	num_points = dist/resolution
	dx = math.copysign(dist_x/num_points, dest_x-cur_x)
	dy = math.copysign(dist_y/num_points, dest_y-cur_y)

	for i in range(int(num_points)):
		moveJoint(cur_x+dx, cur_y+dy)
		cur_x = cur_x+dx
		cur_y = cur_y+dy
		# time.sleep(0.0000001)


# walk_x = [-0.0116,-0.011,-0.0104,-0.0098,-0.0092,-0.0086,-0.008,-0.0074,-0.0068,-0.0062,-0.0056,-0.005,-0.0044,-0.0038,-0.0032,-0.0026,-0.002,-0.0014,-0.0008,-0.0002,0.0004,0.001,0.0016,0.0022,0.0028,0.0034,0.004,0.0046,0.0052,0.0058,0.0064,0.007,0.0076,0.0082,0.0088,0.0094,0.01,0.0106,0.0112,0.0118,0.0122,0.0116,0.011,0.0104,0.0098,0.0092,0.0086,0.008,0.0074,0.0068,0.0062,0.0056,0.005,0.0044,0.0038,0.0032,0.0026,0.002,0.0014,0.0008,0.0002,-0.0004,-0.001,-0.0016,-0.0022,-0.0028,-0.0034,-0.004,-0.0046,-0.0052,-0.0058,-0.0064,-0.007,-0.0076,-0.0082,-0.0088,-0.0094,-0.01,-0.0106,-0.0112,-0.0118,-0.0124]
# print(len(walk_x))
# walk_y = [-0.022388,-0.024077,-0.025346,-0.026247,-0.026834,-0.027153,-0.02725,-0.027167,-0.026942,-0.026612,-0.02621,-0.025765,-0.025304,-0.024851,-0.024428,-0.02405,-0.023734,-0.023491,-0.02333,-0.023255,-0.02327,-0.023374,-0.023564,-0.023832,-0.02417,-0.024564,-0.025,-0.025458,-0.025916,-0.02635,-0.026732,-0.02703,-0.027212,-0.02724,-0.027074,-0.02667,-0.025984,-0.024966,-0.023564,-0.021722,-0.018,-0.014189,-0.012678,-0.011566,-0.010668,-0.009913,-0.0092639,-0.0086984,-0.008202,-0.0077648,-0.0073793,-0.0070401,-0.006743,-0.0064848,-0.0062629,-0.0060752,-0.0059203,-0.0057967,-0.0057037,-0.0056404,-0.0056065,-0.0056016,-0.0056258,-0.0056793,-0.0057624,-0.0058756,-0.00602,-0.0061966,-0.0064069,-0.0066528,-0.0069365,-0.0072613,-0.0076308,-0.0080501,-0.0085258,-0.0090669,-0.0096862,-0.010403,-0.011247,-0.012276,-0.013618,-0.015782]

walk_y_2 = [-0.022701,-0.024316,-0.02552,-0.026366,-0.026904,-0.027184,-0.027248,-0.027138,-0.026893,-0.026549,-0.026138,-0.025688,-0.025227,-0.024778,-0.024361,-0.023993,-0.023689,-0.023458,-0.023311,-0.023251,-0.023281,-0.0234,-0.023603,-0.023884,-0.024232,-0.024635,-0.025075,-0.025535,-0.025991,-0.026418,-0.026788,-0.02707,-0.027228,-0.027227,-0.027024,-0.026577,-0.025839,-0.024761,-0.023289,-0.023,-0.019334,-0.017885,-0.016823,-0.015969,-0.015254,-0.014643,-0.014114,-0.013653,-0.013251,-0.0129,-0.012596,-0.012334,-0.012111,-0.011926,-0.011775,-0.011658,-0.011574,-0.011521,-0.0115,-0.011511,-0.011553,-0.011626,-0.011732,-0.011872,-0.012046,-0.012256,-0.012504,-0.012794,-0.013129,-0.013513,-0.013953,-0.014458,-0.01504,-0.015717,-0.016519,-0.017501,-0.018786,-0.020865]
walk_x_2 = [0.0113,-0.0109,-0.0103,-0.0097,-0.0091,-0.0085,-0.0079,-0.0073,-0.0067,-0.0061,-0.0055,-0.0049,-0.0043,-0.0037,-0.0031,-0.0025,-0.0019,-0.0013,-0.0007,-0.0001,0.0005,0.0011,0.0017,0.0023,0.0029,0.0035,0.0041,0.0047,0.0053,0.0059,0.0065,0.0071,0.0077,0.0083,0.0089,0.0095,0.0101,0.0107,0.0113,0.0113,0.0107,0.0101,0.0095,0.0089,0.0083,0.0077,0.0071,0.0065,0.0059,0.0053,0.0047,0.0041,0.0035,0.0029,0.0023,0.0017,0.0011,0.0005,-0.0001,-0.0007,-0.0013,-0.0019,-0.0025,-0.0031,-0.0037,-0.0043,-0.0049,-0.0055,-0.0061,-0.0067,-0.0073,-0.0079,-0.0085,-0.0091,-0.0097,-0.0103,-0.0109,-0.0115]

walk_x = [-0.0116,-0.011,-0.0104,-0.0098,-0.0092,-0.0086,-0.008,-0.0074,-0.0068,-0.0062,-0.0056,-0.005,-0.0044,-0.0038,-0.0032,-0.0026,-0.002,-0.0014,-0.0008,-0.0002,0.0004,0.001,0.0016,0.0022,0.0028,0.0034,0.004,0.0046,0.0052,0.0058,0.0064,0.007,0.0076,0.0082,0.0088,0.0094,0.01,0.0106,0.0112,0.0118,0.0122,0.0116,0.011,0.0104,0.0098,0.0092,0.0086,0.008,0.0074,0.0068,0.0062,0.0056,0.005,0.0044,0.0038,0.0032,0.0026,0.002,0.0014,0.0008,0.0002,-0.0004,-0.001,-0.0016,-0.0022,-0.0028,-0.0034,-0.004,-0.0046,-0.0052,-0.0058,-0.0064,-0.007,-0.0076,-0.0082,-0.0088,-0.0094,-0.01,-0.0106,-0.0112,-0.0118,-0.0124]
walk_y = [-0.022388,-0.024077,-0.025346,-0.026247,-0.026834,-0.027153,-0.02725,-0.027167,-0.026942,-0.026612,-0.02621,-0.025765,-0.025304,-0.024851,-0.024428,-0.02405,-0.023734,-0.023491,-0.02333,-0.023255,-0.02327,-0.023374,-0.023564,-0.023832,-0.02417,-0.024564,-0.025,-0.025458,-0.025916,-0.02635,-0.026732,-0.02703,-0.027212,-0.02724,-0.027074,-0.02667,-0.025984,-0.024966,-0.023564,-0.021722,-0.018,-0.014189,-0.012678,-0.011566,-0.010668,-0.009913,-0.0092639,-0.0086984,-0.008202,-0.0077648,-0.0073793,-0.0070401,-0.006743,-0.0064848,-0.0062629,-0.0060752,-0.0059203,-0.0057967,-0.0057037,-0.0056404,-0.0056065,-0.0056016,-0.0056258,-0.0056793,-0.0057624,-0.0058756,-0.00602,-0.0061966,-0.0064069,-0.0066528,-0.0069365,-0.0072613,-0.0076308,-0.0080501,-0.0085258,-0.0090669,-0.0096862,-0.010403,-0.011247,-0.012276,-0.013618,-0.015782]
print(len(walk_y))
def walk1():
	for x in range(20):
		for i in range(0,82,2):
			moveLinear(-10000*walk_x[i], (-10000*walk_y[i])/2+200)
			# time.sleep(0.00001)


# def walk2():
# 	for x in range(2):
# 		for i in range(0,76,1):
# 			moveLinearSlow(-10000*walk_x[i], (-10000*walk_y[i])/2+200)
# 			# time.sleep(0.00001)
def walk2():
	start_time = time.time()
	for i in range(0,76,1):
		moveLinearSlow(-10000*walk_x[i], (-10000*walk_y[i])/2+200)
		# time.sleep(0.00001)

def stretch():
	od.axis0.motor.config.current_lim = 25
	od.axis1.motor.config.current_lim = 25

	moveLinearSlow(0, 355)
	time.sleep(1)
	moveLinearSlow(0, 180)
	time.sleep(1)
	moveLinearSlow(0, 355)
	time.sleep(1)
	moveLinearadj(0, 180, 0.25)
	time.sleep(0.5)
	moveLinearadj(0, 355, 0.25)
	time.sleep(0.5)
	moveLinearadj(0, 180, 0.5)
	time.sleep(0.35)
	moveLinearadj(0, 355, 0.5)
	time.sleep(0.35)

	od.axis0.motor.config.current_lim = 15
	od.axis1.motor.config.current_lim = 12


def jump1():
	moveLinear(-80,120)
	time.sleep(1)
	moveLinear(-0, 355)
	time.sleep(0.1)
	moveLinear(-80,120)
	time.sleep(0.01)
	moveLinear(-80, 200)

def jump2():
	moveLinear(-80,120)
	time.sleep(1)
	moveJoint(-0, 355)
	time.sleep(0.1)
	moveLinear(-80,120)
	time.sleep(0.01)
	moveLinear(-80, 200)

def jump3():
	knee.pos_setpoint = -3200
	hip.pos_setpoint = -4000
	time.sleep(1)
	knee.pos_setpoint = 0
	hip.pos_setpoint = 0
	time.sleep(0.1)
	knee.pos_setpoint = -4000
	hip.pos_setpoint = -4800
	time.sleep(0.1)
	knee.pos_setpoint = -3200
	hip.pos_setpoint = -4000

def jump4():# DO NOT EDIT -- Reliable jump for demo
	knee.pos_setpoint = -3500
	hip.pos_setpoint = -3800
	time.sleep(1)
	knee.pos_setpoint = 0
	hip.pos_setpoint = 0
	time.sleep(0.1)
	knee.pos_setpoint = -4000
	hip.pos_setpoint = -4800
	time.sleep(0.1)
	knee.pos_setpoint = -3500
	hip.pos_setpoint = -3800
# def jump4():# DO NOT EDIT -- Reliable jump for demo
# 	knee.pos_setpoint = -3500
# 	hip.pos_setpoint = -3800
# 	time.sleep(1)
# 	for i in range(10):
# 		time_read = time.time() - start_time
# 		tm.write("%f\r\n" % time_read)
# 		currentk = math.fabs(od.axis0.motor.current_meas_phC) + math.fabs(od.axis0.motor.current_meas_phB)
# 		crk.write("%f\r\n" % currentk)
#
# 		currenth = math.fabs(od.axis1.motor.current_meas_phC) + math.fabs(od.axis1.motor.current_meas_phB)
# 		crh.write("%f\r\n" % currenth)
# 	knee.pos_setpoint = 0
# 	hip.pos_setpoint = 0
# 	for i in range(100):
# 		time_read = time.time() - start_time
# 		tm.write("%f\r\n" % time_read)
# 		currentk = math.fabs(od.axis0.motor.current_meas_phC) + math.fabs(od.axis0.motor.current_meas_phB)
# 		crk.write("%f\r\n" % currentk)
#
# 		currenth = math.fabs(od.axis1.motor.current_meas_phC) + math.fabs(od.axis1.motor.current_meas_phB)
# 		crh.write("%f\r\n" % currenth)
# 	knee.pos_setpoint = -4000
# 	hip.pos_setpoint = -4800
# 	for i in range(100):
# 		time_read = time.time() - start_time
# 		tm.write("%f\r\n" % time_read)
# 		currentk = math.fabs(od.axis0.motor.current_meas_phC) + math.fabs(od.axis0.motor.current_meas_phB)
# 		crk.write("%f\r\n" % currentk)
#
# 		currenth = math.fabs(od.axis1.motor.current_meas_phC) + math.fabs(od.axis1.motor.current_meas_phB)
# 		crh.write("%f\r\n" % currenth)
# 	knee.pos_setpoint = -3500
# 	hip.pos_setpoint = -3800


def jump5():# DO NOT EDIT -- Retraction Jump
	knee.pos_setpoint = -3500
	hip.pos_setpoint = -3800
	time.sleep(1)
	knee.pos_setpoint = 0
	hip.pos_setpoint = 0
	time.sleep(0.125)
	knee.pos_setpoint = -4100
	hip.pos_setpoint = -5300
	time.sleep(0.2)
	knee.pos_setpoint = -3500
	hip.pos_setpoint = -3800


def jump6():# DO NOT EDIT -- Right Box Jump - Kicks box 5 out of way
	knee.pos_setpoint = -3500
	hip.pos_setpoint = -3800
	time.sleep(1)
	knee.pos_setpoint = 0
	hip.pos_setpoint = 0
	time.sleep(0.125)
	knee.pos_setpoint = -4100
	hip.pos_setpoint = -5300
	time.sleep(0.2)
	knee.pos_setpoint = -1000
	hip.pos_setpoint = -3800


def jump7():# DO NOT EDIT -- Right Box Jump - Lands on box 5
	knee.pos_setpoint = -3500
	hip.pos_setpoint = -3800
	time.sleep(1)
	knee.pos_setpoint = 0
	hip.pos_setpoint = 0
	time.sleep(0.125)
	knee.pos_setpoint = -1000
	hip.pos_setpoint = -3800


def jump8():# DO NOT EDIT -- left Box Jump
	knee.pos_setpoint = -3500
	hip.pos_setpoint = -3800
	time.sleep(1)
	knee.pos_setpoint = 0
	hip.pos_setpoint = 0
	time.sleep(0.125)
	knee.pos_setpoint = -4100
	hip.pos_setpoint = -2000
	time.sleep(0.1)
	moveJoint(200, 230)

def jump9():# DO NOT EDIT -- ninja mode
	time.sleep(5)
	jump7()  # Kicks Box #6 Right side
	time.sleep(0.5)
	## added for ninja move
	time.sleep(0.3)
	knee.pos_setpoint = -1600
	hip.pos_setpoint = -3500
	time.sleep(0.08)
	knee.pos_setpoint = -900
	hip.pos_setpoint = -4200
	jumphome()
	time.sleep(0.8)
	jump8()  # Kicks Box #2 on Left side
	time.sleep(0.5)
	moveJoint(200, 280)
	time.sleep(0.1)
	jumphome()
	time.sleep(0.8)
	jump6()  # Kicks Box #5 on Right side
	time.sleep(0.5)
	time.sleep(0.2)
	knee.pos_setpoint = -1600
	hip.pos_setpoint = -3500
	time.sleep(0.08)
	knee.pos_setpoint = -900
	hip.pos_setpoint = -4200
	jumphome()





def jumphome():# DO NOT EDIT -- Jump456 Home
	knee.move_to_pos(-3500)
	hip.move_to_pos(-3800)

def demo():
	input("Press enter to run fast walking trajectory")
	walk1()
	input("Press enter to run slow walking trajectory")
	walk2()
	input("Press enter to stretch robot leg")
	stretch()
	input("Press enter to go to jump home position")
	jumphome()
	input("Press enter to execute basic jump")
	jumphome()
	time.sleep(1)
	jump4()
	input("Press enter to execute retracted foot jump")
	jumphome()
	time.sleep(1)
	jump5()
	input("Press enter to jump onto single box on left")
	jumphome()
	time.sleep(1)
	jump8()
	input("Press enter to jump onto five boxes on right")
	jumphome()
	time.sleep(1)
	jump7()
	time.sleep(1)
	jumphome()

def plot():
	k_read = open("knee_reading.txt", "r")
	knee_reads = []
	for val in k_read.read().split():
		knee_reads.append(float(val))
	k_read.close()

	h_read = open("hip_reading.txt", "r")
	hip_reads = []
	for val in h_read.read().split():
		hip_reads.append(float(val))
	h_read.close()

	k_set = open("knee_setpoint.txt", "r")
	knee_sets = []
	for val in k_set.read().split():
		knee_sets.append(float(val))
	k_set.close()

	h_set = open("hip_setpoint.txt", "r")
	hip_sets = []
	for val in h_set.read().split():
		hip_sets.append(float(val))
	h_set.close()

	time_rd = open("time.txt", "r")
	time_reads = []
	for val in time_rd.read().split():
		time_reads.append(float(val))
	time_rd.close()

	x_set = open("x_set.txt", "r")
	x_sets = []
	for val in x_set.read().split():
		x_sets.append(float(val))
	x_set.close()

	y_set = open("y_set.txt", "r")
	y_sets = []
	for val in y_set.read().split():
		y_sets.append(float(val))
	y_set.close()

	currk = open("currentk.txt", "r")
	cur_meas_k = []
	for val in currk.read().split():
		cur_meas_k.append(float(val))
	currk.close()

	currh = open("currenth.txt", "r")
	cur_meas_h = []
	for val in currh.read().split():
		cur_meas_h.append(float(val))
	currh.close()

	knee_rad = []
	hip_rad = []
	knee_s_rad = []
	hip_s_rad = []
	# forward Kinematics
	for knee_r in knee_reads:
		knee_rad.append(knee_r/(rad_to_cpr*knee_ratio))
	for hip_r in hip_reads:
		hip_rad.append(hip_r / (rad_to_cpr * hip_ratio))
	for knee_s in knee_sets:
		knee_s_rad.append(knee_s/(rad_to_cpr*knee_ratio))
	for hip_s in hip_sets:
		hip_s_rad.append(hip_s / (rad_to_cpr * hip_ratio))
	#
	# real_x_pos = []
	# real_y_pos = []
	# set_x_pos = []
	# set_y_pos = []
	# idx = 0
	# for kn in knee_rad:
	# 	real_y_pos.append(a1 * math.sin(hip_rad[idx]+math.pi/12.5) + a2 * math.sin(-kn + hip_rad[idx]+math.pi/12.5))
	# 	real_x_pos.append(-(a1*math.cos(hip_rad[idx]+math.pi/12.5)+a2*math.cos(-kn+hip_rad[idx]+math.pi/12.5)))
	# 	idx = idx + 1
	#
	# # plt.plot(time_reads, knee_reads, label='Knee Position')
	# # plt.plot(time_reads, knee_sets, label='Knee Setpoint')
	# #
	# # plt.plot(time_reads, hip_reads, label='Hip Position')
	# # plt.plot(time_reads, hip_sets, label='Hip Setpoint')
	# # plt.plot(real_x_pos, real_y_pos, label="Measured Foot Position")
	# # plt.plot(x_sets, y_sets, label="Measured Foot Position")
	# err = []
	# idx = 0
	# for x in x_sets:
	# 	err.append(math.sqrt((x - real_x_pos[idx])*(x - real_x_pos[idx]) + (y_sets[idx] - real_y_pos[idx])*(y_sets[idx] - real_y_pos[idx])))
	# 	idx = idx+1
	# x_sets.reverse()
	# y_sets.reverse()
	# real_y_pos.reverse()
	# real_x_pos.reverse()
	# for i in range(1500):
	# 	x_sets.pop()
	# 	y_sets.pop()
	# 	real_y_pos.pop()
	# 	real_x_pos.pop()
	# x_sets.reverse()
	# y_sets.reverse()
	# real_y_pos.reverse()
	# real_x_pos.reverse()
	# plt.ylim(370, 190)
	# plt.ylabel('Vertical Position (mm)')
	# plt.xlabel('Horizontal Position (mm)')
	plt.title('Motor Current During Jump')
	# plt.plot(x_sets, y_sets, label="Commanded Foot Position")
	# plt.plot(real_x_pos, real_y_pos, label="Measured Foot Position")
	# plt.plot(time_reads, err)
	# plt.plot(time_reads, knee_rad, label="Knee Measurement")
	# plt.plot(time_reads, hip_rad, label="Hip Measurement")
	# plt.plot(time_reads, knee_s_rad, label="Knee Setpoint")
	# plt.plot(time_reads, hip_s_rad, label="Hip Setpoint")
	plt.plot(time_reads, cur_meas_k, label="Knee")
	plt.plot(time_reads, cur_meas_h, label="Hip")
	plt.tight_layout()
	plt.ylabel('Current (A)')
	plt.xlabel('Time (sec)')
	plt.legend()
	plt.show()




hip_level = -100 #90 deg = 3900
leg_level = 2100
zero_pos_hip = 3800
zero_pos_knee = 0
serial = 3666335E3037
a1 = 180
a2 = 180
knee_ratio = 6.25
hip_ratio = 84/11
rad_to_cpr = (2000/(2*math.pi))


##### Ranges
knee_lower = -5500
knee_upper = 0
hip_lower = -6000
hip_upper = 3900

od = odrive.find_any()
# od = odrive.find(serial)
print("Init done")


knee = od.axis0.controller # Knee
hip = od.axis1.controller # Hip

od.axis0.motor.config.current_lim = 25
od.axis1.motor.config.current_lim = 20

knee.move_to_pos(0)
hip.move_to_pos(0)

print("Welcome to the Dynamic Robotic Leg Motor Control Interface")


while (True):
	xpos_str = input("\n\nEnter X: ")
	
	if xpos_str == "fast":
		fast = True
		slow = False
		print("Fast mode enabled!\n")
		continue
	if xpos_str == "slow":
		fast = False
		slow = True
		print("Slow mode enabled!\n")
		continue
	elif xpos_str == "normal":
		fast = False
		slow = False
		print("Fast and slow modes disabled\n")
		continue
	elif xpos_str == "safe":
		safe_mode = True
		print("safe mode enabled!\n")
		continue
	elif xpos_str == "unsafe":
		safe_mode = False
		print("safe mode disabled\n")
		continue
	elif xpos_str == "linear":
		linear = True
		print("linear move mode enabled!\n")
		continue
	elif xpos_str == "joint":
		linear = False
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "jump1":
		jump1()
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "jump2":
		jump2()
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "walk1":
		walk1()
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "walk2":
		walk2()
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "jump3":
		jump3()
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "jump4":
		jump4()
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "jump5":
		jump5()
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "jump6":
		jump6()
		print("joint move mode enabled!\n")
		continue
	elif xpos_str == "jump7":
		jump7()
		print("right box high\n")
		continue
	elif xpos_str == "jump8":
		jump8()
		print("right box\n")
		continue
	elif xpos_str == "jump9":
		jump9()
		print("ninja mode activated\n")
		continue
	elif xpos_str == "jump10":
		jump10()
		print("Doesnt exist\n")
		continue
	elif xpos_str == "jumphome":
		jumphome()
		print("demo routine started\n")
		continue
	elif xpos_str == "demo":
		demo()
		print("demo routine started\n")
		continue
	elif xpos_str == "stretch":
		stretch()
		print("demo routine started\n")
		continue
	elif xpos_str == "pass":
		record = False
		print("not_recording \n")
		continue
	elif xpos_str == "record":
		record = True
		print("Recording \n")
		continue
	elif xpos_str == "save":
		kr.close()
		ks.close()
		hr.close()
		hs.close()
		tm.close()
		setx.close()
		sety.close()
		crk.close()
		crh.close()
		continue
	elif xpos_str == "plot":
		plot()
		continue
	else:
		xpos = float(xpos_str)
		
	ypos = float(input("Enter Y: "))

	print("Using X: " + str(xpos) + ", Y: " + str(ypos))
	try:
		q2 = -math.acos((xpos*xpos + ypos*ypos - a1*a1 - a2*a2)/(2*a1*a2))
		q1 = math.atan2(ypos, xpos) + math.atan2(a2*math.sin(q2), a1 + a2*math.cos(q2)) - q2
	except ValueError:
		print("Error")
		continue
	print("Generated q2: " + str(q2) + ", q1: " + str(q1))

	knee_setpoint = q2*rad_to_cpr*knee_ratio + zero_pos_knee
	hip_setpoint = -q1*rad_to_cpr*hip_ratio + zero_pos_hip

	print("Knee setpoint: " + str(knee_setpoint))
	print("Hip setpoint: " + str(hip_setpoint))

	if knee_setpoint > knee_upper:
		print("Setpoint out of range (knee)")
		continue
	elif knee_setpoint < knee_lower:
		print("Setpoint out of range (knee)")
		continue
	elif hip_setpoint > hip_upper:
		print("Setpoint out of range (hip)")
		continue
	elif hip_setpoint < hip_lower:
		print("Setpoint out of range (hip)")
		continue

	if safe_mode:
		safe_input = input("Continue? Y / N: ")

		if safe_input.lower() != "y":
			print("Resetting\n")
			continue
	if slow:
		moveLinearSlow(xpos, ypos)
	elif linear:
		moveLinear(xpos, ypos)
	if fast:
		moveJoint(xpos, ypos)

sys.exit()


'''
Knee
-3300 to 2100 (pos is cw)

Hip
-3900 3800 (pos is cw)
'''
