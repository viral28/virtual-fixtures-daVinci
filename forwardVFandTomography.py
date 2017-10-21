#!/usr/bin/python
import sys
import argparse
import dvrk
import numpy
import PyKDL 
import time
import math

if __name__ == "__main__":


	p = dvrk.psm("PSM3")
	m = dvrk.mtm("MTML")
	time.sleep(float(0.5))		
	initialrot = PyKDL.Rotation(0,1,0, 1,0,0, 0,0,-1)
        p.move(initialrot)
	trans = PyKDL.Rotation.RotZ(math.pi)
	w = PyKDL.Vector(1, 0l, 0)            # Normal vector of the VP (virtual plane)
	w = w / PyKDL.Vector.Norm(w)
	pos_pre_f = m.get_current_position()
	time.sleep(float(0.5))
	pos_pre = pos_pre_f.p
	pos_st = pos_pre                     # Starting point
    

	radius = 0.05
        initialframe = p.get_current_position()
	initialrot = initialframe.M
        initialpos = initialframe.p
	parallel = initialrot.UnitZ()*-1  #vector from center of the circle to the point on the circle (tooltip)
	parallel = w*(parallel*w)
	parallel = parallel/PyKDL.Vector.Norm(parallel)
	pos_ctr = initialpos - radius*parallel


	#pos_ctr = PyKDL.dot(w, pos_st) * w   # Center or a point of the VP
	u_z = pos_ctr - initialpos
	u_z = u_z / PyKDL.Vector.Norm(u_z)

	u_y = u_z * w
	orientation_slave = PyKDL.Rotation(w, u_y, u_z)
	#time.sleep(float(0.5))
	p.move(orientation_slave)
	#time.sleep(float(0.5))
	pos_file = open("forwardTomography.txt", "w")
	print "Starting"
	for i in range(300):
		
		pos_nxt_f = m.get_current_position() # Next point
		pos_nxt = pos_nxt_f.p
		dis = PyKDL.dot(w, pos_st - pos_nxt)
		pos_nxt_proj = pos_nxt + dis * w     # Next point prjected onto the VP
		delta_pos = pos_nxt_proj - pos_pre
		delta_pos_slave = trans * delta_pos
		delta_pos_slave = 0.2 * delta_pos_slave
		pos_slave = p.get_current_position().p + delta_pos_slave
		u_z = pos_ctr - pos_slave
                u_z = w*(u_z*w)
		u_z = u_z / PyKDL.Vector.Norm(u_z)
		u_y = u_z * w
		orientation_slave = PyKDL.Rotation(w, u_y, u_z)
		f_slave = PyKDL.Frame(orientation_slave, pos_slave)
		
		#time.sleep(float(0.5))
		start_time = time.time()
		p.move(f_slave)
		
		print("--- %s seconds: " % (time.time() - start_time))
		#time.sleep(float(0.5))
		pos_pre = pos_nxt_proj
		print i
		pos_slave_l = [pos_slave[0], pos_slave[1], pos_slave[2], orientation_slave[0, 0], orientation_slave[0, 1], orientation_slave[0, 2], orientation_slave[1, 0], orientation_slave[1, 1], orientation_slave[1, 2], orientation_slave[2, 0], orientation_slave[2, 1], orientation_slave[2, 2]]
		pos_file.write('	'.join(map(repr, pos_slave_l)))
		pos_file.write("\n")
		#print "Bingo"
	pos_file.close()

