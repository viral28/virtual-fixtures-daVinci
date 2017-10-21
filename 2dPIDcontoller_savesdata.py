# February 22, 2017
#Raghul and Viral
# The following code restricts motion of the Master Controller in one out of three dimensions as determined by user input.
# Input Arguments: Dimensions that are getting restricted
# cntrl + z  to stop the process
import sys
import argparse
import dvrk
import numpy
import PyKDL 
import time
import numpy as np
import curses #for the interupt
from dvrk import mtm

if __name__ == "__main__":
#	parser = argparse.ArgumentParser()  #taking information from command line initialization
#	parser.add_argument('-x', '--xAxes', help = 'the x component of the vector.' )
#	parser.add_argument('-y', '--yAxes', help = 'the y component of the vector.' )
#	parser.add_argument('-z', '--zAxes', help = 'the z component of the vector.' )
#	parser.add_argument('-a', '--kp', help = 'The ratio of force felt compared to the deviation from trajectory.' )#it is currently user difined but can be changed to be fixed
#	parser.add_argument('-b', '--maxforce', help = 'The maximum force provided by the joint in one axes' ) # it is currently user difined but can be changed to be fixed (I dont know what should be the maximum force)
#	args = parser.parse_args()
    
    
    x_axis = 0#int(args.xAxes) temp test input
    y_axis = 0#int(args.yAxes)
    z_axis = 1#int(args.zAxes)
        

    userDefVector = PyKDL.Vector(x_axis, y_axis , z_axis) #vector perpendicular to the plane 
   

    kp=float(100) # gain
    kd=float(10)
    ki= float(0.000002)

    #initialization constants
    previoustime = 0
    dt= 0
    prevforcedirection = PyKDL.Vector(0,0,0)
    intergrator = PyKDL.Vector(0,0,0)
    derivator = PyKDL.Vector(0,0,0)
    maxforce= 2 #args.maxforce
    testforce = PyKDL.Vector(2, 0, 0)
    #get normalized vector of the vector defined by the user
    #lists
    force_list = []
    intergrator_list = []
    derivator_list = []
    forcedirection_list = []
   
     
    unitUserDefVector = userDefVector/PyKDL.Vector.Norm(userDefVector)


    m = mtm('MTML') 
    print "Homing"
    m.home()
    time.sleep(1)
    print "Starting to move!"
    time.sleep(1)
            
    print "meme" 
    window = curses.initscr()
    print "move joint to point on the plane and then press enter"
    
    ch = window.getch()
    curses.endwin() 
     
    posInit_temp=m.get_current_position() 
    posInit = posInit_temp.p


    m.set_wrench_body_orientation_absolute(True)# When True, force direction is constant.  Otherwise force direction defined in gripper coordinate system
    m.set_gravity_compensation(True)
 
   
 
     
    #Forever loop to determine position and apply wrench-body forces to axes being constrained
    while True:
        currenttime = time.time()
        
        posFin_temp =m.get_current_position()
        posFin = posFin_temp.p
        posDelta = PyKDL.Vector(posInit - posFin)

        proj = PyKDL.dot(posDelta, unitUserDefVector)
       
        forcedirection = proj*unitUserDefVector #force direction is the error term
      
        intergrator = intergrator+forcedirection*dt
        derivator = (forcedirection-prevforcedirection)/dt
        force = forcedirection*kp+(ki*intergrator)+(kd*derivator)
        #making sure the force does not exceed the maximum force
        
        if(PyKDL.Vector.Norm(force)!=0):    
            unitofforce = force/PyKDL.Vector.Norm(force)
        else:
            unitofforce= force
        
        #intergral windup
                
        #intergral windup
        if(PyKDL.Vector.Norm(intergrator)>10000):
            intergrator = PyKDL.Vector(0,0,0)
        
        force = unitofforce * min(PyKDL.Vector.Norm(force),maxforce)
        
        m.set_wrench_body_force(force)
        
        #the sleep 0.01 second seemed to make the pid work better
        time.sleep(0.01)
       
        print PyKDL.Vector.Norm(forcedirection) ,PyKDL.Vector.Norm(intergrator), PyKDL.Vector.Norm(derivator), PyKDL.Vector.Norm(force), force
        #print '\n'
        #print time.time() - currenttime
        dt=currenttime-previoustime
        previoustime = currenttime
        prevforcedirection = forcedirection        
        
        force_list.append(force)
        intergrator_list.append(intergrator)
        derivator_list.append(derivator)
        forcedirection_list.append(forcedirection)
   
        window = curses.initscr()     
        window.nodelay(1)
        ch = window.getch()
        if ch >= 0:
         print(ch) 
         curses.endwin()
         break
         
        curses.endwin()
          
    print('done')
    m.set_wrench_body_force((0.0, 0.0, 0.0))
     
   
    #save data
    thefile = open('data.txt', 'w')
    for item in range(len(force_list)):
 	 thefile.write("%s" % force_list[item])
 	 thefile.write("%s" % intergrator_list[item])
 	 thefile.write("%s" % derivator_list[item])
 	 thefile.write("%s\n" % forcedirection_list[item])
   
