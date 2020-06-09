import numpy as np
import math as m
import time
import queue
from mayavi import mlab
from tvtk.api import tvtk
import matplotlib.axes as ax
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QHBoxLayout, QLineEdit, QLabel, QFormLayout 
import matplotlib.animation as animation
from PyQt5.QtCore import pyqtSlot


class Sat:
    pos = np.zeros(3)
    last_pos = np.zeros((10000,3))
    vel = np.zeros(3)
    accel = np.zeros(3).astype(float)
    att = np.zeros(2) #attitude array containing pitch and yaw (in radians)
    datt = np.zeros(2) #delta attitude or velocity of spin
    spin_vector = np.zeros(3) #attitude descibed in 3 cartesian unit vectors (used for ploting only)
    G = 6.67408*pow(10,-11)
    M = 5.972*pow(10,24)
    
    def spinSat(self, dt):
        for i in [0,1]:
            self.att[i] = self.att[i] + self.datt[i]*dt
            if self.att[i] > 2*m.pi:
                self.att[i] -= 2*m.pi 
    
    def updateSpinVector(self):
        r = m.cos(self.att[0]) 
        if self.spin_vector[0] != 0 or self.spin_vector[1] != 0 or self.spin_vector[2] != 0: 
            R = m.sqrt(self.spin_vector[0]**2+self.spin_vector[1]**2+self.spin_vector[2]**2)
        else:
            R = 1  
        self.spin_vector[0] = r*m.cos(self.att[1])/R
        self.spin_vector[1] = r*m.sin(self.att[1])/R
        self.spin_vector[2] = m.sin(self.att[0])/R  
    
    def period(self):
        v = m.sqrt(self.vel[0]**2+self.vel[1]**2+self.vel[2]**2)
        R = m.sqrt(self.pos[0]**2+self.pos[1]**2+self.pos[2]**2)
        mu = self.G*self.M
        e = (v**2)/2-(mu/abs(R))
        a = -mu/(2*e)
        T = 2*m.pi*m.sqrt(a**3/mu)
        return(T)
    
    def __init__(self, _pos, _vel, _att):
        self.pos = _pos
        self.vel = _vel
        self.att = _att
        self.updateSpinVector()	
        updateGravity(self)
        self.last_pos = np.zeros((int(self.period()/10)*2, 3))

##end Sat##

class Planet:
    att = np.zeros(3)
    datt = np.zeros(3)

def spinEarth(_sat, dt):
	R = np.sqrt(_sat.pos[0]**2 + _sat.pos[1]**2 + _sat.pos[2]**2) 
	r = np.sqrt(_sat.pos[0]**2 + _sat.pos[1]**2) 
	if _sat.pos[1] != 0:
		theta = m.atan(_sat.pos[0]/_sat.pos[1]) + 2*np.radians(0.0041780746219999996202)*dt 
	else:
		theta = 0
	phi = m.asin(_sat.pos[2]/R)
	#accel_r = accel*m.cos(phi)##accel_r is acceleration allong  r, where r is the vector projected onto the theta plane
	if _sat.pos[0] > 0:
		_sat.pos[0] = r*m.sin(theta)
		if sat.pos[1] < 0:
			_sat.pos[0] = -_sat.pos[0] 
	else:
		_sat.pos[0] = -r*m.sin(theta) 
		if sat.pos[1] > 0:
			_sat.pos[0] = -_sat.pos[0] 
	if _sat.pos[1] > 0:
		_sat.pos[1] = r*m.cos(theta)
	else:
                _sat.pos[1] = -r*m.cos(theta)
	return sat	


def updateGravity(_sat):
    R = np.sqrt(_sat.pos[0]**2 + _sat.pos[1]**2 + _sat.pos[2]**2) 
    accel = -_sat.G * _sat.M / pow(R,2) #calculate magnitude of accel 
    ##calculate z accel
    phi = m.asin(_sat.pos[2]/R) 
    _sat.accel[2] = accel*m.sin(phi)
    ##the following shit-show of an if else block calculate x and y accel given different cases and quadrents
    if _sat.pos[0] == 0 and _sat.pos[1] == 0:       ##for when we are directly over a pole
        _sat.accel[0] = 0
        _sat.accel[1] = 0
    else:
        if _sat.pos[1] == 0:                        ##this if handles special case which really only happens in perticular polar obit
            phi = m.asin(_sat.pos[2]/R) 
            _sat.accel[1] = 0
            if _sat.pos[0] > 0:
                _sat.accel[0] = accel*m.cos(phi)
            else:
                _sat.accel[0] = -accel*m.cos(phi) 
        else:                                       ##general case
            theta = m.atan(_sat.pos[0]/_sat.pos[1])
            phi = m.asin(_sat.pos[2]/R)
            accel_r = accel*m.cos(phi)##accel_r is acceleration allong  r, where r is the vector projected onto the theta plane
            if _sat.pos[0] > 0:
                _sat.accel[0] = accel_r*m.sin(theta)
                if sat.pos[1] < 0:
                    _sat.accel[0] = -_sat.accel[0] 
            else:
                _sat.accel[0] = -accel_r*m.sin(theta) 
                if _sat.pos[1] > 0:
                    _sat.accel[0] = -_sat.accel[0] 
            if _sat.pos[1] > 0:
                _sat.accel[1] = accel_r*m.cos(theta)
            else:
                _sat.accel[1] = -accel_r*m.cos(theta)

def stepTime(sat, total_time, dt):
    steps = int(total_time/dt)
    #this function is made to take multiple steps in one call 
    time_array = np.linspace(0,total_time,steps)
    for t in range(len(time_array)):
        sat = spinEarth(sat, dt)
        sat.spinSat(dt)
        sat.updateSpinVector()
        updateGravity(sat)
        for i in [0,1,2]:
            sat.last_pos[:-1,i] = sat.last_pos[1:,i]
            sat.last_pos[-1,i] = sat.pos[i]
            sat.vel[i] = sat.vel[i] + sat.accel[i]*(dt/2)
            sat.pos[i] = sat.pos[i] + sat.vel[i]*dt 
        updateGravity(sat)
        for i in [0,1,2]:
            sat.vel[i] = sat.vel[i] + sat.accel[i]*(dt/2)
    return sat

def setupGroundMap():
	fig = plt.figure()
	plt.ion()
	img = plt.imread('blue_marble_5400x2700.jpg')
	plt.imshow(img, extent=[-180,180,-90,90])
	track, = plt.plot([], [], 'y.', markersize=0.5)
	sat_marker, = plt.plot(0,0, 'r.', markersize=7)
	picture_marker, = plt.plot([], [], 'm.', markersize=4)
	return track, sat_marker, picture_marker 

def updateGroundMap(_ground_track, _sat_marker, _sat, _picture_marker, _picture_points):
	#convert sat's x,y,z to long and lat
	R = np.sqrt(_sat.last_pos[:,0]**2 + _sat.last_pos[:,1]**2 + _sat.last_pos[:,2]**2) 
	theta = np.arctan2(_sat.last_pos[:,0],_sat.last_pos[:,1]) #long
	phi = np.arcsin(_sat.last_pos[:,2]/R)  #lat
	theta = np.degrees(theta)
	phi = np.degrees(phi)
	#now update and plot
	_ground_track.set_data(theta,phi)

	#find the long and lat point that's in the centre of all the data (this is where the sat is)
	sat_phi = phi[int(_sat.period()/10)]
	sat_theta = theta[int(_sat.period()/10)]
	#now update and plot th sat's pos
	_sat_marker.set_data(sat_theta,sat_phi)
	
	#convert the picture points from x,y,x to long and lat
	R = np.sqrt(_picture_points[:,0]**2 + _picture_points[:,1]**2 + _picture_points[:,2]**2) 
	theta = np.arctan2(_picture_points[:,0],_picture_points[:,1]) #long
	phi = np.arcsin(_picture_points[:,2]/R)  #lat
	theta = np.degrees(theta)
	phi = np.degrees(phi)
	#now update and plot
	_picture_marker.set_data(theta,phi)

def createSphere(fig, image_file):

    # load and map the texture
    img = tvtk.JPEGReader()
    img.file_name = image_file
    texture = tvtk.Texture(input_connection=img.output_port, interpolate=1)
    # (interpolate for a less raster appearance when zoomed in)

    # use a TexturedSphereSource, a.k.a. getting our hands dirty
    R = 6371*1000 
    Nrad = 360 

    # create the sphere source with a given radius and angular resolution
    sphere = tvtk.TexturedSphereSource(radius=R, theta_resolution=Nrad,
                                       phi_resolution=Nrad)

    # assemble rest of the pipeline, assign texture    
    sphere_mapper = tvtk.PolyDataMapper(input_connection=sphere.output_port)
    sphere_actor = tvtk.Actor(mapper=sphere_mapper, texture=texture)
    fig.scene.add_actor(sphere_actor)
    sphere_actor.rotate_z(180)
    return fig, sphere_actor
##end createSphere##

def simulate(sim_time, dt, sat, ground_track, sat_marker, picture_marker, picture_points):
	elapsed_time = 0 
	start_time = time.time()
	step_start_time = start_time 
	time_deviation = 0

	for i in [0,1,2]:
		sat.vel[i] = - sat.vel[i]
	sat = stepTime(sat, int(sat.period()), 10)
	for i in [0,1,2]:
		sat.vel[i] = - sat.vel[i]
	sat = stepTime(sat, int(sat.period())*2, 10)
	while elapsed_time < sim_time:
		##SIMULATION LOOP##
		#step through time
		sat = stepTime(sat, dt, dt)
	
		## create 2d map with track 
		updateGroundMap(ground_track, sat_marker, sat, picture_marker, picture_points)		
		plt.pause(0.05)		
		plt.draw()
	
		## create 3d globe with track 
		mlab.clf()
		scaling_factor = m.sqrt(400000**2/3)	
		line = mlab.quiver3d(sat.last_pos[int(sat.period()/dt),1],sat.last_pos[int(sat.period()/dt),0],sat.last_pos[int(sat.period()/dt),2],sat.spin_vector[0],sat.spin_vector[1],sat.spin_vector[2], scale_factor=400000, line_width=2, figure=fig, color=(1,0,0), mode='2darrow')
		mlab.points3d(picture_points[:,0], picture_points[:,1], picture_points[:,2], figure = fig, scale_factor = 200000, color=(1,0,1))		
		mlab.draw()
		#make sure step only last one second in real time
		if abs(time.time()-step_start_time) <= 1:
			if 1-abs(time.time()-step_start_time)-time_deviation > 0:
				time.sleep(1-abs(time.time()-step_start_time)-time_deviation)
			else:
				time.sleep(1-abs(time.time()-step_start_time))
		step_start_time = time.time()
		elapsed_time = time.time() - start_time
		print(elapsed_time)
		time_deviation = abs(start_time-time.time()) % 1

	
if __name__ == "__main__":
	##SETUP##
	picture_points = np.array([[6371*1000, 0, 0],[0,6371*1000,0]])
	sat = Sat(np.array([0,(6771*1000),0]).astype(float), np.array([5000,0,5000]).astype(float), np.array([0,0]).astype(float))#polar orbit is 8000m/s
	ground_track, sat_marker, picture_marker = setupGroundMap()
	fig = mlab.figure(size = (600,600), bgcolor=(0,0,0))
	image_file = 'blue_marble_5400x2700.jpg'
	fig, sphere = createSphere(fig, image_file)
	sim_time = 50	
	dt = 10 #for some reason this number can't be exactly one
	period = int(sat.period())
	earth_rotation_time = 24*60*60

	sat.datt[1] = 2*m.pi/(sat.period())
	simulate(sim_time, dt, sat, ground_track, sat_marker, picture_marker, picture_points)
#	sat = stepTime(sat, earth_rotation_time, dt)
	updateGroundMap(ground_track, sat_marker, sat, picture_marker, picture_points)
	longitude = 0
	latitude = 0
	plt.show()
	mlab.show()

#	@pyqtSlot()
#	def on_click(self):
#		global longitude
#		longitude = self.text()
#
#	long_text = QLineEdit()
#	button = QPushButton("Submit")
#	button.clicked.connect(on_click)	


#	app = QApplication([])
#	window = QWidget()
#	layout = QFormLayout()
#	layout.addWidget(QLabel('Longitude:'))
#	layout.addWidget(long_text)
#	layout.addWidget(QLabel('Latitude:'))
#	layout.addWidget(QLineEdit())
#	layout.addWidget(button)
#	window.setLayout(layout)
#	window.show()
#	app.exec_()

#	print(longitude)

