import numpy as np
from .BaseModel import BaseModel

# CONSTANTS
# LD250 platform mass [kg]
PLATFORM_MASS = 146
# rocker ratio, (drive wheel to pivot pin) / (drive wheel to caster)
# TODO: confirm this makes sense in LD250 model that has no rockers
ROCKER_RATIO_k = 0.48
# distance from center of drive wheel to front/rear caster pivot [m]
DRIVE_WHEEL_TO_CASTER_PIVOT_L = 0.285
# caster swivel radius, from caster swivel to caster wheel center [m]
CASTER_SWIVEL_RADIUS_r = 0.03175
# drive wheel to rocker pivot. Doesn't seem used by LD250 model
# DRIVE_WHEEL_TO_ROCKER_PIVOT_l1 = DRIVE_WHEEL_TO_CASTER_PIVOT_L * ROCKER_RATIO_k. Doesn't seem used by LD250 model
# self.l2 = DRIVE_WHEEL_TO_CASTER_PIVOT_L * (1-ROCKER_RATIO_k). Doesn't seem used by LD250 model
# self.h2 = 0.1. Doesn't seem used by LD250 model
# self.h1 = 0.05. Doesn't seem used by LD250 model

# fixed drive wheel down force and no rockers
class LD250(BaseModel):
    def __init__(self, payload_mass=250):
        BaseModel.__init__(self)
        self.name = 'LD250'
        # Fixed robot parameters LD250
        self.platform_mass = PLATFORM_MASS # prev 'Mrobot'
        self.JzRobot = 32*self.platform_mass/223
        self.payload_mass = payload_mass
        # self.JzPayload = 43.28*self.payload_mass/600  # not needed with payload_mass property decorator
        # self.Jz = 85.3*self.total_mass/823  # not needed with payload_mass property decorator
        # self.total_mass = self.payload_mass+self.platform_mass # prev 'M', not needed with platform_mass property decorator
        # self.g = 9.8, not needed since initialized in BaseModel
        self.Dc = 0.465/2
        self.Dd = 0.605/2
        self.robotH = 0.38
        self.platform_cg = [0, 0, (self.robotH - 0.04) / 2 + 0.04]  # prev pRobot
        self.kr = 3.75
        self.kf = 3.75
        self.kk = self.kr/(self.kr+self.kf)

        self.downForce = 460 # drive wheel downward force
        self.maxDriveAccelF = 1.25 * 30 / (0.2032 / 2) # single drive wheel
        self.maxDriveDecelF = self.maxDriveAccelF * 3 # because of the gear box, decel force is larger than the accel force under same motor current

        self.solve_caster_angle()

    # xyh here is the overall CG
    def modelNoBrake(self, x,y,h):
        if self.centripetal_acceleration != 0:
            alphaz = self.ax/self.R
        else:
            alphaz = 0
        Mk = np.array( [[0,	-np.cos(self.rear_right_caster_angle),	0,	-np.cos(self.rear_left_caster_angle),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self.front_right_caster_angle),	0,	-np.cos(self.front_left_caster_angle)],
                        [0,		np.sin(self.rear_right_caster_angle),	0,	np.sin(self.rear_left_caster_angle),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self.front_right_caster_angle),	0, 	-np.sin(self.front_left_caster_angle)],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0],
                        [-(self.Dc-CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_right_caster_angle)+y),	np.sin(self.rear_right_caster_angle)*h,		self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_left_caster_angle)-y,	np.sin(self.rear_left_caster_angle)*h,		-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	h,	-(self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.front_right_caster_angle)+y),	-np.sin(self.front_right_caster_angle)*h,		self.Dc-np.sin(self.front_left_caster_angle)*CASTER_SWIVEL_RADIUS_r-y,		-np.sin(self.front_left_caster_angle)*h],
                        [DRIVE_WHEEL_TO_CASTER_PIVOT_L+CASTER_SWIVEL_RADIUS_r*np.cos(self.rear_right_caster_angle)+x,	np.cos(self.rear_right_caster_angle)*h,		DRIVE_WHEEL_TO_CASTER_PIVOT_L+CASTER_SWIVEL_RADIUS_r*np.cos(self.rear_left_caster_angle)+x,	np.cos(self.rear_left_caster_angle)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(DRIVE_WHEEL_TO_CASTER_PIVOT_L-CASTER_SWIVEL_RADIUS_r*np.cos(self.front_right_caster_angle)-x),	np.cos(self.front_right_caster_angle)*h,		-(DRIVE_WHEEL_TO_CASTER_PIVOT_L-CASTER_SWIVEL_RADIUS_r*np.cos(self.front_left_caster_angle)-x),	np.cos(self.front_left_caster_angle)*h],
                        [0,		-np.cos(self.rear_right_caster_angle)*(self.Dc-CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_right_caster_angle)+y)-np.sin(self.rear_right_caster_angle)*(DRIVE_WHEEL_TO_CASTER_PIVOT_L+np.cos(self.rear_right_caster_angle)*CASTER_SWIVEL_RADIUS_r+x),	0,	np.cos(self.rear_left_caster_angle)*(self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_left_caster_angle)-y)-np.sin(self.rear_left_caster_angle)*(DRIVE_WHEEL_TO_CASTER_PIVOT_L+np.cos(self.rear_left_caster_angle)*CASTER_SWIVEL_RADIUS_r+x),	0,	-(self.Dd+y),	self.Dd+y,	0,	self.Dd-y,	-(self.Dd-y),	-x,	0,	-np.cos(self.front_right_caster_angle)*(self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.front_right_caster_angle)+y)-np.sin(self.front_right_caster_angle)*(DRIVE_WHEEL_TO_CASTER_PIVOT_L-np.cos(self.front_right_caster_angle)*CASTER_SWIVEL_RADIUS_r-x),	0,	np.cos(self.front_left_caster_angle)*(self.Dc-CASTER_SWIVEL_RADIUS_r*np.sin(self.front_left_caster_angle)-y)-np.sin(self.front_left_caster_angle)*(DRIVE_WHEEL_TO_CASTER_PIVOT_L-np.cos(self.front_left_caster_angle)*CASTER_SWIVEL_RADIUS_r-x)],
                        [-(self.Dc-CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_right_caster_angle)+y),    self.kk*np.sin(self.rear_right_caster_angle)*h,	self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_left_caster_angle)-y,	self.kk*np.sin(self.rear_left_caster_angle)*h,	-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	self.kk*h,	0,	-self.kk*np.sin(self.front_right_caster_angle)*h,	0,	-self.kk*np.sin(self.front_left_caster_angle)*h],
                        [-self.urc,		1,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		-self.urc,	1,	    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		-self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	-self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	-self.urc,	1,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	-self.urc,	1],
                        [0,		    0,	    0,	    0,      1,	0,	0,	0,	0,	0,	0,	0,	0,	0,			0],
                        [0,			0,		0,		0,		0,	0,	0,	1,	0,	0,	0,	0,	0,	0,			0]])
        Y = np.array([[self.total_mass*(self.ax*(self.R-y)/self.R - self.w**2*x) + self.total_mass*self.g*np.sin(self.theta)], [self.total_mass*(self.w**2*(self.R-y) + self.ax*x/self.R)], [self.total_mass*self.g*np.cos(self.theta)],[0], [0], [self.Jz*alphaz],[0],[0],[0],[0],[0],[0],[0],[self.downForce],[self.downForce]])
        # [Frz1;Frf1;Frz2;Frf2;Fdz1;Fdf1;Fdt1;Fdz2;Fdf2;Fdt2;Fc;Ffz1;Fff1;Ffz2;Fff2];
        #     0;    1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14  
        return np.linalg.solve(Mk,Y)

    # xyh here is the overall CG
    def modelBrake(self, x,y,h):
        self.dynamicU()
        if self.centripetal_acceleration != 0:
            ux = 0.95
        else:
            ux = 1
        Mk = np.array( [[0,	-np.cos(self.rear_right_caster_angle),	0,	-np.cos(self.rear_left_caster_angle),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self.front_right_caster_angle),	0,	-np.cos(self.front_left_caster_angle), -self.total_mass, 0, 0],
                        [0,		np.sin(self.rear_right_caster_angle),	0,	np.sin(self.rear_left_caster_angle),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self.front_right_caster_angle),	0, 	-np.sin(self.front_left_caster_angle), 0, -self.total_mass, 0],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0, 0, 0, 0],
                        [-(self.Dc-CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_right_caster_angle)+y),	np.sin(self.rear_right_caster_angle)*h,		self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_left_caster_angle)-y,	np.sin(self.rear_left_caster_angle)*h,		-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	h,	-(self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.front_right_caster_angle)+y),	-np.sin(self.front_right_caster_angle)*h,		self.Dc-np.sin(self.front_left_caster_angle)*self.r-y,		-np.sin(self.front_left_caster_angle)*h, 0, 0, 0],
                        [DRIVE_WHEEL_TO_CASTER_PIVOT_L+CASTER_SWIVEL_RADIUS_r*np.cos(self.rear_right_caster_angle)+x,	np.cos(self.rear_right_caster_angle)*h,		DRIVE_WHEEL_TO_CASTER_PIVOT_L+CASTER_SWIVEL_RADIUS_r*np.cos(self.rear_left_caster_angle)+x,	np.cos(self.rear_left_caster_angle)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(DRIVE_WHEEL_TO_CASTER_PIVOT_L-CASTER_SWIVEL_RADIUS_r*np.cos(self.front_right_caster_angle)-x),	np.cos(self.front_right_caster_angle)*h,		-(DRIVE_WHEEL_TO_CASTER_PIVOT_L-CASTER_SWIVEL_RADIUS_r*np.cos(self.front_left_caster_angle)-x),	np.cos(self.front_left_caster_angle)*h, 0, 0, 0],
                        [0,		-np.cos(self.rear_right_caster_angle)*(self.Dc-CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_right_caster_angle)+y)-np.sin(self.rear_right_caster_angle)*(DRIVE_WHEEL_TO_CASTER_PIVOT_L+np.cos(self.rear_right_caster_angle)*self.r+x),	0,	np.cos(self.rear_left_caster_angle)*(self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_left_caster_angle)-y)-np.sin(self.rear_left_caster_angle)*(DRIVE_WHEEL_TO_CASTER_PIVOT_L+np.cos(self.rear_left_caster_angle)*self.r+x),	0,	-(self.Dd+y),	self.Dd+y,	0,	self.Dd-y,	-(self.Dd-y),	-x,	0,	-np.cos(self.front_right_caster_angle)*(self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.front_right_caster_angle)+y)-np.sin(self.front_right_caster_angle)*(DRIVE_WHEEL_TO_CASTER_PIVOT_L-np.cos(self.front_right_caster_angle)*self.r-x),	0,	np.cos(self.front_left_caster_angle)*(self.Dc-CASTER_SWIVEL_RADIUS_r*np.sin(self.front_left_caster_angle)-y)-np.sin(self.front_left_caster_angle)*(DRIVE_WHEEL_TO_CASTER_PIVOT_L-np.cos(self.front_left_caster_angle)*self.r-x),0,	0, -self.Jz],
                        [-(self.Dc-CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_right_caster_angle)+y),	self.kk*np.sin(self.rear_right_caster_angle)*h,			self.Dc+CASTER_SWIVEL_RADIUS_r*np.sin(self.rear_left_caster_angle)-y,	self.kk*np.sin(self.rear_left_caster_angle)*h,			-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	self.kk*h,	0,			-self.kk*np.sin(self.front_right_caster_angle)*h,		0,			-self.kk*np.sin(self.front_left_caster_angle)*h,		0,	0,	0],
                        [-self.urc,		1,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		-self.urc,	1,	    0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		-self.urd,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	 -self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	-self.urc,	1,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	-self.urc,	1, 0, 0, 0],
                        [0,			0,		0,		0,	 ux*self.u,	0,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,   ux*self.u,	0,	1*self.dir,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,		    0,		0,	    0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 1,-x],
                        [0,		    0,	    0,	    0,      1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,  0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,  0, 0, 0]])
        Y = np.array([[self.total_mass*self.g*np.sin(self.theta)], [0], [self.total_mass*self.g*np.cos(self.theta)],[0], [0], [0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[self.w**2*(self.R-y)],[self.downForce],[self.downForce]])
        # [Frz1;Frf1;Frz2;Frf2;Fdz1;Fdf1;Fdt1;Fdz2;Fdf2;Fdt2;Fc;Ffz1;Fff1;Ffz2;Fff2;aCGx;aCGy;alphaZ];
        #     0;   1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14;  15;  16;    17  
        self.staticU()
        try:
            return np.linalg.solve(Mk,Y)
        except:
            print('Singular Matrix at: ' + str(x) + ', ' + str(y) + ', ' + str(h))
            Y[0] = -100
            return Y # return an array that will fail in criterion check