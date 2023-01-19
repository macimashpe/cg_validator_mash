import numpy as np

from .BaseModel import BaseModel

# CONSTANTS
# LD250 platform mass [kg]
PLATFORM_MASS = 146
# distance from center of drive wheel to front/rear caster pivot [m]
DRIVE_WHEEL_TO_CASTER_PIVOT_L = 0.285
# caster swivel radius, from caster pivot to caster wheel center [m]
CASTER_SWIVEL_RADIUS_r = 0.03175
# acceleration due to gravity
G = 9.8
# width from robot center to caster swivel center [m]
CASTER_PIVOT_TO_PLATFORM_CENTER_Y_Dc = 0.465/2
# width from drive wheel center to caster swivel center [m]
DRIVE_WHEEL_TO_PLATFORM_CENTER_Y_Dd = 0.605/2
# height of platform [m]
PLATFORM_Z = 0.38
# distance between floor and bottom of platform [m]
GROUND_CLEARANCE = 0.04
# stiffness of rear rocker and/or caster
REAR_CASTER_STIFFNESS = 3.75
# stiffness of front rocker and/or caster
FRONT_CASTER_STIFFNESS = 3.75
# ramp angle [deg]
RAMP_ANGLE = 0
# static friction coefficient
STATIC_FRICTION_COEFFICIENT = 0.76
# Dynamic friction coefficient
DYNAMIC_FRICTION_COEFFICIENT = 0.63

# fixed drive wheel down force and no rockers
class LD250(BaseModel):
    def __init__(self, payload_mass=250):
        # BaseModel.__init__(self)
        self.name = 'LD250'
        # Fixed robot parameters LD250
        self.L = DRIVE_WHEEL_TO_CASTER_PIVOT_L
        self.r = CASTER_SWIVEL_RADIUS_r
        #don't believe these are used in LD250
        # self.l1 = self.L * self.k
        # self.l2 = self.L * (1-self.k)
        # self.h2 = 0.1 
        # self.h1 = 0.05
        self._platform_mass = PLATFORM_MASS
        self.payload_mass = payload_mass
        # self._total_mass = self.payload_mass+self._platform_mass  # not needed, handled in decorator
        self._JzRobot = 32*self._platform_mass/223
        # self.JzPayload = 43.28*self.payload_mass/600 # not needed, handled in decorator
        # self.Jz = 85.3*self._total_mass/823  # not needed, handled in decorator
        self._g = G
        self._caster_pivot_to_platform_center_y_Dc = CASTER_PIVOT_TO_PLATFORM_CENTER_Y_Dc
        self._drive_wheel_to_platform_center_y_Dd = DRIVE_WHEEL_TO_PLATFORM_CENTER_Y_Dd
        self._platform_z = PLATFORM_Z
        self._ground_clearance = GROUND_CLEARANCE
        self._platform_cg = [0, 0, (self._platform_z - self._ground_clearance) / 2 + self._ground_clearance]
        self.kr = REAR_CASTER_STIFFNESS
        self.kf = FRONT_CASTER_STIFFNESS
        self.kk = self.kr/(self.kr+self.kf)
        self._caster_resistance_coefficient = 0.024 # rolling resistance coefficiency for casters, prev urc
        self._drive_wheel_resistance_coefficient = 0.026 # rolling resistance coefficiency for drive wheel, prev urd
        self.us = STATIC_FRICTION_COEFFICIENT # Static friction coefficiency
        self.ud = DYNAMIC_FRICTION_COEFFICIENT # Dynamic coefficientcy
        self.u = self.us

        self.downForce = 460 # drive wheel downward force
        self.maxDriveAccelF = 1.25 * 30 / (0.2032 / 2) # single drive wheel
        self.maxDriveDecelF = self.maxDriveAccelF * 3 # because of the gear box, decel force is larger than the accel force under same motor current

        self.velocity = 2.2
        self.acceleration = 0.8
        self.centripetal_acceleration = 1
        self.R = self.velocity**2/self.centripetal_acceleration # robot trajectory radius
        self.w = self.velocity / self.R # robot angular velocity

                # caster angles, [rad]
        self._rear_right_caster_angle = 0 # Rear caster 1 angle during corning, Br1
        self._front_right_caster_angle = 0 # Front caster 1 angle during corning, Bf1
        self._rear_left_caster_angle = 0 # Rear caster 2 angle during corning, Br2
        self._front_left_caster_angle = 0 # Front caster 2 angle during corning, Bf2

        self.dir = 1
        self.theta = np.rad2deg(RAMP_ANGLE)

        self.solve_caster_angle()

    # xyh here is the overall CG
    def modelNoBrake(self, x,y,h):
        if self.centripetal_acceleration != 0:
            alphaz = self.acceleration/self.R
        else:
            alphaz = 0
        Mk = np.array( [[0,	-np.cos(self._rear_right_caster_angle),	0,	-np.cos(self._rear_left_caster_angle),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self._front_right_caster_angle),	0,	-np.cos(self._front_left_caster_angle)],
                        [0,		np.sin(self._rear_right_caster_angle),	0,	np.sin(self._rear_left_caster_angle),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self._front_right_caster_angle),	0, 	-np.sin(self._front_left_caster_angle)],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0],
                        [-(self._caster_pivot_to_platform_center_y_Dc-self.r*np.sin(self._rear_right_caster_angle)+y),	np.sin(self._rear_right_caster_angle)*h,		self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._rear_left_caster_angle)-y,	np.sin(self._rear_left_caster_angle)*h,		-(self._drive_wheel_to_platform_center_y_Dd+y),	0,	0,	self._drive_wheel_to_platform_center_y_Dd-y,	0,	0,	h,	-(self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._front_right_caster_angle)+y),	-np.sin(self._front_right_caster_angle)*h,		self._caster_pivot_to_platform_center_y_Dc-np.sin(self._front_left_caster_angle)*self.r-y,		-np.sin(self._front_left_caster_angle)*h],
                        [self.L+self.r*np.cos(self._rear_right_caster_angle)+x,	np.cos(self._rear_right_caster_angle)*h,		self.L+self.r*np.cos(self._rear_left_caster_angle)+x,	np.cos(self._rear_left_caster_angle)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(self.L-self.r*np.cos(self._front_right_caster_angle)-x),	np.cos(self._front_right_caster_angle)*h,		-(self.L-self.r*np.cos(self._front_left_caster_angle)-x),	np.cos(self._front_left_caster_angle)*h],
                        [0,		-np.cos(self._rear_right_caster_angle)*(self._caster_pivot_to_platform_center_y_Dc-self.r*np.sin(self._rear_right_caster_angle)+y)-np.sin(self._rear_right_caster_angle)*(self.L+np.cos(self._rear_right_caster_angle)*self.r+x),	0,	np.cos(self._rear_left_caster_angle)*(self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._rear_left_caster_angle)-y)-np.sin(self._rear_left_caster_angle)*(self.L+np.cos(self._rear_left_caster_angle)*self.r+x),	0,	-(self._drive_wheel_to_platform_center_y_Dd+y),	self._drive_wheel_to_platform_center_y_Dd+y,	0,	self._drive_wheel_to_platform_center_y_Dd-y,	-(self._drive_wheel_to_platform_center_y_Dd-y),	-x,	0,	-np.cos(self._front_right_caster_angle)*(self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._front_right_caster_angle)+y)-np.sin(self._front_right_caster_angle)*(self.L-np.cos(self._front_right_caster_angle)*self.r-x),	0,	np.cos(self._front_left_caster_angle)*(self._caster_pivot_to_platform_center_y_Dc-self.r*np.sin(self._front_left_caster_angle)-y)-np.sin(self._front_left_caster_angle)*(self.L-np.cos(self._front_left_caster_angle)*self.r-x)],
                        [-(self._caster_pivot_to_platform_center_y_Dc-self.r*np.sin(self._rear_right_caster_angle)+y),    self.kk*np.sin(self._rear_right_caster_angle)*h,	self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._rear_left_caster_angle)-y,	self.kk*np.sin(self._rear_left_caster_angle)*h,	-(self._drive_wheel_to_platform_center_y_Dd+y),	0,	0,	self._drive_wheel_to_platform_center_y_Dd-y,	0,	0,	self.kk*h,	0,	-self.kk*np.sin(self._front_right_caster_angle)*h,	0,	-self.kk*np.sin(self._front_left_caster_angle)*h],
                        [-self._caster_resistance_coefficient,		1,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		-self._caster_resistance_coefficient,	1,	    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		-self._drive_wheel_resistance_coefficient,	1*self.dir,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	-self._drive_wheel_resistance_coefficient,	1*self.dir,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	-self._caster_resistance_coefficient,	1,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	-self._caster_resistance_coefficient,	1],
                        [0,		    0,	    0,	    0,      1,	0,	0,	0,	0,	0,	0,	0,	0,	0,			0],
                        [0,			0,		0,		0,		0,	0,	0,	1,	0,	0,	0,	0,	0,	0,			0]])
        Y = np.array([[self._total_mass*(self.acceleration*(self.R-y)/self.R - self.w**2*x) + self._total_mass*self._g*np.sin(self.theta)], [self._total_mass*(self.w**2*(self.R-y) + self.acceleration*x/self.R)], [self._total_mass*self._g*np.cos(self.theta)],[0], [0], [self._Jz*alphaz],[0],[0],[0],[0],[0],[0],[0],[self.downForce],[self.downForce]])
        # [Frz1;Frf1;Frz2;Frf2;Fdz1;Fdf1;Fdt1;Fdz2;Fdf2;Fdt2;Fc;Ffz1;Fff1;Ffz2;Fff2];
        #     0;    1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14  
        return np.linalg.solve(Mk,Y)

    # xyh here is the overall CG
    def modelBrake(self, x,y,h):
        self.dynamicU()
        if self.ac != 0:
            ux = 0.95
        else:
            ux = 1
        Mk = np.array( [[0,	-np.cos(self._rear_right_caster_angle),	0,	-np.cos(self._rear_left_caster_angle),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self._front_right_caster_angle),	0,	-np.cos(self._front_left_caster_angle), -self._total_mass, 0, 0],
                        [0,		np.sin(self._rear_right_caster_angle),	0,	np.sin(self._rear_left_caster_angle),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self._front_right_caster_angle),	0, 	-np.sin(self._front_left_caster_angle), 0, -self._total_mass, 0],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0, 0, 0, 0],
                        [-(self._caster_pivot_to_platform_center_y_Dc-self.r*np.sin(self._rear_right_caster_angle)+y),	np.sin(self._rear_right_caster_angle)*h,		self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._rear_left_caster_angle)-y,	np.sin(self._rear_left_caster_angle)*h,		-(self._drive_wheel_to_platform_center_y_Dd+y),	0,	0,	self._drive_wheel_to_platform_center_y_Dd-y,	0,	0,	h,	-(self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._front_right_caster_angle)+y),	-np.sin(self._front_right_caster_angle)*h,		self._caster_pivot_to_platform_center_y_Dc-np.sin(self._front_left_caster_angle)*self.r-y,		-np.sin(self._front_left_caster_angle)*h, 0, 0, 0],
                        [self.L+self.r*np.cos(self._rear_right_caster_angle)+x,	np.cos(self._rear_right_caster_angle)*h,		self.L+self.r*np.cos(self._rear_left_caster_angle)+x,	np.cos(self._rear_left_caster_angle)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(self.L-self.r*np.cos(self._front_right_caster_angle)-x),	np.cos(self._front_right_caster_angle)*h,		-(self.L-self.r*np.cos(self._front_left_caster_angle)-x),	np.cos(self._front_left_caster_angle)*h, 0, 0, 0],
                        [0,		-np.cos(self._rear_right_caster_angle)*(self._caster_pivot_to_platform_center_y_Dc-self.r*np.sin(self._rear_right_caster_angle)+y)-np.sin(self._rear_right_caster_angle)*(self.L+np.cos(self._rear_right_caster_angle)*self.r+x),	0,	np.cos(self._rear_left_caster_angle)*(self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._rear_left_caster_angle)-y)-np.sin(self._rear_left_caster_angle)*(self.L+np.cos(self._rear_left_caster_angle)*self.r+x),	0,	-(self._drive_wheel_to_platform_center_y_Dd+y),	self._drive_wheel_to_platform_center_y_Dd+y,	0,	self._drive_wheel_to_platform_center_y_Dd-y,	-(self._drive_wheel_to_platform_center_y_Dd-y),	-x,	0,	-np.cos(self._front_right_caster_angle)*(self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._front_right_caster_angle)+y)-np.sin(self._front_right_caster_angle)*(self.L-np.cos(self._front_right_caster_angle)*self.r-x),	0,	np.cos(self._front_left_caster_angle)*(self._caster_pivot_to_platform_center_y_Dc-self.r*np.sin(self._front_left_caster_angle)-y)-np.sin(self._front_left_caster_angle)*(self.L-np.cos(self._front_left_caster_angle)*self.r-x),0,	0, -self._Jz],
                        [-(self._caster_pivot_to_platform_center_y_Dc-self.r*np.sin(self._rear_right_caster_angle)+y),	self.kk*np.sin(self._rear_right_caster_angle)*h,			self._caster_pivot_to_platform_center_y_Dc+self.r*np.sin(self._rear_left_caster_angle)-y,	self.kk*np.sin(self._rear_left_caster_angle)*h,			-(self._drive_wheel_to_platform_center_y_Dd+y),	0,	0,	self._drive_wheel_to_platform_center_y_Dd-y,	0,	0,	self.kk*h,	0,			-self.kk*np.sin(self._front_right_caster_angle)*h,		0,			-self.kk*np.sin(self._front_left_caster_angle)*h,		0,	0,	0],
                        [-self._caster_resistance_coefficient,		1,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		-self._caster_resistance_coefficient,	1,	    0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		-self._drive_wheel_resistance_coefficient,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	 -self._drive_wheel_resistance_coefficient,	1*self.dir,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	-self._caster_resistance_coefficient,	1,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	-self._caster_resistance_coefficient,	1, 0, 0, 0],
                        [0,			0,		0,		0,	 ux*self.u,	0,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,   ux*self.u,	0,	1*self.dir,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,		    0,		0,	    0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 1,-x],
                        [0,		    0,	    0,	    0,      1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,  0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,  0, 0, 0]])
        Y = np.array([[self._total_mass*self._g*np.sin(self.theta)], [0], [self._total_mass*self._g*np.cos(self.theta)],[0], [0], [0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[self.w**2*(self.R-y)],[self.downForce],[self.downForce]])
        # [Frz1;Frf1;Frz2;Frf2;Fdz1;Fdf1;Fdt1;Fdz2;Fdf2;Fdt2;Fc;Ffz1;Fff1;Ffz2;Fff2;aCGx;aCGy;alphaZ];
        #     0;   1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14;  15;  16;    17  
        self.staticU()
        try:
            return np.linalg.solve(Mk,Y)
        except:
            print('Singular Matrix at: ' + str(x) + ', ' + str(y) + ', ' + str(h))
            Y[0] = -100
            return Y # return an array that will fail in criterion check