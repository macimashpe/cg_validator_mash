import numpy as np

from .BaseModel import BaseModel

DRIVE_WHEEL_TO_CASTER_PIVOT_L = 0.405
CASTER_SWIVEL_RADIUS_r = 0.058
ROCKER_RATIO_k = 0.48
PIVOT_PIN_HEIGHT_h1 = 0.05
REAR_ROCKER_PIVOT_PIN_HEIGHT_h2 = 0.11
CASTER_PIVOT_TO_PLATFORM_CENTER_Y_Dc = 0.292875
DRIVE_WHEEL_TO_PLATFORM_CENTER_Y_Dd = 0.3189
PLATFORM_Z = 0.320

class MD(BaseModel):
    def __init__(self, payload_mass=650):
        BaseModel.__init__(self)
        self.name = 'MD'
        self.velocity = 2.2  # robot vel max is 2.2 now
        # self.updateSpeed(self._velocity)  # not needed with velocity property decorator
        self.brakeDecel = -1.3 # defined as max
        self.L = DRIVE_WHEEL_TO_CASTER_PIVOT_L  # POC2: 0.370 # drive wheel to swivel center
        self.r = CASTER_SWIVEL_RADIUS_r  # POC2: 0.04 # caster swivel radius
        self.k = ROCKER_RATIO_k  # POC2: 130/370 # rocker ratio
        self.l1 = self.L * self.k
        self.l2 = self.L * (1-self.k)
        self.h1 = PIVOT_PIN_HEIGHT_h1   # rear rocker pivot pin height from the ground
        self.h2 = REAR_ROCKER_PIVOT_PIN_HEIGHT_h2  #  pivot pin height from the ground
        self.axa = 0.5  # acceleration in m/s^2
        self.ax = self.axa
        self.axd = -1.3  # deceleration in m/s^2
        self._centripetal_acceleration = 0.5  # centripetal accel
        self.R = self._velocity ** 2 / self._centripetal_acceleration  # robot trajactory radius
        self.w = self._velocity / self.R  # robot angular velocity
        '''# no knowledge on the following...
        # self.maxBrakeF = 99999  # Max Braking force
        # self.maxDriveF = 99999  # Max motor drive force
        # self.maxDriveDecelF = 99999  # Max motor force during decel
        # self.maxDriveAccelF = 99999  # Max motor force furing accel
        # self.L = 0.405 # POC2: 0.370
        # self.r = 0.058 # POC2: 0.04
        # self.k = 0.48 # POC2: 130/370'''
        # self.h2 = 0.1
        # self.h1 = 0.05
        self.platform_mass = 239 # shen = 230
        self.payload_mass = payload_mass
        self.M = self._payload_mass+self.platform_mass
        self.Dc = CASTER_PIVOT_TO_PLATFORM_CENTER_Y_Dc  #0.294  # caster swivel center to robot center distance in y direction
        self.Dd = DRIVE_WHEEL_TO_PLATFORM_CENTER_Y_Dd  #0.315  # drive wheel center to robot center distance in y direction
        self._platform_z = PLATFORM_Z
        self._platform_cg = [0, 0, (self._platform_z - 0.04) / 2 + 0.04]
        self.JzRobot = 39.32*self.platform_mass/223
        self.JzPayload = 43.28*self._payload_mass/600
        self.Jz = 85.3*self.M/823
        self.kr = 3.75
        self.kf = 1.63 # for MD 1.63
        self.kk = self.kr/(self.kr+self.kf)
        self.solve_caster_angle()
        self.updateBrakeT(10)

    # Update max brake torque, default is 10 N/m
    def updateBrakeT(self, T):
        if self._payload_mass == 900:
            self.maxBrakeF = T * 12.57 / .08 # 12.57 is the gear box ratio, 0.08 is the wheel radius
        else:
            self.maxBrakeF = T * 12.57 / .1

    # xyh here is the overall CG
    def modelNoBrake(self,x,y,h):
        if self._centripetal_acceleration != 0:
            alphaz = self.ax/self.R
        else:
            alphaz = 0
        Mk = np.array( [[0,	-np.cos(self._rear_right_caster_angle),	0,	-np.cos(self._rear_left_caster_angle),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self._front_right_caster_angle),	0,	-np.cos(self._front_left_caster_angle)],
                        [0,		np.sin(self._rear_right_caster_angle),	0,	np.sin(self._rear_left_caster_angle),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self._front_right_caster_angle),	0, 	-np.sin(self._front_left_caster_angle)],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0],
                        [-(self.Dc-self.r*np.sin(self._rear_right_caster_angle)+y),	np.sin(self._rear_right_caster_angle)*h,		self.Dc+self.r*np.sin(self._rear_left_caster_angle)-y,	np.sin(self._rear_left_caster_angle)*h,		-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	h,	-(self.Dc+self.r*np.sin(self._front_right_caster_angle)+y),	-np.sin(self._front_right_caster_angle)*h,		self.Dc-np.sin(self._front_left_caster_angle)*self.r-y,		-np.sin(self._front_left_caster_angle)*h],
                        [self.L+self.r*np.cos(self._rear_right_caster_angle)+x,	np.cos(self._rear_right_caster_angle)*h,		self.L+self.r*np.cos(self._rear_left_caster_angle)+x,	np.cos(self._rear_left_caster_angle)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(self.L-self.r*np.cos(self._front_right_caster_angle)-x),	np.cos(self._front_right_caster_angle)*h,		-(self.L-self.r*np.cos(self._front_left_caster_angle)-x),	np.cos(self._front_left_caster_angle)*h],
                        [0,		-np.cos(self._rear_right_caster_angle)*(self.Dc-self.r*np.sin(self._rear_right_caster_angle)+y)-np.sin(self._rear_right_caster_angle)*(self.L+np.cos(self._rear_right_caster_angle)*self.r+x),	0,	np.cos(self._rear_left_caster_angle)*(self.Dc+self.r*np.sin(self._rear_left_caster_angle)-y)-np.sin(self._rear_left_caster_angle)*(self.L+np.cos(self._rear_left_caster_angle)*self.r+x),	0,	-(self.Dd+y),	self.Dd+y,	0,	self.Dd-y,	-(self.Dd-y),	-x,	0,	-np.cos(self._front_right_caster_angle)*(self.Dc+self.r*np.sin(self._front_right_caster_angle)+y)-np.sin(self._front_right_caster_angle)*(self.L-np.cos(self._front_right_caster_angle)*self.r-x),	0,	np.cos(self._front_left_caster_angle)*(self.Dc-self.r*np.sin(self._front_left_caster_angle)-y)-np.sin(self._front_left_caster_angle)*(self.L-np.cos(self._front_left_caster_angle)*self.r-x)],
                        [0,		0,	0,		0,		self.l1,	self.h1,	-self.h1,	0,	0,	0,	0,	-(self.l2-self.r*np.cos(self._front_right_caster_angle)),	np.cos(self._front_right_caster_angle)*self.h1,		0,		0],
                        [0,		0,		0,		0,		0,	0,	0,	self.l1,	self.h1,	-self.h1,	0,	0,	0,	-(self.l2-self.r*np.cos(self._front_left_caster_angle)),	np.cos(self._front_left_caster_angle)*self.h1],
                        [-(self.Dc-self.r*np.sin(self._rear_right_caster_angle)),	np.sin(self._rear_right_caster_angle)*self.h2,		self.Dc+self.r*np.sin(self._rear_left_caster_angle),	np.sin(self._rear_left_caster_angle)*self.h2,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [-self.urc,		1,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		-self.urc,	1,	    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		-self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	-self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	-self.urc,	1,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	-self.urc,	1]])
        Y = np.array([[self.M*(self.ax*(self.R-y)/self.R - self.w**2*x) + self.M*self.g*np.sin(self.theta)], [self.M*(self.w**2*(self.R-y) + self.ax*x/self.R)], [self.M*self.g*np.cos(self.theta)],[0], [0], [self.Jz*alphaz], [0], [0],[0],[0],[0],[0],[0],[0],[0]])
        # [Frz1;Frf1;Frz2;Frf2;Fdz1;Fdf1;Fdt1;Fdz2;Fdf2;Fdt2;Fc;Ffz1;Fff1;Ffz2;Fff2];
        #     0;    1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14  
        return np.linalg.solve(Mk,Y)

    # xyh here is the overall CG
    def model_no_brake_2(self,x,y,h):
        if self._centripetal_acceleration != 0:
            alphaz = self.ax/self.R
        else:
            alphaz = 0
        Mk = np.array( [[0,	-np.cos(self._rear_right_caster_angle),	0,	-np.cos(self._rear_left_caster_angle),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self._front_right_caster_angle),	0,	-np.cos(self._front_left_caster_angle)],
                        [0,		np.sin(self._rear_right_caster_angle),	0,	np.sin(self._rear_left_caster_angle),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self._front_right_caster_angle),	0, 	-np.sin(self._front_left_caster_angle)],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0],
                        [-(self.Dc-self.r*np.sin(self._rear_right_caster_angle)+y),	np.sin(self._rear_right_caster_angle)*h,		self.Dc+self.r*np.sin(self._rear_left_caster_angle)-y,	np.sin(self._rear_left_caster_angle)*h,		-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	h,	-(self.Dc+self.r*np.sin(self._front_right_caster_angle)+y),	-np.sin(self._front_right_caster_angle)*h,		self.Dc-np.sin(self._front_left_caster_angle)*self.r-y,		-np.sin(self._front_left_caster_angle)*h],
                        [self.L+self.r*np.cos(self._rear_right_caster_angle)+x,	np.cos(self._rear_right_caster_angle)*h,		self.L+self.r*np.cos(self._rear_left_caster_angle)+x,	np.cos(self._rear_left_caster_angle)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(self.L-self.r*np.cos(self._front_right_caster_angle)-x),	np.cos(self._front_right_caster_angle)*h,		-(self.L-self.r*np.cos(self._front_left_caster_angle)-x),	np.cos(self._front_left_caster_angle)*h],
                        [0,		-np.cos(self._rear_right_caster_angle)*(self.Dc-self.r*np.sin(self._rear_right_caster_angle)+y)-np.sin(self._rear_right_caster_angle)*(self.L+np.cos(self._rear_right_caster_angle)*self.r+x),	0,	np.cos(self._rear_left_caster_angle)*(self.Dc+self.r*np.sin(self._rear_left_caster_angle)-y)-np.sin(self._rear_left_caster_angle)*(self.L+np.cos(self._rear_left_caster_angle)*self.r+x),	0,	-(self.Dd+y),	self.Dd+y,	0,	self.Dd-y,	-(self.Dd-y),	-x,	0,	-np.cos(self._front_right_caster_angle)*(self.Dc+self.r*np.sin(self._front_right_caster_angle)+y)-np.sin(self._front_right_caster_angle)*(self.L-np.cos(self._front_right_caster_angle)*self.r-x),	0,	np.cos(self._front_left_caster_angle)*(self.Dc-self.r*np.sin(self._front_left_caster_angle)-y)-np.sin(self._front_left_caster_angle)*(self.L-np.cos(self._front_left_caster_angle)*self.r-x)],
                        [0,		0,	0,		0,		self.l1,	self.h1,	-self.h1,	0,	0,	0,	0,	-(self.l2-self.r*np.cos(self._front_right_caster_angle)),	np.cos(self._front_right_caster_angle)*self.h1,		0,		0],
                        [0,		0,		0,		0,		0,	0,	0,	self.l1,	self.h1,	-self.h1,	0,	0,	0,	-(self.l2-self.r*np.cos(self._front_left_caster_angle)),	np.cos(self._front_left_caster_angle)*self.h1],
                        [-(self.Dc-self.r*np.sin(self._rear_right_caster_angle)),	np.sin(self._rear_right_caster_angle)*self.h2,		self.Dc+self.r*np.sin(self._rear_left_caster_angle),	np.sin(self._rear_left_caster_angle)*self.h2,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [-self.urc,		1,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		-self.urc,	1,	    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		-self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	-self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	-self.urc,	1,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	-self.urc,	1]])
        Y = np.array([[self.M*(self.ax*(self.R-y)/self.R - self.w**2*x) + self.M*self.g*np.sin(self.theta)], [self.M*(self.w**2*(self.R-y) + self.ax*x/self.R)], [self.M*self.g*np.cos(self.theta)],[0], [0], [self.Jz*alphaz], [0], [0],[0],[0],[0],[0],[0],[0],[0]])
        #     0;    1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14  
        keys = ['Frz1','Frf1', 'Frz2', 'Frf2', 'Fdz1', 'Fdf1', 'Fdt1', 'Fdz2', 'Fdf2', 'Fdt2', 'Fc', 'Ffz1', 'Fff1', 'Ffz2', 'Fff2']
        wheel_forces = np.linalg.solve(Mk,Y)
        wheel_forces_dict = {key:value for (key, value) in zip(keys, wheel_forces)}
        return wheel_forces_dict

    # will run brake locked model first under static friction coefficient. This will produce max decel and it's the worst case.
    # If brake locked passed, the robot will slide, change the friction coefficient to a smaller value and check the locked model again
    #   if static friction works, then dynamic friction will work too.
    # If brake locked failed, check un-locked model. This one will produce smaller decel (loose the condition)
    def modelBrake(self, x, y, h):
        if self.brakeDriveCriterion(self.modelBrake_lock(x, y, h)):
            self.dynamicU()
            X = self.modelBrake_lock(x, y, h)
            self.staticU()
            #print("modelBrake: brake is locked (slipping drivewheel)!")
        else:
            X = self.modelBrake_unlock(x, y, h)
            #print("modelBrake: brake is unlocked (slipping brake plate)!")
        return X

    # xyh here is the overall CG
    # Assuming brake torque is large enough, the robot will slide
    def modelBrake_lock(self, x,y,h):
        if self._centripetal_acceleration != 0:
            ux = 0.95
        else:
            ux = 1
        Mk = np.array( [[0,	-np.cos(self._rear_right_caster_angle),	0,	-np.cos(self._rear_left_caster_angle),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self._front_right_caster_angle),	0,	-np.cos(self._front_left_caster_angle), -self.M, 0, 0],
                        [0,		np.sin(self._rear_right_caster_angle),	0,	np.sin(self._rear_left_caster_angle),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self._front_right_caster_angle),	0, 	-np.sin(self._front_left_caster_angle), 0, -self.M, 0],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0, 0, 0, 0],
                        [-(self.Dc-self.r*np.sin(self._rear_right_caster_angle)+y),	np.sin(self._rear_right_caster_angle)*h,		self.Dc+self.r*np.sin(self._rear_left_caster_angle)-y,	np.sin(self._rear_left_caster_angle)*h,		-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	h,	-(self.Dc+self.r*np.sin(self._front_right_caster_angle)+y),	-np.sin(self._front_right_caster_angle)*h,		self.Dc-np.sin(self._front_left_caster_angle)*self.r-y,		-np.sin(self._front_left_caster_angle)*h, 0, 0, 0],
                        [self.L+self.r*np.cos(self._rear_right_caster_angle)+x,	np.cos(self._rear_right_caster_angle)*h,		self.L+self.r*np.cos(self._rear_left_caster_angle)+x,	np.cos(self._rear_left_caster_angle)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(self.L-self.r*np.cos(self._front_right_caster_angle)-x),	np.cos(self._front_right_caster_angle)*h,		-(self.L-self.r*np.cos(self._front_left_caster_angle)-x),	np.cos(self._front_left_caster_angle)*h, 0, 0, 0],
                        [0,		-np.cos(self._rear_right_caster_angle)*(self.Dc-self.r*np.sin(self._rear_right_caster_angle)+y)-np.sin(self._rear_right_caster_angle)*(self.L+np.cos(self._rear_right_caster_angle)*self.r+x),	0,	np.cos(self._rear_left_caster_angle)*(self.Dc+self.r*np.sin(self._rear_left_caster_angle)-y)-np.sin(self._rear_left_caster_angle)*(self.L+np.cos(self._rear_left_caster_angle)*self.r+x),	0,	-(self.Dd+y),	self.Dd+y,	0,	self.Dd-y,	-(self.Dd-y),	-x,	0,	-np.cos(self._front_right_caster_angle)*(self.Dc+self.r*np.sin(self._front_right_caster_angle)+y)-np.sin(self._front_right_caster_angle)*(self.L-np.cos(self._front_right_caster_angle)*self.r-x),	0,	np.cos(self._front_left_caster_angle)*(self.Dc-self.r*np.sin(self._front_left_caster_angle)-y)-np.sin(self._front_left_caster_angle)*(self.L-np.cos(self._front_left_caster_angle)*self.r-x),0,	0, -self.Jz],
                        [0,		0,	0,		0,		self.l1,	self.h1,	-self.h1,	0,	0,	0,	0,	-(self.l2-self.r*np.cos(self._front_right_caster_angle)),	np.cos(self._front_right_caster_angle)*self.h1,		0,		0, 0, 0, 0],
                        [0,		0,		0,		0,		0,	0,	0,	self.l1,	self.h1,	-self.h1,	0,	0,	0,	-(self.l2-self.r*np.cos(self._front_left_caster_angle)),	np.cos(self._front_left_caster_angle)*self.h1, 0, 0, 0],
                        [-(self.Dc-self.r*np.sin(self._rear_right_caster_angle)),	np.sin(self._rear_right_caster_angle)*self.h2,		self.Dc+self.r*np.sin(self._rear_left_caster_angle),	np.sin(self._rear_left_caster_angle)*self.h2,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [-self.urc,		1,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		-self.urc,	1,	    0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		-self.urd,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	 -self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	-self.urc,	1,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	-self.urc,	1, 0, 0, 0],
                        [0,			0,		0,		0,	 ux*self.u,	0,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,   ux*self.u,	0,	1*self.dir,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,		    0,		0,	    0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 1,-x]])
        Y = np.array([[self.M*self.g*np.sin(self.theta)], [0], [self.M*self.g*np.cos(self.theta)],[0], [0], [0], [0], [0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[self.w**2*(self.R-y)]])
        # [Frz1;Frf1;Frz2;Frf2;Fdz1;Fdf1;Fdt1;Fdz2;Fdf2;Fdt2;Fc;Ffz1;Fff1;Ffz2;Fff2;aCGx;aCGy;alphaZ];
        #     0;   1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14;  15;  16;    17  
        try:
            return np.linalg.solve(Mk,Y)
        except:
            print('Singular Matrix at: ' + str(x) + ', ' + str(y) + ', ' + str(h))
            Y[0] = -100
            return Y # return an array that will fail in criterion check
    
    # xyh here is the overall CG
    # Assuming brake torque is not large enough, the brake will slide
    def modelBrake_unlock(self, x, y, h):
        Mk = np.array( [[0,	-np.cos(self._rear_right_caster_angle),	0,	-np.cos(self._rear_left_caster_angle),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self._front_right_caster_angle),	0,	-np.cos(self._front_left_caster_angle), -self.M, 0, 0],
                        [0,		np.sin(self._rear_right_caster_angle),	0,	np.sin(self._rear_left_caster_angle),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self._front_right_caster_angle),	0, 	-np.sin(self._front_left_caster_angle), 0, -self.M, 0],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0, 0, 0, 0],
                        [-(self.Dc-self.r*np.sin(self._rear_right_caster_angle)+y),	np.sin(self._rear_right_caster_angle)*h,		self.Dc+self.r*np.sin(self._rear_left_caster_angle)-y,	np.sin(self._rear_left_caster_angle)*h,		-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	h,	-(self.Dc+self.r*np.sin(self._front_right_caster_angle)+y),	-np.sin(self._front_right_caster_angle)*h,		self.Dc-np.sin(self._front_left_caster_angle)*self.r-y,		-np.sin(self._front_left_caster_angle)*h, 0, 0, 0],
                        [self.L+self.r*np.cos(self._rear_right_caster_angle)+x,	np.cos(self._rear_right_caster_angle)*h,		self.L+self.r*np.cos(self._rear_left_caster_angle)+x,	np.cos(self._rear_left_caster_angle)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(self.L-self.r*np.cos(self._front_right_caster_angle)-x),	np.cos(self._front_right_caster_angle)*h,		-(self.L-self.r*np.cos(self._front_left_caster_angle)-x),	np.cos(self._front_left_caster_angle)*h, 0, 0, 0],
                        [0,		-np.cos(self._rear_right_caster_angle)*(self.Dc-self.r*np.sin(self._rear_right_caster_angle)+y)-np.sin(self._rear_right_caster_angle)*(self.L+np.cos(self._rear_right_caster_angle)*self.r+x),	0,	np.cos(self._rear_left_caster_angle)*(self.Dc+self.r*np.sin(self._rear_left_caster_angle)-y)-np.sin(self._rear_left_caster_angle)*(self.L+np.cos(self._rear_left_caster_angle)*self.r+x),	0,	-(self.Dd+y),	self.Dd+y,	0,	self.Dd-y,	-(self.Dd-y),	-x,	0,	-np.cos(self._front_right_caster_angle)*(self.Dc+self.r*np.sin(self._front_right_caster_angle)+y)-np.sin(self._front_right_caster_angle)*(self.L-np.cos(self._front_right_caster_angle)*self.r-x),	0,	np.cos(self._front_left_caster_angle)*(self.Dc-self.r*np.sin(self._front_left_caster_angle)-y)-np.sin(self._front_left_caster_angle)*(self.L-np.cos(self._front_left_caster_angle)*self.r-x),0,	0, -self.Jz],
                        [0,		0,	0,		0,		self.l1,	self.h1,	-self.h1,	0,	0,	0,	0,	-(self.l2-self.r*np.cos(self._front_right_caster_angle)),	np.cos(self._front_right_caster_angle)*self.h1,		0,		0, 0, 0, 0],
                        [0,		0,		0,		0,		0,	0,	0,	self.l1,	self.h1,	-self.h1,	0,	0,	0,	-(self.l2-self.r*np.cos(self._front_left_caster_angle)),	np.cos(self._front_left_caster_angle)*self.h1, 0, 0, 0],
                        [-(self.Dc-self.r*np.sin(self._rear_right_caster_angle)),	np.sin(self._rear_right_caster_angle)*self.h2,		self.Dc+self.r*np.sin(self._rear_left_caster_angle),	np.sin(self._rear_left_caster_angle)*self.h2,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [-self.urc,		1,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		-self.urc,	1,	    0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		-self.urd,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	 -self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	-self.urc,	1,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	-self.urc,	1, 0, 0, 0],
                        [0,			0,		0,		0,	    0,	0,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,   0,	0,	1*self.dir,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,		    0,		0,	    0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 1,-x]])
        Y = np.array([[self.M*self.g*np.sin(self.theta)], [0], [self.M*self.g*np.cos(self.theta)],[0], [0], [0], [0], [0],[0],[0],[0],[0],[0],[0],[0],[-self.maxBrakeF],[-self.maxBrakeF],[self.w**2*(self.R-y)]])
        # [Frz1;Frf1;Frz2;Frf2;Fdz1;Fdf1;Fdt1;Fdz2;Fdf2;Fdt2;Fc;Ffz1;Fff1;Ffz2;Fff2;aCGx;aCGy;alphaZ];
        #     0;   1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14;  15;  16;    17  
        try:
            return np.linalg.solve(Mk,Y)
        except:
            print('Singular Matrix at: ' + str(x) + ', ' + str(y) + ', ' + str(h))
            Y[0] = -100
            return Y # return an array that will fail in criterion check        
                