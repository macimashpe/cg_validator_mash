#%%
import numpy as np

from BaseModel import BaseModel

# Two rear side rockers and no front rocker
# the model here is still based on MD which has two side front rockers. This model removed the rear rocker
# So the speed and accel direction should be the opposite, e.g. going forward the speed will be negative
class LD450(BaseModel):
    def __init__(self, payloadM=450):
        BaseModel.__init__(self)
        self.name = 'LD450'
        # Fixed robot parameters LD450
        # self.L = 0.295 # 0.285 for ld250 with 5'' caster, 0.295 for 4inch blickle caster
        self.L = 0.285
        # self.r = 0.043 # 0.03175 for ld250 with 5'' caster, 0.043 for 4inch blickle caster
        self.r = 0.03175
        self.k = .16 / self.L
        self.l1 = self.L * self.k
        self.l2 = self.L * (1-self.k)
        self.h2 = 0.1 
        self.h1 = 0.055
        self.Mpayload = payloadM
        self.Mrobot = 150
        self.M = self.Mpayload+self.Mrobot
        # self.Dc = 0.241 # 0.465/2 for ld250 with 5'' caster, 0.241 for 4inch blickle caster
        self.Dc = 0.465/2
        self.Dd = 0.605/2
        self.robotH = 0.38
        self.pRobot = [0, 0, (self.robotH - 0.04) / 2 + 0.04]
        self.JzRobot = 32*self.Mrobot/223
        self.JzPayload = 43.28*self.Mpayload/600
        self.Jz = 85.3*self.M/823
        self.kr = 3.75 # rear caster stiffness
        self.kf = 2.1 # front rocker stiffness, for MD 1.63
        self.kk = self.kr/(self.kr+self.kf)
        # self.Fspring = -100 * 5.46 # 100 psi * factor = N, positive means pull the drive wheel up; negative means pushing it down

        self.maxDriveAccelF = 1.4 * 40 / (0.2032 / 2) # single drive wheel, Kt is 1.4Nm/A, default 30A
        self.maxDriveDecelF = self.maxDriveAccelF * 3 # because of the gear box, decel force is larger than the accel force under same motor current

        self.solveCasterAngle()

    # change the direction
    def updateAx(self, axi):
        self.ax = -axi
        if self.ax < self.brakeDecel:
            self.brakeDecel = self.ax
        
    # speed in mm/s, change the direction
    def updateSpeed(self, speed):
        self.v = -speed/1000
        self.cornering(self.ac)
        if self.v >= 0:
            self.dir = 1
        else:
            self.dir = -1       
    
    # Centripital accel ac in m/s^2, change the direction
    def cornering(self, ac = -0.5):
        if ac:
            self.ac = -ac
            self.R = self.v**2/self.ac
            self.w = self.v / self.R
        else:
            self.ac = 0
            self.w = 0
            self.R = 9999999
        self.solveCasterAngle()

    # xyh here is the overall CG
    def modelNoBrake(self, x,y,h):
        if self.ac != 0:
            alphaz = self.ax/self.R
        else:
            alphaz = 0
        Mk = np.array( [[0,	-np.cos(self.Br1),	0,	-np.cos(self.Br2),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self.Bf1),	0,	-np.cos(self.Bf2)],
                        [0,		np.sin(self.Br1),	0,	np.sin(self.Br2),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self.Bf1),	0, 	-np.sin(self.Bf2)],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0],
                        [-(self.Dc-self.r*np.sin(self.Br1)+y),	np.sin(self.Br1)*h,		self.Dc+self.r*np.sin(self.Br2)-y,	np.sin(self.Br2)*h,		-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	h,	-(self.Dc+self.r*np.sin(self.Bf1)+y),	-np.sin(self.Bf1)*h,		self.Dc-np.sin(self.Bf2)*self.r-y,		-np.sin(self.Bf2)*h],
                        [self.L+self.r*np.cos(self.Br1)+x,	np.cos(self.Br1)*h,		self.L+self.r*np.cos(self.Br2)+x,	np.cos(self.Br2)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(self.L-self.r*np.cos(self.Bf1)-x),	np.cos(self.Bf1)*h,		-(self.L-self.r*np.cos(self.Bf2)-x),	np.cos(self.Bf2)*h],
                        [0,		-np.cos(self.Br1)*(self.Dc-self.r*np.sin(self.Br1)+y)-np.sin(self.Br1)*(self.L+np.cos(self.Br1)*self.r+x),	0,	np.cos(self.Br2)*(self.Dc+self.r*np.sin(self.Br2)-y)-np.sin(self.Br2)*(self.L+np.cos(self.Br2)*self.r+x),	0,	-(self.Dd+y),	self.Dd+y,	0,	self.Dd-y,	-(self.Dd-y),	-x,	0,	-np.cos(self.Bf1)*(self.Dc+self.r*np.sin(self.Bf1)+y)-np.sin(self.Bf1)*(self.L-np.cos(self.Bf1)*self.r-x),	0,	np.cos(self.Bf2)*(self.Dc-self.r*np.sin(self.Bf2)-y)-np.sin(self.Bf2)*(self.L-np.cos(self.Bf2)*self.r-x)],
                        [0,		0,	0,		0,		self.l1,	self.h1,	-self.h1,	0,	0,	0,	0,	-(self.l2-self.r*np.cos(self.Bf1)),	np.cos(self.Bf1)*self.h1,		0,		0],
                        [0,		0,		0,		0,		0,	0,	0,	self.l1,	self.h1,	-self.h1,	0,	0,	0,	-(self.l2-self.r*np.cos(self.Bf2)),	np.cos(self.Bf2)*self.h1],
                        [-(self.Dc-self.r*np.sin(self.Br1)+y),	self.kk*np.sin(self.Br1)*h,	self.Dc+self.r*np.sin(self.Br2)-y, self.kk*np.sin(self.Br2)*h,	0,	0,	0,	0,	0,	0,	self.kk*h,	0,	-self.kk*np.sin(self.Bf1)*h,	0,	-self.kk*np.sin(self.Bf2)*h],
                        [-self.urc,		1,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		-self.urc,	1,	    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		-self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	-self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	-self.urc,	1,	0,	0],
                        [0,			0,		0,		0,		0,	0,	0,	0,	0,	0,	0,	0,	0,	-self.urc,	1]])
        Y = np.array([[self.M*(self.ax*(self.R-y)/self.R - self.w**2*x) + self.M*self.g*np.sin(self.theta)], [self.M*(self.w**2*(self.R-y) + self.ax*x/self.R)], [self.M*self.g*np.cos(self.theta)],[0], [0], [self.Jz*alphaz], [-self.Fspring*self.l1], [-self.Fspring*self.l1],[0],[0],[0],[0],[0],[0],[0]])
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
        Mk = np.array( [[0,	-np.cos(self.Br1),	0,	-np.cos(self.Br2),	0,	-1,	1,	0,	-1,	1,	0,	0,	-np.cos(self.Bf1),	0,	-np.cos(self.Bf2), -self.M, 0, 0],
                        [0,		np.sin(self.Br1),	0,	np.sin(self.Br2),	0,	0,	0,	0,	0,	0,	1,	0,	-np.sin(self.Bf1),	0, 	-np.sin(self.Bf2), 0, -self.M, 0],
                        [1,		0,	   1,	0,	  1,	0,	0,	1,	0,	0,	0,	1,	0,	  1,		0, 0, 0, 0],
                        [-(self.Dc-self.r*np.sin(self.Br1)+y),	np.sin(self.Br1)*h,		self.Dc+self.r*np.sin(self.Br2)-y,	np.sin(self.Br2)*h,		-(self.Dd+y),	0,	0,	self.Dd-y,	0,	0,	h,	-(self.Dc+self.r*np.sin(self.Bf1)+y),	-np.sin(self.Bf1)*h,		self.Dc-np.sin(self.Bf2)*self.r-y,		-np.sin(self.Bf2)*h, 0, 0, 0],
                        [self.L+self.r*np.cos(self.Br1)+x,	np.cos(self.Br1)*h,		self.L+self.r*np.cos(self.Br2)+x,	np.cos(self.Br2)*h,		x,	h,	-h,	x,	h,	-h,	0,	-(self.L-self.r*np.cos(self.Bf1)-x),	np.cos(self.Bf1)*h,		-(self.L-self.r*np.cos(self.Bf2)-x),	np.cos(self.Bf2)*h, 0, 0, 0],
                        [0,		-np.cos(self.Br1)*(self.Dc-self.r*np.sin(self.Br1)+y)-np.sin(self.Br1)*(self.L+np.cos(self.Br1)*self.r+x),	0,	np.cos(self.Br2)*(self.Dc+self.r*np.sin(self.Br2)-y)-np.sin(self.Br2)*(self.L+np.cos(self.Br2)*self.r+x),	0,	-(self.Dd+y),	self.Dd+y,	0,	self.Dd-y,	-(self.Dd-y),	-x,	0,	-np.cos(self.Bf1)*(self.Dc+self.r*np.sin(self.Bf1)+y)-np.sin(self.Bf1)*(self.L-np.cos(self.Bf1)*self.r-x),	0,	np.cos(self.Bf2)*(self.Dc-self.r*np.sin(self.Bf2)-y)-np.sin(self.Bf2)*(self.L-np.cos(self.Bf2)*self.r-x),0,	0, -self.Jz],
                        [0,		0,	0,		0,		self.l1,	self.h1,	-self.h1,	0,	0,	0,	0,	-(self.l2-self.r*np.cos(self.Bf1)),	np.cos(self.Bf1)*self.h1,		0,		0, 0, 0, 0],
                        [0,		0,		0,		0,		0,	0,	0,	self.l1,	self.h1,	-self.h1,	0,	0,	0,	-(self.l2-self.r*np.cos(self.Bf2)),	np.cos(self.Bf2)*self.h1, 0, 0, 0],
                        [-(self.Dc-self.r*np.sin(self.Br1)+y),	self.kk*np.sin(self.Br1)*h,	self.Dc+self.r*np.sin(self.Br2)-y, self.kk*np.sin(self.Br2)*h,	0,	0,	0,	0,	0,	0,	self.kk*h,	0,	-self.kk*np.sin(self.Bf1)*h,	0,	-self.kk*np.sin(self.Bf2)*h, 0, 0, 0],
                        [-self.urc,		1,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		-self.urc,	1,	    0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		-self.urd,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	 -self.urd,	1*self.dir,	0,	0,	0,	0,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	-self.urc,	1,	0,	0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	-self.urc,	1, 0, 0, 0],
                        [0,			0,		0,		0,	 ux*self.u,	0,	1*self.dir,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,			0,		0,		0,		0,	0,	0,   ux*self.u,	0,	1*self.dir,	0,	0,	0,	0,		0, 0, 0, 0],
                        [0,		    0,		0,	    0,		0,	0,	0,	    0,	0,	0,	0,	0,	0,	0,		0, 0, 1,-x]])
        Y = np.array([[self.M*self.g*np.sin(self.theta)], [0], [self.M*self.g*np.cos(self.theta)],[0], [0], [0], [-self.Fspring*self.l1], [-self.Fspring*self.l1],[0],[0],[0],[0],[0],[0],[0],[0],[0],[self.w**2*(self.R-y)]])
        # [Frz1;Frf1;Frz2;Frf2;Fdz1;Fdf1;Fdt1;Fdz2;Fdf2;Fdt2;Fc;Ffz1;Fff1;Ffz2;Fff2;aCGx;aCGy;alphaZ];
        #     0;   1;  2;   3;   4;   5;   6;   7;   8;   9;10;  11;  12;  13;  14;  15;  16;    17  
        self.staticU()
        try:
            return np.linalg.solve(Mk,Y)
        except:
            print('Singular Matrix at: ' + str(x) + ', ' + str(y) + ', ' + str(h))
            Y[0] = -100
            return Y # return an array that will fail in criterion check