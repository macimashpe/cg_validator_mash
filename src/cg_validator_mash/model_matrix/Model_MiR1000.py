import numpy as np

from BaseModel import BaseModel

class MiR1000(BaseModel):
    def __init__(self, payloadM=900):
        BaseModel.__init__(self)
        self.name = 'MiR'
        self.L = 0.54
        self.r = 0.043
        self.k = 0.61
        self.l1 = self.L * self.k
        self.l2 = self.L * (1-self.k)
        self.h2 = 0.127
        self.h1 = 0.127
        self.Mpayload = payloadM
        self.Mrobot = 230
        self.M = self.Mpayload+self.Mrobot
        self.Dc = 0.64/2
        self.Dd = 0.665/2
        self.robotH = 0.32
        self.pRobot = [0, 0, (self.robotH - 0.04) / 2 + 0.04]
        self.JzRobot = 39.32*self.Mrobot/223
        self.JzPayload = 43.28*self.Mpayload/600
        self.Jz = 85.3*self.M/823
        # self.kr = 3.75
        # self.kf = 1.63 # for MD 1.63
        # self.kk = self.kr/(self.kr+self.kf)

        self.solveCasterAngle()

    # xyh here is the overall CG
    def modelNoBrake(self,x,y,h):
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
                        [-(self.Dc-self.r*np.sin(self.Br1)),	np.sin(self.Br1)*self.h2,		self.Dc+self.r*np.sin(self.Br2),	np.sin(self.Br2)*self.h2,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0],
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
    # Assuming brake torque is large enough, the robot will slide
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
                        [-(self.Dc-self.r*np.sin(self.Br1)),	np.sin(self.Br1)*self.h2,		self.Dc+self.r*np.sin(self.Br2),	np.sin(self.Br2)*self.h2,		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0],
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
        self.staticU()
        try:
            return np.linalg.solve(Mk,Y)
        except:
            print('Singular Matrix at: ' + str(x) + ', ' + str(y) + ', ' + str(h))
            Y[0] = -100
            return Y # return an array that will fail in criterion check
                