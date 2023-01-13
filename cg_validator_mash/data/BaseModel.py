import numpy as np
from scipy.optimize import fsolve

# Base model for all robots
class BaseModel:
    def __init__(self):
        self.name = 'LD450'
        self.k = .48 # rocker ratio, (drive wheel to pivot pin) / (drive wheel to caster)
        self.us = 0.76 # Static friction coefficiency
        self.ud = 0.63 # Dynamic coefficientcy
        self.u = self.us
        self.theta = np.deg2rad(0) # ramp angle
        self.axa = 0.8 # acceleration in m/s^2
        self.axd = -1.2 # deceleration in m/s^2
        self.ax = self.axa
        self.brakeDecel = -1.3 # brake decel should be larger than 1.3
        self.ac = 0.5 # centripetal accel
        self.v = 1.2 # robot center velocity
        self.R = self.v**2/self.ac # robot trajactory radius
        self.w = self.v / self.R # robot angular velocity
        self.urc = 0.024 # rolling resistance coefficiency for casters
        self.urd = 0.026 # rolling resistance coefficiency for drive wheel
        self.g = 9.81
        self.maxBrakeF = 99999 # Max Braking force
        self.maxDriveF = 99999 # Max motor drive force
        self.maxDriveDecelF = 99999 # Max motor force during decel
        self.maxDriveAccelF = 99999 # Max motor force furing accel

        # default values come from LD450
        self.L = 0.285 # Drive wheel to front/back caster distance
        self.r = 0.03175 # caster swivel radius
        self.l1 = self.L * self.k # drive wheel to pivot pin
        self.l2 = self.L * (1-self.k) # caster to pivot pin
        self.h2 = 0.1 # back rocker pivot height
        self.h1 = 0.05 # front rocker pivot height
        self.Mpayload = 450 # payload mass
        self.Mrobot = 140 # robot mass
        self.M = self.Mpayload+self.Mrobot # total mass
        self.Dc = 0.465/2 # width from robot center to caster swivel center 
        self.Dd = 0.60553/2 # width from robot center to drive wheel center
        self.robotH = 0.38 # robot height
        self.ground_clearance = 0.04
        self.pRobot = [0, 0, (self.robotH - self.ground_clearance) / 2 + self.ground_clearance] # robot center of mass position without payload in [x, y, z]
        self.JzRobot = 32*self.Mrobot/223 # robot moment of inertia without payload
        self.JzPayload = 43.28*self.Mpayload/600 # payload moment of inertia
        self.Jz = 85.3*self.M/823 # robot with payload moment of inertia
        self.kr = 3.75 # stiffness of rear rocker and/or caster
        self.kf = 2.1 # for MD 1.63. Stiffness of front rocker and/or caster
        self.kk = self.kr/(self.kr+self.kf)
        self.Fspring = 0 # Extra force acted on the drive wheel
    
        self.Br1 = 0 # Rear caster 1 angle during corning
        self.Bf1 = 0 # Front caster 1 angle during corning
        self.Br2 = 0 # Rear caster 2 angle during corning
        self.Bf2 = 0 # Front caster 2 angle during corning

        self.dir = 1 # direction signal

    # need to be ran before update overallCG, payloadCG and overallJz
    def updatePayloadM(self, m):
        self.Mpayload = m
        self.M = self.Mpayload+self.Mrobot
        self.JzPayload = 43.28*self.Mpayload/600
        self.Jz = 85.3*self.M/823

    def updateK(self, ki):
        self.k = ki
        self.l1 = self.L * self.k
        self.l2 = self.L * (1-self.k)
    def updateRampAngle(self, angle):
        self.theta = np.deg2rad(angle)
    def updateAx(self, axi):
        self.ax = axi
        if self.ax < self.brakeDecel:
            self.brakeDecel = self.ax
    def useAccel(self):
        self.ax = self.axa
    def useDecel(self):
        self.ax = self.axd
        if self.ax < self.brakeDecel:
            self.brakeDecel = self.ax
    def updateU(self, u):
        self.u = u
    def dynamicU(self):
        self.u = self.ud
    def staticU(self):
        self.u = self.us

    # Centripital accel ac in m/s^2
    def cornering(self, ac=0.5):
        if ac and ac != 0.0:
            self.ac = ac
            self.R = self.v**2/self.ac
            self.w = self.v / self.R
        else:
            self.ac = 0
            self.w = 0
            self.R = 9999999
        self.solveCasterAngle()
    
    # speed in m/s
    def updateSpeed(self, speed):
        self.v = speed
        self.cornering(self.ac)
        if self.v >= 0:
            self.dir = 1
        else:
            self.dir = -1

    def solveCasterAngle(self):
        if self.v > 0 and self.ac > 0:
            self.Br1 = fsolve(self.r1CasterAngle, 0)[0]
            self.Bf1 = fsolve(self.f1CasterAngle, 0)[0]
            self.Br2 = fsolve(self.r2CasterAngle, 0)[0]
            self.Bf2 = fsolve(self.f2CasterAngle, 0)[0]      
        elif self.v > 0 and self.ac < 0:
            self.Br1 = fsolve(self.r1CasterAngle, 2*np.pi)[0]
            self.Bf1 = fsolve(self.f1CasterAngle, 2*np.pi)[0]
            self.Br2 = fsolve(self.r2CasterAngle, 2*np.pi)[0]
            self.Bf2 = fsolve(self.f2CasterAngle, 2*np.pi)[0]                 
        elif self.v < 0 and self.ac > 0:
            self.Br1 = fsolve(self.r1CasterAngle, np.pi)[0]
            self.Bf1 = fsolve(self.f1CasterAngle, np.pi)[0]
            self.Br2 = fsolve(self.r2CasterAngle, np.pi)[0]
            self.Bf2 = fsolve(self.f2CasterAngle, np.pi)[0]   
        elif self.v < 0 and self.ac < 0:
            self.Br1 = fsolve(self.r1CasterAngle, np.pi)[0]
            self.Bf1 = fsolve(self.f1CasterAngle, np.pi)[0]
            self.Br2 = fsolve(self.r2CasterAngle, np.pi)[0]
            self.Bf2 = fsolve(self.f2CasterAngle, np.pi)[0]   
        elif self.v < 0 and self.ac == 0:
            self.Br1 = np.pi
            self.Bf1 = np.pi
            self.Br2 = np.pi
            self.Bf2 = np.pi
        else:
            self.Br1 = 0
            self.Bf1 = 0
            self.Br2 = 0
            self.Bf2 = 0        

    def r1CasterAngle(self, x):
        return (self.L + np.cos(x) * self.r) / (self.R + self.Dc - np.sin(x) * self.r) - np.tan(x)

    def r2CasterAngle(self, x):
        return (self.L + np.cos(x) * self.r) / (self.R - self.Dc - np.sin(x) * self.r) - np.tan(x)

    def f1CasterAngle(self, x):
        return (self.L - np.cos(x) * self.r) / (self.R + self.Dc + np.sin(x) * self.r) - np.tan(x)

    def f2CasterAngle(self, x):
        return (self.L - np.cos(x) * self.r) / (self.R - self.Dc + np.sin(x) * self.r) - np.tan(x)

    # from overall CG to payload CG
    def toPayloadCG(self, x, y, h):
        lst = []
        lst.append((x-self.pRobot[0])*self.Mrobot/(self.M-self.Mrobot) + x)
        lst.append((y-self.pRobot[1])*self.Mrobot/(self.M-self.Mrobot) + y)
        lst.append((h-self.pRobot[2])*self.Mrobot/(self.M-self.Mrobot) + h)
        return lst

    # from payload CG to overall
    def toOverallCG(self, x, y, h):
        lst = []
        lst.append(self.pRobot[0]*self.Mrobot/self.M + x*(self.M-self.Mrobot)/self.M)
        lst.append(self.pRobot[1]*self.Mrobot/self.M + y*(self.M-self.Mrobot)/self.M)
        lst.append(self.pRobot[2]*self.Mrobot/self.M + h*(self.M-self.Mrobot)/self.M)
        return lst    

    # input is payload CG location and overall CG location as list of [x, y, h]
    def toOverallJz(self, pp, po):
        self.Jz = self.Mrobot * ((po[0]-self.pRobot[0])**2 + (po[1]-self.pRobot[1])**2) + self.JzRobot + (self.M - self.Mrobot) * ((pp[0]-po[0])**2 + (pp[1]-po[1])**2) + self.JzPayload

    def normalDriveCriterion(self, X):
        # Fc1 and Fc2 are allocated based normal force on each drive wheel
        Fc1 = X[10] * X[4] / (X[4] + X[7])
        Fc2 = X[10] * X[7] / (X[4] + X[7])
        # Vector sum of traction and centripetal force
        Ftf1 = np.sqrt(Fc1**2 + X[6]**2)
        Ftf2 = np.sqrt(Fc2**2 + X[9]**2)
        if (self.v > 0 and self.ac > 0) or (self.v < 0 or self.ac < 0):
            self.maxDriveF = self.maxDriveAccelF
        else:
            self.maxDriveF = self.maxDriveDecelF
        return X[0] > 0 and X[2] > 0 and X[4] > 0 and X[7] > 0 and X[11] > 0 and X[13] > 0 and \
            Ftf1 < self.u*X[4] and Ftf2 < self.u*X[7] and abs(X[6]) <= self.maxDriveF+0.1 and abs(X[9]) <= self.maxDriveF+0.1

    def normalDriveCriterion_print(self, X):
        Fc1 = X[10] * X[4] / (X[4] + X[7])
        Fc2 = X[10] * X[7] / (X[4] + X[7])
        Ftf1 = np.sqrt(Fc1**2 + X[6]**2)
        Ftf2 = np.sqrt(Fc2**2 + X[9]**2)
        if (self.v > 0 and self.ac > 0) or (self.v < 0 or self.ac < 0):
            self.maxDriveF = self.maxDriveAccelF
        else:
            self.maxDriveF = self.maxDriveDecelF
        flag = True
        if not (X[0] > 0 and X[2] > 0 and X[4] > 0 and X[7] > 0 and X[11] > 0 and X[13] > 0):
            print('Normal Force < 0')
            flag = False
        if not Ftf1 < self.u*X[4]:
            print('Right Drive Wheel Traction Slipped')
            flag = False
        if not Ftf2 < self.u*X[7] :
            print('Left Drive Wheel Traction Slipped')
            flag = False
        if not (abs(X[6]) <= self.maxDriveF+0.1 and abs(X[9]) <= self.maxDriveF+0.1):
            print('Motor Limit')
            flag = False
        return flag

    def brakeDriveCriterion(self, X):
        return X[0] > 0 and X[2] > 0 and X[4] > 0 and X[7] > 0 and X[11] > 0 and X[13] > 0 and abs(X[15]) >= abs(self.brakeDecel) and abs(X[6]) <= self.maxBrakeF+0.1 and abs(X[9]) <= self.maxBrakeF+0.1

    def brakeDriveCriterion_print(self, X):
        flag = True
        if not (X[0] > 0 and X[2] > 0 and X[4] > 0 and X[7] > 0 and X[11] > 0 and X[13] > 0):
            print('Normal Force < 0')
            flag = False
        if not abs(X[15]) >= abs(self.brakeDecel):
            print('Brake too slow')
            flag = False
        if not (abs(X[6]) <= self.maxBrakeF+0.1 and abs(X[9]) <= self.maxBrakeF+0.1):
            print('Brake Torque Limit')
            flag = False
        return flag

    # true if brake is locked
    def brakeisLocked(self, X):
        return abs(X[6]) <= self.maxBrakeF+0.1 and abs(X[9]) <= self.maxBrakeF+0.1