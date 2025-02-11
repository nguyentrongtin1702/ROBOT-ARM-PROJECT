from os import error
import numpy as np
from math import *
import sympy as sp
import matplotlib.pyplot as plt
from matplotlib import style

c = lambda x: np.cos(x)
s = lambda x: np.sin(x)
t = lambda x,y : atan2(x,y)



def Forward_Kinematic(t1,t2,t3):
    L2 = 140
    L3 = 150
    t1 = radians(t1)
    t2 = radians(t2)
    t3 = radians(t3)
    Px = L3*cos(t1)*cos(t2+t3)+L2*cos(t1)*cos(t2)
    Py = L3*sin(t1)*cos(t2+t3)+L2*sin(t1)*cos(t2)
    Pz = L3*sin(t2+t3) + L2*sin(t2)
    Px = round(Px,0)
    Py = round(Py,0)
    Pz = round(Pz,0)
    return Px,Py,Pz

def Inverse_Kinematic(Px,Py,Pz):
    L2 = 140
    L3 = 150
    theta12 = atan2(0,1) - atan2((-Py/(sqrt(Px*Px+Py*Py))),(Px/sqrt(Px*Px+Py*Py)))
    
    a = -2*Pz*L2
    b = -2*L2*(Px*cos(theta12)+Py*sin(theta12))
    d = L3*L3 - (Px*cos(theta12)+Py*sin(theta12))*(Px*cos(theta12)+Py*sin(theta12))-Pz*Pz-L2*L2
    
    theta22 = atan2((d/(sqrt(a*a+b*b))),-sqrt((a*a+b*b-d*d)/(a*a+b*b))) - atan2((b/sqrt(a*a+b*b)),(a/sqrt(a*a+b*b)))
    
    s23 = (Pz-L2*sin(theta22))/L3
    c23 = sqrt(1-s23*s23)
    
    theta31 = atan2(s23,c23) - theta22
    
    theta1 = degrees(theta12)
    theta2 = degrees(theta22)
    theta3 = degrees(theta31)
    theta1 = round(theta1,0)
    theta2 = round(theta2,0)
    theta3 = round(theta3,0)
    return theta1,theta2,theta3


# FK = Forward_Kinematic(120,12,-12,185,180,81)
# print(FK)
# IK = Inverse_Kinematic(-115.98, 200.884, 283.035,185,180,81,0)
 #print("hello")