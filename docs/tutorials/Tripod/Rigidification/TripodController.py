#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
from math import sin,cos


def rotateQuat(q1,q2):
    c0 = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]
    c1 = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1]
    c2 = q1[3]*q2[1] + q1[1]*q2[3] + q1[2]*q2[0] - q1[0]*q2[2]
    c3 = q1[3]*q2[2] + q1[2]*q2[3] + q1[0]*q2[1] - q1[1]*q2[0]

    q1 = [c1,c2,c3,c0]

    return q1

            
            

#Rotate a vector using a quaternion v'=qvq(-1)
def rotate(v,q):
    c0 = ((1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))*v[0] + (2.0 * (q[0] * q[1] - q[2] * q[3])) * v[1] + (2.0 * (q[2] * q[0] + q[1] * q[3])) * v[2])
    c1 = ((2.0 * (q[0] * q[1] + q[2] * q[3]))*v[0] + (1.0 - 2.0 * (q[2] * q[2] + q[0] * q[0]))*v[1] + (2.0 * (q[1] * q[2] - q[0] * q[3]))*v[2])
    c2 = ((2.0 * (q[2] * q[0] - q[1] * q[3]))*v[0] + (2.0 * (q[1] * q[2] + q[0] * q[3]))*v[1] + (1.0 - 2.0 * (q[1] * q[1] + q[0] * q[0]))*v[2])

    v[0] = c0
    v[1] = c1
    v[2] = c2

    return v



def inverseQuat(q):
    return [ -q[0], -q[1], -q[2], q[3] ]


def inverseTransform(a_H_b):
    b_H_a=[ ]
    q = [ a_H_b[3], a_H_b[4], a_H_b[5], a_H_b[6] ] ;
    b_H_a= b_H_a +  rotate([ -a_H_b[0], -a_H_b[1],-a_H_b[2] ], q)
    b_H_a= b_H_a +  inverseQuat( q ) ;
    
    return b_H_a
 
    
def composeTransform(a_H_b, b_H_c):
    a_H_c = [ ]
    b_c_in_a = rotate([ b_H_c[0], b_H_c[1], b_H_c[2]],  inverseQuat([a_H_b[3], a_H_b[4], a_H_b[5],a_H_b[6]])  )
    a_H_c = a_H_c + [ a_H_b[0] + b_c_in_a[0],  a_H_b[1] + b_c_in_a[1], a_H_b[2] + b_c_in_a[2] ]     
    a_H_c = a_H_c + rotateQuat([a_H_b[3], a_H_b[4], a_H_b[5],a_H_b[6]] , [b_H_c[3], b_H_c[4], b_H_c[5],b_H_c[6]] )  
    
    return a_H_c
    





def transformTableInString(Table):
	sizeT =  len(Table);
	strOut= ' ';
	for p in range(sizeT):
		strOut = strOut+ str(Table[p])+' '

	return strOut


def transformDoubleTableInSimpleTable(Table):
    size0 =  len(Table);

    # count the size
    size=0;
    for i in range(size0):
        size = size+len(Table[i]);

    
    TableOut=[0]*size;
    s=0;
    for i in range(size0):
        for j in range(len(Table[i])):
            TableOut[s] = Table[i][j];
            s=s+1;

        
    return TableOut


class controller(Sofa.PythonScriptController):

    def initGraph(self, node):


        self.actuatorNode = node
        PI=3.14159265359;
        self.Angle0=0;
        self.Angle1=0;
        self.Angle2=0;
        
        
        y0 = -30
        x0=0;
        z0=0
        y1=cos(2*PI/3)*y0;
        x1=sin(2*PI/3)*y0;
        y2=cos(4*PI/3)*y0;
        x2=sin(4*PI/3)*y0;
        
        self.InitTransform_Ground_H_0 = [x0,y0,z0,0,0,0,1]
        self.InitTransform_Ground_H_1  = [x1,y1,z0,0,0,sin(4*PI/6),cos(4*PI/6)]
        self.InitTransform_Ground_H_2  = [x2,y2,z0,0,0,sin(2*PI/6),cos(2*PI/6)]
        
        


    def onKeyPressed(self,c):
        
        

        
        PI=3.14159265359;
        Delta_angle=PI/30;
        
        mo= self.actuatorNode.getObject('RigidFrames')
        rest_pos = mo.findData('rest_position').value;
        
        
        
        ###Move
        if (ord(c) == 18):  #  <--
            
            self.Angle0 = self.Angle0 - Delta_angle
            Transform_0_H_new0 = [0,0,0,sin(self.Angle0/2), 0,0,cos(self.Angle0/2)]
            Transform_Ground_H_new0 = composeTransform(self.InitTransform_Ground_H_0 , Transform_0_H_new0)
            rest_pos[0] = Transform_Ground_H_new0 ;
 
        elif (ord(c) == 20): # -->
            self.Angle0 = self.Angle0 + Delta_angle
            Transform_0_H_new0 = [0,0,0,sin(self.Angle0/2), 0,0,cos(self.Angle0/2)]
            Transform_Ground_H_new0 = composeTransform(self.InitTransform_Ground_H_0 , Transform_0_H_new0)
            rest_pos[0] = Transform_Ground_H_new0 ;
            
            
        elif (ord(c) == 19): #  up
            self.Angle1 = self.Angle1 - Delta_angle
            Transform_1_H_new1 = [0,0,0,sin(self.Angle1/2), 0,0,cos(self.Angle1/2)]
            Transform_Ground_H_new1 = composeTransform(self.InitTransform_Ground_H_1 , Transform_1_H_new1)
            rest_pos[1] = Transform_Ground_H_new1 ;
        
        
        elif (ord(c) == 21): # down 
            self.Angle1 = self.Angle1 + Delta_angle
            Transform_1_H_new1 = [0,0,0,sin(self.Angle1/2), 0,0,cos(self.Angle1/2)]
            Transform_Ground_H_new1 = composeTransform(self.InitTransform_Ground_H_1 , Transform_1_H_new1)
            rest_pos[1] = Transform_Ground_H_new1 ;
            


        elif (c == "+"):
            self.Angle2 = self.Angle2 + Delta_angle
            Transform_2_H_new2 = [0,0,0,sin(self.Angle2/2), 0,0,cos(self.Angle2/2)]
            Transform_Ground_H_new2 = composeTransform(self.InitTransform_Ground_H_2 , Transform_2_H_new2)
            rest_pos[2] = Transform_Ground_H_new2 ;
            

        elif (c == "-"):
            self.Angle2 = self.Angle2 - Delta_angle
            Transform_2_H_new2 = [0,0,0,sin(self.Angle2/2), 0,0,cos(self.Angle2/2)]
            Transform_Ground_H_new2 = composeTransform(self.InitTransform_Ground_H_2 , Transform_2_H_new2)
            rest_pos[2] = Transform_Ground_H_new2 ;
            
            
            
            
        rest_pos_table= transformDoubleTableInSimpleTable(rest_pos)
        mo.findData('rest_position').value = transformTableInString(rest_pos_table)
        
