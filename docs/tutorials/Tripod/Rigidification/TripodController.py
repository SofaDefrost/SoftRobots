#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
from math import sin,cos


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

        self.node = node
        self.actuatorNode = self.node.getChild('actuators')
        PI=3.14159265359;
        self.Angle0=PI/2;
        self.Angle1=PI/2;
        self.Angle2=PI/2;
        
        


    def onKeyPressed(self,c):
        
        PI=3.14159265359;
        Delta_angle=PI/30;
        Radius=25;
        CenterRot=25;

        
        mo= self.actuatorNode.getObject('MechanicalObject')
        rest_pos = mo.findData('rest_position').value;
        
        
        
        ###Move
        if (ord(c) == 18):  #  <--
            
            self.Angle0 = self.Angle0 - Delta_angle
            
            rest_pos[0][1]= -CenterRot - Radius*sin(self.Angle0)
            rest_pos[0][2]= Radius*cos(self.Angle0)
            rest_pos[0][3] = sin(self.Angle0/2)
            rest_pos[0][6] = cos(self.Angle0/2)
 
        elif (ord(c) == 20): # -->
            self.Angle0 = self.Angle0 + Delta_angle
            rest_pos[0][1]= -CenterRot - Radius*sin(self.Angle0)
            rest_pos[0][2]= Radius*cos(self.Angle0)            
            rest_pos[0][3] = sin(self.Angle0/2)
            rest_pos[0][6] = cos(self.Angle0/2)
            
            
        elif (ord(c) == 19): #  up
            self.Angle1 = self.Angle1 - Delta_angle
            
            rest_pos[1][0]= (-CenterRot - Radius*sin(self.Angle1))*sin(2*PI/3)
            rest_pos[1][1]= (-CenterRot - Radius*sin(self.Angle1))*cos(2*PI/3)
            rest_pos[1][2]= Radius*cos(self.Angle1)
            
            rest_pos[1][3] = -sin(self.Angle1/2)*0.5
            rest_pos[1][4] = -sin(self.Angle1/2)*0.8660254
            rest_pos[1][6] = cos(self.Angle1/2)
        
        
        elif (ord(c) == 21): # down 
            self.Angle1 = self.Angle1 + Delta_angle

            rest_pos[1][0]= (-CenterRot - Radius*sin(self.Angle1))*sin(2*PI/3)
            rest_pos[1][1]= (-CenterRot - Radius*sin(self.Angle1))*cos(2*PI/3)
            rest_pos[1][2]= Radius*cos(self.Angle1)
            
            rest_pos[1][3] = -sin(self.Angle1/2)*0.5
            rest_pos[1][4] = -sin(self.Angle1/2)*0.8660254
            rest_pos[1][6] = cos(self.Angle1/2)
            


        elif (c == "+"):
            self.Angle2 = self.Angle2 + Delta_angle
            rest_pos[2][0]= (-CenterRot - Radius*sin(self.Angle2))*sin(4*PI/3)
            rest_pos[2][1]= (-CenterRot - Radius*sin(self.Angle2))*cos(4*PI/3)
            rest_pos[2][2]= Radius*cos(self.Angle2)
            
            rest_pos[2][3] = -sin(self.Angle2/2)*0.5
            rest_pos[2][4] = sin(self.Angle2/2)*0.8660254
            rest_pos[2][6] = cos(self.Angle2/2)
            

        elif (c == "-"):
            self.Angle2 = self.Angle2 - Delta_angle
            rest_pos[2][0]= (-CenterRot - Radius*sin(self.Angle2))*sin(4*PI/3)
            rest_pos[2][1]= (-CenterRot - Radius*sin(self.Angle2))*cos(4*PI/3)
            rest_pos[2][2]= Radius*cos(self.Angle2)
            
            rest_pos[2][3] = -sin(self.Angle2/2)*0.5
            rest_pos[2][4] = sin(self.Angle2/2)*0.8660254
            rest_pos[2][6] = cos(self.Angle2/2)
            
        rest_pos_table= transformDoubleTableInSimpleTable(rest_pos)
        mo.findData('rest_position').value = transformTableInString(rest_pos_table)
        
