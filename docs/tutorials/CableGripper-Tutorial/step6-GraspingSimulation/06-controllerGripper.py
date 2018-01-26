#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import Sofa


def moveRestPos(rest_pos, dx, dy, dz):
    str_out = ' '
    for i in xrange(0,len(rest_pos)) :
        str_out= str_out + ' ' + str(rest_pos[i][0]+dx)
        str_out= str_out + ' ' + str(rest_pos[i][1]+dy)
        str_out= str_out + ' ' + str(rest_pos[i][2]+dz)
    return str_out


class controller(Sofa.PythonScriptController):





    def initGraph(self, node):

            self.node = node
            self.finger1Node=self.node.getChild('finger1')
            self.finger2Node=self.node.getChild('finger2')
            self.finger3Node=self.node.getChild('finger3')

            self.cable1Node=self.finger1Node.getChild('actuator1')
            self.cable2Node=self.finger2Node.getChild('actuator2')
            self.cable3Node=self.finger3Node.getChild('actuator3')





    def onKeyPressed(self,c):

            self.dt = self.node.findData('dt').value
            incr = self.dt*100.0;

            if (c == "+"):
                disp1 = self.cable1Node.getObject('cc1').findData('value').value[0][0] + incr
                self.cable1Node.getObject('cc1').findData('value').value = str(disp1)
                disp2 = self.cable2Node.getObject('cc2').findData('value').value[0][0] + incr
                self.cable2Node.getObject('cc2').findData('value').value = str(disp2)
                disp3 = self.cable3Node.getObject('cc3').findData('value').value[0][0] + incr
                self.cable3Node.getObject('cc3').findData('value').value = str(disp3)

            if (c == "-"):
                disp1 = self.cable1Node.getObject('cc1').findData('value').value[0][0] - incr
                if disp1>-0.000001 :
                    self.cable1Node.getObject('cc1').findData('value').value = str(disp1)
                disp2 = self.cable2Node.getObject('cc2').findData('value').value[0][0] - incr
                if disp2>-0.000001 :
                    self.cable2Node.getObject('cc2').findData('value').value = str(disp2)
                disp3 = self.cable3Node.getObject('cc3').findData('value').value[0][0] - incr
                if disp3>-0.000001 :
                    self.cable3Node.getObject('cc3').findData('value').value = str(disp3)

            self.MecaObject1=self.finger1Node.getObject('tetras');
            self.MecaObject2=self.finger2Node.getObject('tetras');
            self.MecaObject3=self.finger3Node.getObject('tetras');


            # UP key :
            if ord(c)==19:
                test = moveRestPos(self.MecaObject1.rest_position, 10.0, 0.0, 0.0)
                self.MecaObject1.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject2.rest_position, 10.0, 0.0, 0.0)
                self.MecaObject2.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject3.rest_position, 10.0, 0.0, 0.0)
                self.MecaObject3.findData('rest_position').value = test

                disp1 = moveRestPos(self.cable1Node.getObject('cc1').findData('pullPoint').value, 10.0, 0.0, 0.0)
                self.cable1Node.getObject('cc1').findData('pullPoint').value = str(disp1)
                disp2 = moveRestPos(self.cable2Node.getObject('cc2').findData('pullPoint').value, 10.0, 0.0, 0.0)
                self.cable2Node.getObject('cc2').findData('pullPoint').value = str(disp2)
                disp3 = moveRestPos(self.cable3Node.getObject('cc3').findData('pullPoint').value, 10.0, 0.0, 0.0)
                self.cable3Node.getObject('cc3').findData('pullPoint').value = str(disp3)


            # DOWN key : rear
            if ord(c)==21:
                test = moveRestPos(self.MecaObject1.rest_position, -10.0, 0.0, 0.0)
                self.MecaObject1.findData('rest_position').value = str(test)
                test = moveRestPos(self.MecaObject2.rest_position, -10.0, 0.0, 0.0)
                self.MecaObject2.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject3.rest_position, -10.0, 0.0, 0.0)
                self.MecaObject3.findData('rest_position').value = test

                disp1 = moveRestPos(self.cable1Node.getObject('cc1').findData('pullPoint').value, -10.0, 0.0, 0.0)
                self.cable1Node.getObject('cc1').findData('pullPoint').value = str(disp1)
                disp2 = moveRestPos(self.cable2Node.getObject('cc2').findData('pullPoint').value, -10.0, 0.0, 0.0)
                self.cable2Node.getObject('cc2').findData('pullPoint').value = str(disp2)
                disp3 = moveRestPos(self.cable3Node.getObject('cc3').findData('pullPoint').value, -10.0, 0.0, 0.0)
                self.cable3Node.getObject('cc3').findData('pullPoint').value = str(disp3)

            # LEFT key : left
            if ord(c)==20:
                test = moveRestPos(self.MecaObject1.rest_position, 0.0, 10.0, 0.0)
                self.MecaObject1.findData('rest_position').value = str(test)
                test = moveRestPos(self.MecaObject2.rest_position, 0.0, 10.0, 0.0)
                self.MecaObject2.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject3.rest_position, 0.0, 10.0, 0.0)
                self.MecaObject3.findData('rest_position').value = test

                disp1 = moveRestPos(self.cable1Node.getObject('cc1').findData('pullPoint').value, 0.0, 10.0, 0.0)
                self.cable1Node.getObject('cc1').findData('pullPoint').value = str(disp1)
                disp2 = moveRestPos(self.cable2Node.getObject('cc2').findData('pullPoint').value, 0.0, 10.0, 0.0)
                self.cable2Node.getObject('cc2').findData('pullPoint').value = str(disp2)
                disp3 = moveRestPos(self.cable3Node.getObject('cc3').findData('pullPoint').value, 0.0, 10.0, 0.0)
                self.cable3Node.getObject('cc3').findData('pullPoint').value = str(disp3)


            # RIGHT key : right
            if ord(c)==18:
                test = moveRestPos(self.MecaObject1.rest_position, 0.0, -10.0, 0.0)
                self.MecaObject1.findData('rest_position').value = str(test)
                test = moveRestPos(self.MecaObject2.rest_position, 0.0, -10.0, 0.0)
                self.MecaObject2.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject3.rest_position, 0.0, -10.0, 0.0)
                self.MecaObject3.findData('rest_position').value = test


                disp1 = moveRestPos(self.cable1Node.getObject('cc1').findData('pullPoint').value, 0.0, -10.0, 0.0)
                self.cable1Node.getObject('cc1').findData('pullPoint').value = str(disp1)
                disp2 = moveRestPos(self.cable2Node.getObject('cc2').findData('pullPoint').value, 0.0, -10.0, 0.0)
                self.cable2Node.getObject('cc2').findData('pullPoint').value = str(disp2)
                disp3 = moveRestPos(self.cable3Node.getObject('cc3').findData('pullPoint').value, 0.0, -10.0, 0.0)
                self.cable3Node.getObject('cc3').findData('pullPoint').value = str(disp3)
