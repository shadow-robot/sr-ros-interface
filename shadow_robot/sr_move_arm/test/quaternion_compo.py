#!/usr/bin/env python

class Quat:
    def __init__(self):
        self.a = 0
        self.b = 0
        self.c = 0
        self.d = 0

def mult(q1,q2):
    result = Quat()
    result.a = (q1.a*q2.a -q1.b*q2.b -q1.c*q2.c -q1.d*q2.d)
    result.b = (q1.a*q2.b +q1.b*q2.a +q1.c*q2.d -q1.d*q2.c)
    result.c =  (q1.a*q2.c -q1.b*q2.d +q1.c*q2.a +q1.d*q2.b)
    result.d = (q1.a*q2.d +q1.b*q2.c -q1.c*q2.b +q1.d*q2.a)
    return result

def display(q):
    print "Quaternion: "
    print q.a
    print q.b
    print q.c
    print q.d
    print "---"

q1 = Quat()
q1.a = 0.406122
q1.b=-0.363322
q1.c = 0.0986163
q1.d= -0.832669

q2 = Quat()
q2.a = 0.48296291314453427
q2.b = -0.2241438680420133
q2.c = 0.8365163037378078
q2.d = 0.1294095225512604


q3 = mult(q1,q2)

display(q3)
