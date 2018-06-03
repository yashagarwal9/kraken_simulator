from Tkinter import *
import rospy

from kraken_msgs.msg._thrusterData6Thruster import thrusterData6Thruster

pub = rospy.Publisher('KeyboardControl', thrusterData6Thruster, queue_size = 10)
rospy.init_node('KeyboardControl_py', anonymous = True)

thrust = thrusterData6Thruster()
offset = 0
thrust.data[0] = offset#200
thrust.data[1] = offset#-200
thrust.data[2] = offset#-150
thrust.data[3] = offset#-150
thrust.data[4] = offset#offset
thrust.data[5] = offset#540

def Gui_callback0(event=None):
        thrust.data[2] += 100
        thrust.data[3] += 100
        pub.publish(thrust)
        priLabel()
def Gui_callback1(event=None):
        thrust.data[2] -= 100
        thrust.data[3] -= 100
        pub.publish(thrust)
        priLabel()
def Gui_callback2(event=None):
        thrust.data[0] += 100
        thrust.data[1] += 100
        pub.publish(thrust)
        priLabel()
def Gui_callback3(event=None):
        thrust.data[0] -= 100
        thrust.data[1] -= 100
        pub.publish(thrust)
        priLabel()
def Gui_callback4(event=None):
        thrust.data[4] += 100
        thrust.data[5] += 100
        pub.publish(thrust)
        priLabel()
def Gui_callback5(event=None):
        thrust.data[4] -= 100
        thrust.data[5] -= 100
        pub.publish(thrust)
        priLabel()
def Gui_callback6(event=None):
        thrust.data[0] = offset#200
        thrust.data[1] = offset#-200
        thrust.data[2] = offset#-150
        thrust.data[3] = offset#-150
        thrust.data[4] = offset#offset
        thrust.data[5] = offset#540
        pub.publish(thrust)
        priLabel()
def priLabel():
    for i in range(6):
        temp = Label(frame, text = thrust.data[i]).grid(row=i+1, column = 4, sticky=W+E)

root = Tk()
root.title("KeyboardControl")
frame = Frame(root, width=250, height=100)
left = Button(frame, text = "left(a)", command = Gui_callback0).grid(row = 1, column = 0, sticky=W+E)
right = Button(frame, text = "right(d)", command = Gui_callback1).grid(row = 1, column = 2, sticky=W+E)
forward = Button(frame, text = "forward(w)", command = Gui_callback2).grid(row = 0, column = 1, sticky=W+E)
backward = Button(frame, text = "backward(s)", command = Gui_callback3).grid(row = 1, column = 1, sticky=W+E)
upward = Button(frame, text = "upward(t)", command = Gui_callback4).grid(row = 0, column = 0, sticky=W+E)
downward = Button(frame, text = "downward(g)", command = Gui_callback5).grid(row = 0, column = 2, sticky=W+E)
stop = Button(frame, text = "stop(space)", command = Gui_callback6).grid(row = 2, column = 1, sticky=W+E)
label = Label(frame, text = "Force Values").grid(row = 0, column = 3, sticky=W+E)

root.bind("a", Gui_callback0)
root.bind("d", Gui_callback1)
root.bind("w", Gui_callback2)
root.bind("s", Gui_callback3)
root.bind("t", Gui_callback4)
root.bind("g", Gui_callback5)
root.bind("<space>", Gui_callback6)

frame.pack()
root.mainloop()
