#!/usr/bin/env python

import rospy
import random

from Tkinter import *
from giskardpy.python_interface import GiskardWrapper
import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi
from control_msgs.msg import JointTrajectoryControllerState


def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class JointGoalPublisher(object):
    def init_collada(self, robot):
        """
        reads the controllable joints from a collada
        :param robot:
        :return:
        """
        robot = robot.getElementsByTagName('kinematics_model')[0].getElementsByTagName('technique_common')[0]
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                name = child.getAttribute('name')
                if name not in self.giskard_joints:
                    continue

                if child.getElementsByTagName('revolute'):
                    joint = child.getElementsByTagName('revolute')[0]
                else:
                    rospy.logwarn("Unknown joint type %s", child)
                    continue

                if joint:
                    limit = joint.getElementsByTagName('limits')[0]
                    minval = float(limit.getElementsByTagName('min')[0].childNodes[0].nodeValue)
                    maxval = float(limit.getElementsByTagName('max')[0].childNodes[0].nodeValue)
                if minval == maxval:  # this is fixed joint
                    continue

                self.joint_list.append(name)
                joint = {'min':minval*pi/180.0, 'max':maxval*pi/180.0, 'zero':0, 'position':0, 'velocity':0, 'effort':0}
                self.free_joints[name] = joint

    def init_urdf(self, robot):
        """
        reads the controllable joints from a urdf
        :param robot:
        :return:
        """
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints that are controlled by giskard
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype in ['fixed', 'floating', 'planar']:
                    continue
                name = child.getAttribute('name')
                if name not in self.giskard_joints:
                    continue

                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if self.use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval}
                #if self.pub_def_positions:
                  #  joint['position'] = zeroval
                #if self.pub_def_vels:
                 #   joint['velocity'] = 0.0
                #if self.pub_def_efforts:
                 #   joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint

    def send_goal(self, goal):
        """
        sends a joint goal to giskard
        :param goal:
        :type goal: dict
        :return:
        """
        self.giskard_wrapper.set_joint_goal(goal)
        self.giskard_wrapper.plan_and_execute()


    def __init__(self):
        description = get_param('robot_description')

        self. giskard_wrapper = GiskardWrapper()

        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        self.use_mimic = get_param('use_mimic_tags', True)
        self.use_small = get_param('use_smallest_joint_limits', True)

        self.zeros = get_param("zeros")

        #self.pub_def_positions = get_param("publish_default_positions", True)
        #self.pub_def_vels = get_param("publish_default_velocities", False)
        #self.pub_def_efforts = get_param("publish_default_efforts", False)

        msg = rospy.wait_for_message(u'/whole_body_controller/state', JointTrajectoryControllerState)
        self.giskard_joints = msg.joint_names


        robot = xml.dom.minidom.parseString(description)
        if robot.getElementsByTagName('COLLADA'):
            self.init_collada(robot)
        else:
            self.init_urdf(robot)



class JointGoalPublisherGui(Frame):

    def __init__(self, jgp, master=None):
        """
        :param jgp: The JointGoalPublisher that this gui will represent
        :type jgp: JointGoalPublisher
        :param master:
        :type master: Tk
        """
        Frame.__init__(self, master)
        self.master = master
        self.jgp = jgp
        self.joint_map = {}
        self.allow_self_collision = IntVar(value=1)

        self.master.title("Giskard Joint Goal Publisher")

        # allowing the widget to take the full space of the root window
        self.pack(fill=BOTH, expand=1)

        self.slider_frame = self.VerticalScrolledFrame(self)
        self.slider_frame.grid(row=0, stick="ns")

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.sliders = {}
        r = 0
        for name in self.jgp.joint_list:
            if name not in self.jgp.free_joints:
                continue
            joint = self.jgp.free_joints[name]

            if joint['min'] == joint['max']:
                continue

            l = Label(self.slider_frame.interior, text=name)
            l.grid(column=0, row=r)
            slider = Scale(self.slider_frame.interior, from_=joint['min'], to=joint['max'], orient=HORIZONTAL, resolution=0.01)
            slider.set(joint['zero'])
            slider.grid(column=1, row=r)
            self.sliders[name] = slider
            r += 1

        buttonFrame=Frame(self)
        sendGoalButton = Button(buttonFrame, text="send goal", command=self.send_goal)
        randomizeButton = Button(buttonFrame, text="randomize", command=self.randomize)
        resetButton = Button(buttonFrame, text="reset", command=self.reset_sliders)
        currentJsButton = Button(buttonFrame, text="current Js", command=self.current_joint_states)

        selfCollisionButton = Checkbutton(buttonFrame, text="allow self collision", variable=self.allow_self_collision, onvalue=1, offvalue=0)

        sendGoalButton.grid(row=1)
        randomizeButton.grid(row=1, column=1)
        resetButton.grid(row=1, column=2)
        currentJsButton.grid(row=1, column=3)
        selfCollisionButton.grid(row=1, column=4)

        buttonFrame.grid(row=1)

    def send_goal(self):
        """
        sends a joint goal with the joint states set in the sliders
        :return:
        """
        goal_dict = {}
        for key in self.sliders.keys():
            goal_dict[key] = self.sliders[key].get()

        if self.allow_self_collision.get():
            jgp.giskard_wrapper.disable_self_collision()
        else:
            jgp.giskard_wrapper.avoid_collision(0.1, body_b=jgp.giskard_wrapper.robot_name)
            print(jgp.giskard_wrapper.robot_name)

        self.jgp.send_goal(goal_dict)

    def randomize(self):
        """
        sets every slider to a random value
        :return:
        """
        for key in self.sliders.keys():
            val = random.uniform(self.jgp.free_joints[key]['min'], self.jgp.free_joints[key]['max'])
            self.sliders[key].set(val)

    def reset_sliders(self):
        """
        sets the value of every slider to its zerovalue
        :return:
        """
        for key in self.sliders.keys():
            self.sliders[key].set(self.jgp.free_joints[key]['zero'])

    def current_joint_states(self):
        """
        sets the value of every slider to its corresponding current joint state
        :return:
        """
        msg = rospy.wait_for_message(u'joint_states', JointState)
        for i in range(len(msg.name)):
            if msg.name[i] in self.sliders.keys():
                self.sliders[msg.name[i]].set(msg.position[i])


    class VerticalScrolledFrame(Frame):
        """A pure Tkinter scrollable frame that actually works!
        * Use the 'interior' attribute to place widgets inside the scrollable frame
        * Construct and pack/place/grid normally
        * This frame only allows vertical scrolling
        source: https://stackoverflow.com/questions/16188420/python-tkinter-scrollbar-for-frame
        """

        def __init__(self, parent, *args, **kw):
            Frame.__init__(self, parent, *args, **kw)

            # create a canvas object and a vertical scrollbar for scrolling it
            vscrollbar = Scrollbar(self, orient=VERTICAL)
            vscrollbar.pack(fill=Y, side=RIGHT, expand=FALSE)
            canvas = Canvas(self, bd=0, highlightthickness=0,
                            yscrollcommand=vscrollbar.set)
            canvas.pack(side=LEFT, fill=BOTH, expand=TRUE)
            vscrollbar.config(command=canvas.yview)

            # reset the view
            canvas.xview_moveto(0)
            canvas.yview_moveto(0)

            # create a frame inside the canvas which will be scrolled with it
            self.interior = interior = Frame(canvas)
            interior_id = canvas.create_window(0, 0, window=interior,
                                               anchor=NW)

            # track changes to the canvas and frame width and sync them,
            # also updating the scrollbar
            def _configure_interior(event):
                # update the scrollbars to match the size of the inner frame
                size = (interior.winfo_reqwidth(), interior.winfo_reqheight())
                canvas.config(scrollregion="0 0 %s %s" % size)
                if interior.winfo_reqwidth() != canvas.winfo_width():
                    # update the canvas's width to fit the inner frame
                    canvas.config(width=interior.winfo_reqwidth())

            interior.bind('<Configure>', _configure_interior)

            def _configure_canvas(event):
                if interior.winfo_reqwidth() != canvas.winfo_width():
                    # update the inner frame's width to fill the canvas
                    canvas.itemconfigure(interior_id, width=canvas.winfo_width())

            canvas.bind('<Configure>', _configure_canvas)




if __name__ == '__main__':
    try:
        rospy.init_node('joint_goal_publisher')
        jgp = JointGoalPublisher()

        root = Tk()
        root.geometry("500x600")
        gui = JointGoalPublisherGui(jgp, root)

        root.mainloop()

    except rospy.ROSInterruptException:
        pass
