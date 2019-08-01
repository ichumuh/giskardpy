from Queue import Empty, Queue

import actionlib
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from giskard_msgs.msg._MoveGoal import MoveGoal
from giskard_msgs.msg._MoveResult import MoveResult
from py_trees import Blackboard, Status
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from giskardpy.exceptions import MAX_NWSR_REACHEDException, QPSolverException, SolverTimeoutError, InsolvableException, \
    SymengineException, PathCollisionException, UnknownBodyException, ImplementationException
import giskardpy.identifier as identifier
from giskardpy.logging import loginfo
from giskardpy.plugin import GiskardBehavior
from giskardpy.utils import plot_trajectory

ERROR_CODE_TO_NAME = {getattr(MoveResult, x): x for x in dir(MoveResult) if x.isupper()}


class ActionServerHandler(object):
    """
    Interface to action server which is more useful for behaviors.
    """

    def __init__(self, action_name, action_type):
        self.goal_queue = Queue(1)
        self.result_queue = Queue(1)
        self._as = actionlib.SimpleActionServer(action_name, action_type,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        """
        :type goal: MoveGoal
        """
        self.goal_queue.put(goal)
        self.result_queue.get()()

    def pop_goal(self):
        try:
            goal = self.goal_queue.get_nowait()
            # self.canceled = False
            return goal
        except Empty:
            return None

    def has_goal(self):
        return not self.goal_queue.empty()

    def send_feedback(self):
        # TODO
        pass

    def send_preempted(self, result=None):
        # TODO put shit in queue
        def call_me_now():
            self._as.set_preempted(result)

        self.result_queue.put(call_me_now)

    def send_result(self, result=None):
        """
        :type result: MoveResult
        """

        def call_me_now():
            self._as.set_succeeded(result)

        self.result_queue.put(call_me_now)

    def is_preempt_requested(self):
        return self._as.is_preempt_requested()


class ActionServerBehavior(GiskardBehavior):
    def __init__(self, name, as_name, action_type=None):
        self.as_handler = None
        self.as_name = as_name
        self.action_type = action_type
        super(ActionServerBehavior, self).__init__(name)

    def setup(self, timeout):
        # TODO handle timeout
        self.as_handler = Blackboard().get(self.as_name)
        if self.as_handler is None:
            self.as_handler = ActionServerHandler(self.as_name, self.action_type)
            Blackboard().set(self.as_name, self.as_handler)
        return super(ActionServerBehavior, self).setup(timeout)

    def get_as(self):
        """
        :rtype: ActionServerHandler
        """
        return self.as_handler


class GoalReceived(ActionServerBehavior):
    def update(self):
        if self.get_as().has_goal():
            rospy.sleep(.5)
            return Status.SUCCESS
        return Status.FAILURE


class GetGoal(ActionServerBehavior):
    def __init__(self, name, as_name):
        super(GetGoal, self).__init__(name, as_name)

    def pop_goal(self):
        return self.get_as().pop_goal()


class GoalCanceled(ActionServerBehavior):
    def update(self):
        if self.get_as().is_preempt_requested() or self.get_blackboard_exception() is not None:
            return Status.SUCCESS
        else:
            return Status.FAILURE


class SendResult(ActionServerBehavior):
    def __init__(self, name, as_name, action_type=None):
        super(SendResult, self).__init__(name, as_name, action_type)

    def update(self):
        # TODO get result from god map or blackboard
        e = self.get_blackboard_exception()
        Blackboard().set('exception', None)
        result = MoveResult()
        result.error_code = self.exception_to_error_code(e)
        trajectory = self.get_god_map().safe_get_data(identifier.trajectory)
        result.trajectory = self.traj_to_msg(trajectory)
        if self.get_as().is_preempt_requested() or not result.error_code == MoveResult.SUCCESS:
            self.get_as().send_preempted(result)
        else:
            self.get_as().send_result(result)
        return Status.SUCCESS

    def traj_to_msg(self, trajectory):
        """
        :type traj: giskardpy.data_types.Trajectory
        :return: JointTrajectory
        """
        self.controller_joints = rospy.wait_for_message(u'/whole_body_controller/state',
                                                        JointTrajectoryControllerState).joint_names
        sample_period = self.get_god_map().safe_get_data(identifier.sample_period)
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = rospy.get_rostime() + rospy.Duration(0.5)
        trajectory_msg.joint_names = self.controller_joints
        for time, traj_point in trajectory.items():
            p = JointTrajectoryPoint()
            p.time_from_start = rospy.Duration(time*sample_period)
            for joint_name in self.controller_joints:
                if joint_name in traj_point:
                    p.positions.append(traj_point[joint_name].position)
                    p.velocities.append(traj_point[joint_name].velocity)
                else:
                    raise NotImplementedError(u'generated traj does not contain all joints')
            trajectory_msg.points.append(p)
        return trajectory_msg

    def exception_to_error_code(self, exception):
        """
        :type exception: Exception
        :rtype: int
        """
        error_code = MoveResult.SUCCESS
        if isinstance(exception, MAX_NWSR_REACHEDException):
            error_code = MoveResult.MAX_NWSR_REACHED
        elif isinstance(exception, QPSolverException):
            error_code = MoveResult.QP_SOLVER_ERROR
        elif isinstance(exception, UnknownBodyException):
            error_code = MoveResult.UNKNOWN_OBJECT
        elif isinstance(exception, SolverTimeoutError):
            error_code = MoveResult.SOLVER_TIMEOUT
        elif isinstance(exception, InsolvableException):
            error_code = MoveResult.INSOLVABLE
        elif isinstance(exception, SymengineException):
            error_code = MoveResult.SYMENGINE_ERROR
        elif isinstance(exception, PathCollisionException):
            error_code = MoveResult.PATH_COLLISION
        elif isinstance(exception, ImplementationException):
            print(exception)
            error_code = MoveResult.INSOLVABLE
        elif exception is not None:
            error_code = MoveResult.INSOLVABLE
        return error_code
