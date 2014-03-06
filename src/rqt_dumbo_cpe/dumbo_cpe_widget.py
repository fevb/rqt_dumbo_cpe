#
#   dumbo_cpe_widget.py
# 
#   Created on: Feb 13, 2014
#   Authors:   Francisco Vina
#             fevb <at> kth.se
# 

#  Copyright (c) 2014, Francisco Vina, CVAP, KTH
#    All rights reserved.

#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#       * Redistributions of source code must retain the above copyright
#         notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#         notice, this list of conditions and the following disclaimer in the
#         documentation and/or other materials provided with the distribution.
#       * Neither the name of KTH nor the
#         names of its contributors may be used to endorse or promote products
#         derived from this software without specific prior written permission.

#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#    DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
#    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import sys
import os
import copy

import rospy
import rospkg
import rosparam
import moveit_commander
import geometry_msgs.msg
from std_srvs.srv import *
from cob_srvs.srv import *
from rqt_bag.recorder import Recorder

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, QTime, qWarning
from python_qt_binding.QtGui import QGraphicsView, QIcon, QWidget, QFont, QTableWidget, QTableWidgetItem, QMainWindow, QTimeEdit, QRadioButton, QPushButton



class DumboContactPointEstimationGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(DumboContactPointEstimationGraphicsView, self).__init__()


class DumboContactPointEstimationWidget(QMainWindow):
    """
    Widget for use with DumboContactPointEstimation class
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(DumboContactPointEstimationWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_dumbo_cpe'), 'resource', 'dumbo_cpe_widget.ui')
        loadUi(ui_file, self, {'DumboContactPointEstimationGraphicsView': DumboContactPointEstimationGraphicsView})

        self.setObjectName('DumboContactPointEstimationWidget')

        self.startButton.clicked[bool].connect(self._handle_startButton_clicked)
        self.stopButton.clicked[bool].connect(self._handle_stopButton_clicked)
        self.resetButton.clicked[bool].connect(self._handle_resetButton_clicked)

        moveit_commander.roscpp_initialize(sys.argv)
        self._group = moveit_commander.MoveGroupCommander("left_arm")

        # create directory for logging data

        self._datalog_path = os.path.expanduser('~/.ros/cpe_log_data/')
        rospy.loginfo('Saving CPE log data to ' + self._datalog_path)


        if not os.path.exists(self._datalog_path):
            os.makedirs(self._datalog_path)


        # robot joint position 1
        self._q1 = [-0.777780234814, -0.988386511803, 0.855541706085, -1.43784618378, 0.495330154896, -1.16698789597, -0.815340101719]


        # service names for contact point estimator and surface tracing controller
        self._cpe_param_ns = "/contact_point_estimation"
        self._stc_param_ns = "/left_arm_surface_tracing_controller"

        self._cpe_start_srv_name = "/contact_point_estimation/start"
        self._cpe_stop_srv_name = "/contact_point_estimation/stop"

        self._stc_start_srv_name = "/left_arm_surface_tracing_controller/start"
        self._stc_stop_srv_name = "/left_arm_surface_tracing_controller/stop"
        self._arm_recover_srv_name = "/left_arm_controller/recover"



    def _handle_startButton_clicked(self):


        # move the arm up
        waypoints = list()
        waypoints.append(self._group.get_current_pose().pose)

        dz = 0.05

        wpose = geometry_msgs.msg.Pose()
        wpose.orientation = waypoints[0].orientation
        wpose.position.x = waypoints[0].position.x
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z+dz

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self._group.compute_cartesian_path(waypoints,
                                                              0.005,
                                                              0.0)

        self._group.execute(plan)

        rospy.sleep(1.0)

        # move robot to joint position 1
        self._group.set_joint_value_target(self._q1)
        self._group.go()

        rospy.sleep(1.0)

        # move robot down to position 2
        waypoints = list()
        waypoints.append(self._group.get_current_pose().pose)

        dz = 0.082

        wpose = geometry_msgs.msg.Pose()
        wpose.orientation = waypoints[0].orientation
        wpose.position.x = waypoints[0].position.x
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z-dz

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self._group.compute_cartesian_path(waypoints,
                                                              0.005,
                                                              0.0)

        self._group.execute(plan)

        rospy.sleep(1.0)



        # reload the parameters for controllers and estimators
        rp = rospkg.RosPack()
        cpe_param_file_path = os.path.join(rp.get_path('contact_point_estimation'), 'config', 'contact_point_estimator.yaml')
        sne_param_file_path = os.path.join(rp.get_path('contact_point_estimation'), 'config', 'surface_normal_estimator.yaml')
        stc_param_file_path = os.path.join(rp.get_path('dumbo_contact_point_estimation'), 'config', 'dumbo_surface_tracing_controller.yaml')

        params = rosparam.load_file(cpe_param_file_path)
        rosparam.upload_params(self._cpe_param_ns, params[0][0])

        params = rosparam.load_file(sne_param_file_path)
        rosparam.upload_params(self._cpe_param_ns, params[0][0])

        params = rosparam.load_file(stc_param_file_path)
        rosparam.upload_params(self._cpe_param_ns, params[0][0])


        # save controller and estimator parameters
        # check largest suffix of already existing bag and yaml files in directory
        # new bag and config file will increment suffix by 1 to avoid
        # overwriting
        file_suffixes = list()
        suffix = int(0)

        if len(os.listdir(self._datalog_path)) != 0:
            for file in os.listdir(self._datalog_path):
                s = file.strip('.bag').strip('.yaml').strip('run').strip('_cpe').strip('_stc')
                file_suffixes.append(s)

            file_suffixes = map(int, file_suffixes)
            suffix = max(file_suffixes) + 1


        filename = 'run' + str(suffix)
        bag_filename = self._datalog_path + filename + '.bag'
        cpe_yaml_filename = self._datalog_path + filename + '_cpe' + '.yaml'
        stc_yaml_filename = self._datalog_path + filename + '_stc' + '.yaml'

        rospy.loginfo('Recording to %s', bag_filename)
        rospy.loginfo('Saving contact point estimation params to %s', cpe_yaml_filename)
        rospy.loginfo('Saving surface tracing controller params to %s', stc_yaml_filename)


        # start recording bag file
        try:
            self._recorder = Recorder(bag_filename)

        except Exception, ex:
            qWarning('Error opening bag for recording [%s]: %s' % (filename, str(ex)))
            return

        self._recorder.start()
        self._recording = True

        # save parameters
        os.system('rosparam dump -v ' + cpe_yaml_filename + ' ' + self._cpe_param_ns)
        os.system('rosparam dump -v ' + stc_yaml_filename + ' ' + self._stc_param_ns)


        # start surface tracing controller
        rospy.loginfo('Waiting for ' + self._stc_start_srv_name + ' service')
        rospy.wait_for_service(self._stc_start_srv_name)
        try:
            start_stc_srv = rospy.ServiceProxy(self._stc_start_srv_name, Empty)
            start_stc_srv()

        except rospy.ServiceException, e:
            rospy.logerr('Error starting left arm surface tracing controller')
            return

        # start contact point estimator
        rospy.loginfo('Waiting for ' + self._cpe_start_srv_name + ' service')
        rospy.wait_for_service(self._cpe_start_srv_name)
        try:
            start_cpe_srv = rospy.ServiceProxy(self._cpe_start_srv_name, Empty)
            start_cpe_srv()

        except rospy.ServiceException, e:
            rospy.logerr('Error starting contact point estimation')
            return



    def _handle_stopButton_clicked(self):

        # stop recording bag file
        self._recorder.stop()
        self._recording = False

        # stop surface tracing controller
        rospy.loginfo('Waiting for ' + self._stc_stop_srv_name + ' service')
        rospy.wait_for_service(self._stc_stop_srv_name)
        try:
            stop_stc_srv = rospy.ServiceProxy(self._stc_stop_srv_name, Empty)
            stop_stc_srv()

        except rospy.ServiceException, e:
            rospy.logerr('Error stopping left arm surface tracing controller')
            return

        # stop contact point estimator
        rospy.loginfo('Waiting for ' + self._cpe_stop_srv_name + ' service')
        rospy.wait_for_service(self._cpe_stop_srv_name)
        try:
            stop_cpe_srv = rospy.ServiceProxy(self._cpe_stop_srv_name, Empty)
            stop_cpe_srv()

        except rospy.ServiceException, e:
            rospy.logerr('Error stopping contact point estimation')
            return

    def _handle_resetButton_clicked(self):

        # reset the robot (recover)
        rospy.loginfo('Waiting for robot recover service')
        rospy.wait_for_service(self._arm_recover_srv_name)
        try:
            arm_recover_srv = rospy.ServiceProxy(self._arm_recover_srv_name, Trigger)
            ret = arm_recover_srv()

        except rospy.ServiceException, e:
            rospy.logerr('Error recovering arm through ' + self._arm_recover_srv_name + ' service.')
            return

        if not ret.success:
            rospy.logerr('Recovering arm through ' + self._arm_recover_srv_name + ' service failed.')
            return


        # move the arm up
        waypoints = list()
        waypoints.append(self._group.get_current_pose().pose)

        dz = 0.1

        wpose = geometry_msgs.msg.Pose()
        wpose.orientation = waypoints[0].orientation
        wpose.position.x = waypoints[0].position.x
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z+dz

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self._group.compute_cartesian_path(waypoints,
                                                              0.005,
                                                              0.0)

        self._group.execute(plan)


    def shutdown_all(self):
        print "Shutting down Dumbo CPE dashboard ...."



    def __del__(self):

        if self._recording:
            self._recorder.stop()
            self._recording = False
