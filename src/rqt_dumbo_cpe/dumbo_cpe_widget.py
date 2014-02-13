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


import os
import time

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, QTime
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
        ui_file = os.path.join(rp.get_path('dumbo_cpe_gui'), 'resource', 'dumbo_cpe.ui')
        loadUi(ui_file, self, {'DumboContactPointEstimationGraphicsView': DumboContactPointEstimationGraphicsView})

        self.setObjectName('DumboContactPointEstimationWidget')
        


    def shutdown_all(self):
        print "Shutting down Dumbo CPE dashboard ...."
