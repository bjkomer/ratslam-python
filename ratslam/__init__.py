# =============================================================================
# Federal University of Rio Grande do Sul (UFRGS)
# Connectionist Artificial Intelligence Laboratory (LIAC)
# Renato de Pontes Pereira - rppereira@inf.ufrgs.br
# =============================================================================
# Copyright (c) 2013 Renato de Pontes Pereira, renato.ppontes at gmail dot com
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy 
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights 
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
# copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in 
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# =============================================================================

'''
This is a full Ratslam implementation in python. This implementation is based 
on Milford's original implementation [1]_ in matlab, and Christine Lee's python 
implementation [2]_. The original data movies can also be found in [1]_.

The only dependence for this package is Numpy [3]_, thus it does not handle how
to open and manage the movie and image files. For this, I strongly recommend 
the use of OpenCV [4]_.

.. [1] https://wiki.qut.edu.au/display/cyphy/RatSLAM+MATLAB
.. [2] https://github.com/coxlab/ratslam-python
.. [3] http://www.numpy.org/
.. [4] http://opencv.org/
'''

import rospy
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Odometry
from ratslam_ros.msg import ViewTemplate, TopologicalAction
import cv2
import numpy as np
from ratslam._globals import *
from ratslam.visual_odometry import VisualOdometry
from ratslam.view_cells import ViewCells
from ratslam.pose_cells import PoseCells
from ratslam.experience_map import ExperienceMap

# TopologicalAction values
NO_ACTION=0
CREATE_NODE=1
CREATE_EDGE=2
SET_NODE=3

class Ratslam(object):
    '''Ratslam implementation.

    The ratslam is divided into 4 modules: visual odometry, view cells, pose 
    cells, and experience map. This class also store the odometry and pose 
    cells activation in order to plot them.
    '''

    def __init__(self, root='/irat_red'):
        '''Initializes the ratslam modules.'''

        self.root = root
        self.pc_output = TopologicalAction()
        self.prev_time = rospy.Time() # defaults time to 0
        self.odo_update = False
        self.vt_update = False

        #self.visual_odometry = VisualOdometry()
        self.view_cells = ViewCells()
        self.pose_cells = PoseCells()
        #self.experience_map = ExperienceMap()

        # TRACKING -------------------------------
        x, y, th = 0, 0, 0#self.visual_odometry.odometry
        self.odometry = [[x], [y], [th]]
        
        x_pc, y_pc, th_pc = self.pose_cells.active
        self.pc = [[x_pc], [y_pc], [th_pc]]
        # ----------------------------------------

        # Setting up ROS publishers and subscribers
        rospy.init_node('posecell_localview', anonymous=True)
 
        self.sub_im = rospy.Subscriber( self.root + '/camera/image/compressed',
                                        CompressedImage,
                                        self.im_callback
                                      )
        # ViewTemplate Message
        #Header header
        #
        #uint32 current_id
        #float64 relative_rad
        """
        self.sub_vt = rospy.Subscriber( self.root + '/LocalView/Template',
                                        ViewTemplate,
                                        self.vt_callback
                                      )
        """
        self.sub_odo = rospy.Subscriber( self.root + '/odom',
                                         Odometry,
                                         self.odo_callback
                                       )
        # Topological Action Messagea #NO_ACTION=0
        #uint32 CREATE_NODE=1
        #uint32 CREATE_EDGE=2
        #uint32 SET_NODE=3
        #
        #Header header
        #
        #uint32 action
        #
        #uint32 src_id
        #uint32 dest_id
        #
        #float64 relative_rad #angle between the stored template and the agent's current angle
        self.pub_ta = rospy.publisher( self.root + '/PoseCell/TopologicalAction',
                                       TopologicalAction
                                     )
    """
    def vt_callback(self, vt):

        vt.current_id
        vt.relative_rad

    """

    def odo_callback(self, odo):

        # Calculate time difference in seconds
        time_diff = (odo.header.stamp - self.prev_time).secs
        self.prev_time = odo.header.stamp

        vtrans = odo.twist.twist.linear.x
        vrot = odo.twist.twist.angular.z

        #self.experience_map(vtrans, vrot)
        self.pc_output.src_id = self.pose_cells.get_current_exp_id()
        x_pc, y_pc, th_pc = self.pose_cells(self.view_cells.prev_cell, vtrans*time_diff, vrot*time_diff)
        self.pc_output.action = self.pose_cell.get_action()
        
        if self.pc_output.action != NO_ACTION:

            self.pc_output.header.stamp = rospy.Time.now()
            self.pc_output.header.seq += 1
            self.pc_output.dest_id = self.pose_cells.get_current_exp_id()
            self.pc_output.relative_rad = self.pose_cells.get_relative_rad()

            self.pub_ta.publish(self.pc_output)

        # TRACKING -------------------------------
        self.pc[0].append(x_pc)
        self.pc[1].append(y_pc)
        self.pc[2].append(th_pc)
    
    def get_action(self):

        delta_pc = 0
        action = NO_ACTION

        if (self.odo_update & self.vt_update):
            self.odo_update = False
            self.vt_update = False
        else:
            return action # NO_ACTION

        if self.view_cells.size == 0:
            return action # NO_ACTION
        
        if self.view_cells.size == 0:
            self.create_experience()
            action = CREATE_NODE

    def im_callback(self, im):
        np_arr = np.fromstring(im.data, np.uint8)
        bgr_im = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        gray_im = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        
        x_pc, y_pc, th_pc = self.pose_cells.active
        view_cell = self.view_cells(gray_im, x_pc, y_pc, th_pc)
        

    def digest(self, img):
        '''Execute a step of ratslam algorithm for a given image.

        :param img: an gray-scale image as a 2D numpy array.
        '''

        x_pc, y_pc, th_pc = self.pose_cells.active
        view_cell = self.view_cells(img, x_pc, y_pc, th_pc)
        vtrans, vrot = self.visual_odometry(img)
        x_pc, y_pc, th_pc = self.pose_cells(view_cell, vtrans, vrot)
        self.experience_map(view_cell, vtrans, vrot, x_pc, y_pc, th_pc)

        # TRACKING -------------------------------
        x, y, th = self.visual_odometry.odometry
        self.odometry[0].append(x)
        self.odometry[1].append(y)
        self.odometry[2].append(th)
        self.pc[0].append(x_pc)
        self.pc[1].append(y_pc)
        self.pc[2].append(th_pc)
        # ----------------------------------------
