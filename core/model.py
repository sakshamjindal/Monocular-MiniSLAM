import cv2
import numpy as np

from .optimizer import PoseGraph
from .geocom.features import featureTracking

from .utils import *

KMIN_NUM_FEATURE = 1500

class VisualSLAM():
   
    def __init__(self, camera_intrinsics, ground_pose):
        
        self.K = camera_intrinsics
        self.ground_pose = ground_pose
        self.feature_tracker = featureTracking
        self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        self.cur_R = None
        self.cur_t = None
        self.prev_frame = None
        self.trueX, self.trueY, self.trueZ = 0, 0, 0
        self.poses = []
        self.gt = []

        self.pose_graph = PoseGraph(verbose = True)

        
    def getAbsoluteScale(self, frame_id):
        """
        specialized for KITTI odometry dataset
        """
        
        gr_pose = self.ground_pose[frame_id-1]
        x_prev = float(gr_pose[0][-1])
        y_prev = float(gr_pose[1][-1])
        z_prev = float(gr_pose[2][-1])
        gr_pose = self.ground_pose[frame_id]
        x = float(gr_pose[0][-1])
        y = float(gr_pose[1][-1])
        z = float(gr_pose[2][-1])
        self.trueX, self.trueY, self.trueZ = x, y, z
        return np.sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev))
        

    def run_optimizer(self):

        """
        Add poses to the optimizer graph
        """
        
        self.pose_graph.add_vertex(len(self.poses), self.cur_Rt)
        self.pose_graph.add_edge((len(self.poses)-1, len(self.poses)), getTransform(self.cur_Rt, self.prev_Rt))


    def calculate_errors(self):

        """
        Calculate errors propagated
        """

        error_r, error_t = getError(self.poses[-1], self.poses[-2], self.gt[-1], self.gt[-2])
        self.errors.append((error_r, error_t))

    def __call__(self, stage, current_frame):
        
        if stage == 0:
            """ process first frame """

            self.points_ref = self.detector.detect(current_frame)
            self.points_ref = np.array([x.pt for x in self.points_ref])

        elif stage == 1:
            """ process second frame """
            
            self.points_ref, points_cur = self.feature_tracker(self.prev_frame, current_frame, self.points_ref)
            E, mask = cv2.findEssentialMat(points_cur, self.points_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, points_cur, self.points_ref, self.K)
            self.points_ref = points_cur
            
            self.prev_t = self.cur_t
            self.prev_R = self.prev_R
            self.prev_Rt = convert_to_Rt(self.prev_R, self.prev_t)
        else:
            """ process subsequent frames after first 2 frames """
            
            self.points_ref, points_cur = self.feature_tracker(self.prev_frame, current_frame, self.points_ref)
            E, mask = cv2.findEssentialMat(points_cur, self.points_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            _, R, t, mask = cv2.recoverPose(E, points_cur, self.points_ref, self.K)
            absolute_scale = self.getAbsoluteScale(stage)
            if absolute_scale > 0.1:
                self.cur_t = self.prev_t + absolute_scale*self.prev_R@t
                self.cur_R = R@self.prev_R
            if(self.points_ref.shape[0]<KMIN_NUM_FEATURE):
                points_cur = self.detector.detect(current_frame)
                points_cur = np.array([x.pt for x in points_cur], dtype=np.float32)

            self.points_ref = points_cur
        
            self.cur_Rt = convert_to_Rt(cur_R, cur_t)
            self.poses.append(self.cur_Rt)
            self.gt.append(self.ground_pose[stage-1])

            self.run_optimizer()
            self.calculate_errors()


        self.prev_t = self.cur_t
        self.prev_R = self.prev_R
        self.prev_Rt = convert_to_Rt(self.prev_R, self.prev_t)        
        self.prev_frame = current_frame