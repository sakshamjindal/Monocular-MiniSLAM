import cv2
import numpy as np

from .optimizer import PoseGraph
from .geocom.features import featureTracking

from .utils import *
import pdb

KMIN_NUM_FEATURE = 1500
optimize = False

class VisualSLAM():
   
    def __init__(self, camera_intrinsics, ground_pose, args):
        
        self.K = camera_intrinsics
        self.ground_pose = ground_pose
        self.args = args
        self.feature_tracker = featureTracking
        self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        self.cur_R = None
        self.cur_t = None
        self.prev_frame = None
        self.trueX, self.trueY, self.trueZ = 0, 0, 0
        self.poses = []
        self.gt = []
        self.errors = []
        
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
        

    def run_optimizer(self, local_window=10):

        """
        Add poses to the optimizer graph
        """
        if len(self.poses)<local_window+1:
            return

        self.pose_graph = PoseGraph(verbose = True)
        local_poses = self.poses[1:][-local_window:]

        for i in range(1,len(local_poses)):   
            self.pose_graph.add_vertex(i, local_poses[i])
            self.pose_graph.add_edge((i-1, i), getTransform(local_poses[i], local_poses[i-1]))
            self.pose_graph.optimize(self.args.num_iter)
        
        self.poses[-local_window+1:] = self.pose_graph.nodes_optimized


    def calculate_errors(self):

        """
        Calculate errors propagated
        """

        error_r , error_t = getError(self.poses[-1], self.poses[-2], self.gt[-1], self.gt[-2])
        self.errors.append((error_r, error_t))


    def __call__(self, stage, current_frame):
        
        self.gt.append(convert_to_4_by_4(self.ground_pose[stage-1]))
        self.current_frame = current_frame

        if stage == 0:
            """ process first frame """

            self.points_ref = self.detector.detect(current_frame)
            self.points_ref = np.array([x.pt for x in self.points_ref])
            self.prev_frame = current_frame 
            self.prev_Rt = np.eye(4) 
            self.poses.append(self.prev_Rt)
            return
    
        elif stage == 1:
            """ process second frame """
            
            self.points_ref, points_cur = self.feature_tracker(self.prev_frame, current_frame, self.points_ref)
            E, _ = cv2.findEssentialMat(points_cur, self.points_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            _, self.cur_R, self.cur_t, __ = cv2.recoverPose(E, points_cur, self.points_ref, self.K)
            self.points_ref = points_cur
            self.cur_Rt = convert_to_Rt(self.cur_R, self.cur_t)
            self.poses.append(convert_to_4_by_4(self.cur_Rt))
        else:
            """ process subsequent frames after first 2 frames """
            
            self.points_ref, points_cur = self.feature_tracker(self.prev_frame, current_frame, self.points_ref)
            E, mask = cv2.findEssentialMat(points_cur, self.points_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            _, R, t, mask = cv2.recoverPose(E, points_cur, self.points_ref, self.K)
            absolute_scale = self.getAbsoluteScale(stage)
            if absolute_scale > 0.1:
                self.cur_t = self.prev_t + absolute_scale*self.prev_R@t
                self.cur_R = R @ self.prev_R
            if(self.points_ref.shape[0]<KMIN_NUM_FEATURE):
                points_cur = self.detector.detect(current_frame)
                points_cur = np.array([x.pt for x in points_cur], dtype=np.float32)

            self.points_ref = points_cur
        
            self.cur_Rt = convert_to_Rt(self.cur_R, self.cur_t)
            self.poses.append(convert_to_4_by_4(self.cur_Rt))

            if self.args.optimize:
                self.run_optimizer(self.args.local_window)
                #self.calculate_errors()

        self.prev_t = self.cur_t
        self.prev_R = self.cur_R
        self.prev_Rt = convert_to_Rt(self.prev_R, self.prev_t)        
        self.prev_frame = current_frame
        