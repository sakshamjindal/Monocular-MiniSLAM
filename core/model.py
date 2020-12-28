import cv2
import numpy as np

from .geocom.features import featureTracking

KMIN_NUM_FEATURE = 1500

class VisualOdometry():
   
    def __init__(self, camera_intrinsics, ground_pose,):
        
        self.K=camera_intrinsics
        self.ground_pose=ground_pose
        self.feature_tracker=featureTracking
        self.detector=cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        self.cur_R=None
        self.cur_t=None
        self.prev_frame=None
        self.trueX, self.trueY, self.trueZ = 0, 0, 0
        
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
        
    def __call__(self, stage, current_frame):
        
        if stage==0:
            #process first frame
            self.points_ref=self.detector.detect(current_frame)
            self.points_ref=np.array([x.pt for x in self.points_ref])
        elif stage==1:
            #process second frame
            self.points_ref, points_cur = self.feature_tracker(self.prev_frame, current_frame, self.points_ref)
            E, mask = cv2.findEssentialMat(points_cur, self.points_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, points_cur, self.points_ref, self.K)
            self.points_ref = points_cur
        else:
            #process subsequent frames after first 2 frames
            self.points_ref, points_cur = self.feature_tracker(self.prev_frame, current_frame, self.points_ref)
            E, mask = cv2.findEssentialMat(points_cur, self.points_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            _, R, t, mask = cv2.recoverPose(E, points_cur, self.points_ref, self.K)
            absolute_scale = self.getAbsoluteScale(stage)
            if absolute_scale>0.1:
                self.cur_t=self.cur_t + absolute_scale*self.cur_R@t
                self.cur_R=R@self.cur_R
            if(self.points_ref.shape[0]<KMIN_NUM_FEATURE):
                points_cur=self.detector.detect(current_frame)
                points_cur=np.array([x.pt for x in points_cur], dtype=np.float32)
            self.points_ref=points_cur
            
        self.prev_frame=current_frame
        
        return self.cur_t