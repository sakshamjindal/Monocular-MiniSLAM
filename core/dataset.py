# Added from Stereo Visual Odometry repository
# Originally present at : https://github.com/sakshamjindal/Stereo-Visual-Odometry-SFM/blob/master/stereoVO/datasets/KITTI_Dataset.py

import cv2
import os
import numpy as np
from pathlib import Path

__all__ = ['CameraParameters', 'KittiDataset']


class CameraParameters():
    def __init__(self, fx, fy, cx, cy):

        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    @property
    def camera_matrix(self):

        matrix = np.array([[self.fx, 0.0, self.cx],
                           [0.0, self.fx, self.cy],
                           [0.0, 0.0, 1.0]])
        return matrix

    def __call__(self):

        return self.camera_matrix


class KittiDataset():

    """
    Dataloader for KIITT dataset : (http://www.cvlibs.net/datasets/kitti/)
    KITTI dataset has been recorded from a moving platform while driving in and around Karlsruhe,
    Germany (Figure 2). It includes camera images, laser scans, high-precision GPS measurements 
    and IMU accelerations from a combined GPS/IMU system. The main purpose of this dataset is to 
    push forward the development of computer vision and robotic algorithms for autonomous driving
    """

    def __init__(self, path):

        """
        :param path (str):path to KIITI sequence (ending with sequence number)
        """

        self.path = Path(path)

        self.left_images_path = str(self.path.joinpath('image_0'))
        self.right_images_path = str(self.path.joinpath('image_1'))
        self.sequence_count = str(self.path.name)
        self.calibfile = str(self.path.joinpath('calib.txt'))
        self.gt_path = str(self.path.parents[3].joinpath('data_odometry_poses', 'dataset', 'poses', self.sequence_count + '.txt'))

        self.ground_truth = self.load_ground_truth_pose(self.gt_path)
        self.left_image_paths = self.load_image_paths(self.left_images_path)
        self.right_image_paths = self.load_image_paths(self.right_images_path)

        self.camera_intrinsic, self.PL, self.PR = self.load_camera_parameters(self.calibfile)
        self.intrinsic = self.camera_intrinsic()

        assert len(self.ground_truth) == len(self.left_image_paths)

    def convert_text_to_ground_truth(self, gt_line):

        """
        Converts a text line to projection matrix
        :param gt_line (str)      : flatten pose as a string
        :return matrix (np.array) : sizez(3,4) camera pose
        """

        matrix = np.array(gt_line.split()).reshape((3, 4)).astype(np.float32)

        return matrix

    def load_ground_truth_pose(self, gt_path):

        """
        Reads path to ground truth and aggregates to list of ground path
        :param gt_path (str)                 : path to ground truth file with each text line containing true pose of camera 
        :return ground_truth (list(np.array)): aggregated ground truth for each frame in sequence
        """

        ground_truth = None
        if not os.path.exists(gt_path):
            print("ground truth path is not found.")
            return None

        ground_truth = []

        with open(gt_path) as gt_file:
            gt_lines = gt_file.readlines()

            for gt_line in gt_lines:
                pose = self.convert_text_to_ground_truth(gt_line)
                ground_truth.append(pose)

        return ground_truth


    def load_image_paths(self, image_dir):

        """
        Returns images in path sorted by the frame number
        :path image_dir(str)     : path to image dir
        :return img_paths (list) : image paths sorted by frame number
        """
        img_paths = [os.path.join(image_dir, img_id) for img_id in os.listdir(image_dir) if os.path.isfile(os.path.join(image_dir, img_id))]
        img_paths.sort()

        return img_paths

    def load_camera_parameters(self, calibfile):

        """
        Loads left projection matrix and right projection matrix (both in the left image coordinate frame)
        to project 3D coordinates (left coordinate frame) to left frame and right frame.
        :param calibfile(str) : path to text file containing calibration parameter
        Returns:
            :param (np.array): size(3,3) intrisic camera matrix
            :projL (np.array): size (3x4) left projection matrix such that x_L = PL * X_w
            :projR (np.array): size (3x4) right projection matrix such that x_R = PR * X_w
                               (where X_w is  in the frame of the left camera)
        """

        if not os.path.exists(calibfile):
            print("camera parameter file path is not found.")
            return None

        calibParams = open(calibfile, 'r').readlines()

        P1Vals = calibParams[0].split()
        P2Vals = calibParams[1].split()

        projL = np.array([[float(P1Vals[row*4 + column + 1]) for column in range(4)] for row in range(3)])
        projR = np.array([[float(P2Vals[row*4 + column + 1]) for column in range(4)] for row in range(3)])

        param = CameraParameters(float(P1Vals[1]), float(P1Vals[6]),
                                 float(P1Vals[3]), float(P1Vals[7]))

        return param, projL, projR

    def __len__(self):

        """
        Fetch number of images returned by the dataloader
        """
        
        return len(self.left_image_paths)

    def __getitem__(self, index):

        """
        Fetches left frame, right frame and ground pose for a particular frame number (or time instant)
        :param index(int): frame number (index of stereo state)
        Returns
            :img_left  (np.array): size(H,W) left frame of a stereo configuration for a particular frame number  
            :img_right (np.array): size(H,W) right frame of a stereo configuration for a particular frame number
            :true_pose (np.array): size(3,4) true pose of left camera of stereo state relative to initial state
        """

        img_left = cv2.imread(self.left_image_paths[index])
        img_right = cv2.imread(self.right_image_paths[index])

        img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        true_pose = self.ground_truth[index]

        return img_left, img_right, true_pose