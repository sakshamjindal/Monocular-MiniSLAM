import os
import glob
import numpy as np

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
    
    def __init__(self, path):
        
        self.images_path=os.path.join(path, 'image_0')
        self.calibfile=os.path.join(path, 'calib.txt')
        self.sequence_count=os.path.dirname(path).split('/')[-1]
        self.gt_path=os.path.join(path, 'data_odometry_poses', 'dataset', 'poses', self.sequence_count + '.txt')
        
        self.ground_truth=self.load_ground_truth_pose(self.gt_path)
        self.image_paths=self.load_image_paths(self.images_path)
        self.camera_intrinsics=self.load_camera_parameters(self.calibfile)
        
        assert len(self.ground_truth)==len(self.image_paths)
        
    def convert_text_to_ground_truth(self, gt_line):
        
        matrix = np.array(gt_line.split()).reshape((3, 4)).astype(np.float32)
        return matrix
    
    def load_ground_truth_pose(self, gt_path):
        
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
        
        img_paths = [os.path.join(image_dir,img_id) for img_id in os.listdir(image_dir) if os.path.isfile(os.path.join(image_dir, img_id))]
        img_paths.sort()
        return img_paths
        
    def load_camera_parameters(self, calibfile):
        
        if not os.path.exists(calibfile):
            print("camera parameter file path is not found.")
            return None

        with open(calibfile, 'r') as f:
            line = f.readline()
            part = line.split()
            param = CameraParameters(float(part[1]), float(part[6]),
                                     float(part[3]), float(part[7]))

            return param
        
    def __len__(self):
        
        return len(self.image_paths)
        
    def __getitem__(self, index):
        
        img = cv2.imread(self.image_paths[index])
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        gr_truth = self.ground_truth[index]
        
        return img, gr_truth
 