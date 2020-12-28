import numpy as np 
import cv2
import argparse

from core.model import VisualOdometry
from core.dataset import KittiDataset
from core.utils import draw_trajectory

def parse_argument():
    
    parser=argparse.ArgumentParser()
    parser.add_argument('--dataset', default='kitti')
    parser.add_argument('--path', required=True)
    return parser.parse_args()


def main():
    
    args = parse_argument()
    
    # Get data params using the dataloader 
    dataset = KittiDataset(args.path)
    camera_matrix = dataset.camera_intrinsics()
    ground_truth_poses = dataset.ground_truth
    num_frames = len(dataset)
    
    # Initialise the mono-VO model
    model = VisualOdometry(camera_matrix, ground_truth_poses)
    
    # Initialise an empty drawing board for trajectory
    blank_slate = np.zeros((600,600,3), dtype=np.uint8)
    
    # Iterate over the frames and update the rotation and translation vectors
    for index in range(num_frames):
        frame, _ = dataset[index]
        updated_t = model(index, frame)
        if index>2:
            x, y, z = updated_t[0], updated_t[1], updated_t[2]
        else:
            x, y, z = 0.,  0., 0.
            
        print(x,y,z)
        draw_x, draw_y =int(x) + 290, int(z) + 290
        true_x, true_y = int(model.trueX) + 290, int(model.trueZ) + 290
        
        draw_trajectory(blank_slate, index, x, y, z, draw_x, draw_y, true_x, true_y)
        cv2.imshow('Road facing camera', frame)
        cv2.waitKey(1)
    
if __name__ == "__main__":
    main()