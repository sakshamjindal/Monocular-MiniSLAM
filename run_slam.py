import numpy as np 
import cv2
import argparse

from core.model import VisualSLAM
from core.dataset import KittiDataset
from core.utils import draw_trajectory
from core.display2D import Displayer
from core.display3D import Viewer3D

def parse_argument():
    
    parser=argparse.ArgumentParser()
    parser.add_argument('--dataset', default='kitti')
    parser.add_argument('--path', required=True)
    parser.add_argument('--optimize', action='store_true', help='enable pose graph optimization')
    parser.add_argument('--local_window', default=10, type=int, help='number of frames to run the optimization')
    parser.add_argument('--num_iter', default=100, type=int, help='number of max iterations to run the optimization')
    
    return parser.parse_args()

def main():
    
    args = parse_argument()
    
    # Get data params using the dataloader 
    dataset = KittiDataset(args.path)
    camera_matrix = dataset.intrinsic
    ground_truth_poses = dataset.ground_truth
    num_frames = len(dataset)
    
    # Initialise the mono-VO model
    model = VisualSLAM(camera_matrix, ground_truth_poses, args)
    
    # Initialie the viewer object
    viewer = Viewer3D()
    error = []
    
    # Iterate over the frames and update the rotation and translation vectors
    for index in range(num_frames):

        frame, _ , _ = dataset[index]
        model(index, frame)

        if index>2:
            viewer.update(model)

        if index>2:
            x, y, z = model.cur_t[0], model.cur_t[1], model.cur_t[2]
        else:
            x, y, z = 0.,  0., 0.
            
        ## Set ofset to remove the overlap
        offset_x, offset_y = 5, 5
        draw_x, draw_y =int(x) + 290 - offset_x, int(z) + 290 - offset_y
        true_x, true_y = int(model.trueX) + 290, int(model.trueZ) + 290
        
        #draw_trajectory(blank_slate, index, x, y, z, draw_x, draw_y, true_x, true_y)
        cv2.imshow('Road facing camera', frame)
        cv2.waitKey(1)


    viewer.update(model)
    viewer.stop()

if __name__ == "__main__":
    main()