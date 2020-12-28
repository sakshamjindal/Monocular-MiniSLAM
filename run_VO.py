import numpy as np 
import cv2

from core.model import VisualOdometry
from core.dataset import KittiDataset

def parse_argument():
    parser=argparse.ArgumentParser()
    parser.add_argument('--dataset', default='kitti')
    parser.add_argument('--path', required=True)

    return parser.parse_args()


def main():
    
    args = parse_argument()
    
    dataset = KittiDataset(args.path)
    camera_matrix = dataset.camera_intrinsics()
    ground_truth_poses = dataset.ground_truth
    model = VisualOdometry(camera_matrix, ground_truth_poses)
    num_frames = len(dataset)
    
    for index in range(num_frames):
        frame, _ = dataset[index]
        model(frame)
        
    
if __name__ == "__main__":
    main()