import cv2
import numpy as np

def draw_trajectory(traj, frame_id, x, y, z, draw_x, draw_y, true_x, true_y):
    
    cv2.circle(traj, (draw_x,draw_y), 1, (frame_id*255/4540,255-frame_id*255/4540,0), 1)
    cv2.circle(traj, (true_x,true_y), 1, (0,0,255), 2)
    cv2.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
    text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
    cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)
    cv2.imshow('Trajectory', traj)
    