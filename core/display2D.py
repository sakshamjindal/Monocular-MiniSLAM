import cv2 as cv

class Displayer:
    def __init__(self, win_name):
        self.win_name = win_name
        cv.namedWindow(self.win_name, cv.WINDOW_AUTOSIZE)
    def __del__(self):
        cv.destroyWindow(self.win_name)
    def display(self, image, delay):
        cv.imshow(self.win_name, image)
        cv.waitKey(delay)

def draw_keypoints(img, kps):
    for kp in kps:
        u, v = int(round(kp.pt[0])), int(round(kp.pt[1]))
        cv.circle(img, (u,v), radius=2, color=(0, 255, 0), thickness=1)

def draw_relative_movements(img, matched_uvs):
    for match in matched_uvs:
        prev_kp = match[0]
        cur_kp = match[1]
        cv.line(img, prev_kp, cur_kp, (255, 0, 0), thickness = 1)