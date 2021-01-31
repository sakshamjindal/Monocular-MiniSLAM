import pangolin
import cv2
import numpy as np
import OpenGL.GL as gl
from multiprocessing import Process, Queue, Event

vt_done = Event()

class Viewer3D(object):

  w_i, h_i = (600, 200)

  def __init__(self):

    self.state = None
    self.state_gt = None
    self.state_optimized = None
    self.q_poses = Queue()
    self.q_poses_optimized = Queue()
    self.q_gt = Queue()
    self.q_img = Queue()
    self.q_errors = Queue()
    
    self.poses = []
    self.gt = []
    self.poses.append(np.eye(4))
    self.gt.append(np.eye(4))
    
    self.vt = Process(target=self.viewer_thread, args=(self.q_poses,self.q_gt,self.q_img,self.q_errors, self.q_poses_optimized))
    self.vt.daemon = True
    self.vt.start()
    
  def viewer_thread(self, q_poses, q_gt, q_img, q_errors, q_poses_optimized):
    self.viewer_init()

    while not pangolin.ShouldQuit():#True:
      #print('refresh')
      self.viewer_refresh(q_poses,q_gt, q_img, q_errors, q_poses_optimized)
    print("you hit quit")
    vt_done.set() 
    #return None
    #self.stop()

  def viewer_init(self):
    w, h = (1024,768)
    f = 2000 #420

    pangolin.CreateWindowAndBind("Visual Odometry Trajectory Viewer", w, h)
    gl.glEnable(gl.GL_DEPTH_TEST) #prevents point overlapping issue, check out fake-stereo's issues for more info

    # Projection and ModelView Matrices
    self.scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(w, h, f, f, w //2, h//2, 0.1, 100000),
        pangolin.ModelViewLookAt(0, -50.0, -10.0,
                              0.0, 0.0, 0.0,
                              0.0, -1.0, 0.0))#pangolin.AxisDirection.AxisY))
    self.handler = pangolin.Handler3D(self.scam)

    # Interactive View in Window
    self.dcam = pangolin.CreateDisplay()
    self.dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -w/h)
    self.dcam.SetHandler(self.handler)
    self.dcam.Activate()


    #Image viewport
    
    self.dimg = pangolin.Display('image')
    self.dimg.SetBounds(0, self.h_i/h, 1-self.w_i/w, 1.0, -w/h)
    self.dimg.SetLock(pangolin.Lock.LockLeft, pangolin.Lock.LockTop)

    self.texture = pangolin.GlTexture(self.w_i, self.h_i, gl.GL_RGB, False, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
    self.img = np.ones((self.h_i, self.w_i, 3),'uint8')*255
    
    # Translation error graph
    self.log = pangolin.DataLog()
    self.labels = ['error_t', 'error_r']#, "error_euclidean"]
    self.log.SetLabels(self.labels)

    self.plotter = pangolin.Plotter(self.log,0.0, 1500, -1500, 2500,10, 0.5)
    self.plotter.SetBounds(0.0, self.h_i/h, 0.0, 1-self.w_i/w, -w/h)

    self.plotter.Track("$i", "")

    pangolin.DisplayBase().AddDisplay(self.plotter)
    self.errorlog_r, self.errorlog_t = [], []

  def viewer_refresh(self, q_poses, q_gt, q_img, q_errors, q_poses_optimized):
    while not q_poses.empty():
      self.state = q_poses.get()
    if True:
      while not q_poses_optimized.empty():
        self.state_optimized = q_poses_optimized.get()
    if not q_img.empty():
      self.img = q_img.get()
      self.img = self.img[::-1, :]
      self.img = cv2.resize(self.img, (self.w_i, self.h_i))

    while not q_gt.empty():
      self.state_gt = q_gt.get()

    while not q_errors.empty():
      error_r, error_t = q_errors.get()[-1]
      #self.log.Log(errors[0], errors[1], errors[2])

      self.errorlog_t.append(error_t)
      self.errorlog_r.append(error_r)
      #print(np.shape(self.errorlog))
      self.log.Log(np.sum(self.errorlog_t), np.sum(self.errorlog_r))

    # Clear and Activate Screen (we got a real nice shade of gray
    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
    gl.glClearColor(0.15, 0.15, 0.15, 0.0)
    #gl.glClearColor(0.0,0.0, 0.0, 0.0)
    self.dcam.Activate(self.scam)

    # Render
    if self.state is not None:
      gl.glLineWidth(1)
      # Render current pose
      if self.state_gt[0].shape[0] >= 1:
        gl.glColor3f(1.0, 1.0, 1.0)
        pangolin.DrawCameras(self.state_gt[1:])
      # Render previous keyframes
      if self.state[0].shape[0] >= 2:
        gl.glColor3f(1.0, 0.0, 1.0)
        pangolin.DrawCameras(self.state[:-1])

      # Render current pose
      if self.state[0].shape[0] >= 1:
        gl.glColor3f(0.2, 1.0, 0.2)
        pangolin.DrawCameras(self.state[-1:])
    if True:
      if self.state_optimized is not None:
        if self.state_optimized[0].shape[0] >=1:
          gl.glColor3f(0.5, 1.0, 1.0)
          pangolin.DrawCameras(self.state_optimized)


    #print(self.img.shape)
    #cv2.imshow("test", self.img)
    
    self.texture.Upload(self.img, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)

    self.dimg.Activate()
    gl.glColor3f(1.0, 1.0, 1.0)

    self.texture.RenderToViewport()

    pangolin.FinishFrame()

  def update(self, vo):
    '''
    Add new data to queue
    '''

    if self.q_img is None or self.q_poses is None:
      return

    self.q_gt.put(np.array(vo.gt))
    self.q_poses.put(vo.poses)

    if len(vo.errors) > 0:
      self.q_errors.put(vo.errors)
    # if len(vo.pose_graph.nodes_optimized) > 1:
    #   self.q_poses_optimized.put(vo.pose_graph.nodes_optimized)
    # else:
    #   self.q_img.put(vo.annotate_frames())

  def stop(self):
    self.vt.terminate()
    self.vt.join()
    #qtype = type(Queue())
    for x in self.__dict__.values():
      if isinstance(x, type(Queue())):
        while not x.empty():
          _ = x.get()
    print("viewer stopped")