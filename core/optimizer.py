import g20
import numpy as np

class PoseGraph():
    """
    
    """
    nodes = []
    edges = []

    nodes_optimized = []
    edges_optimized = []

    def __init__(self, verbose=True):
        
        self.solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
        self.solver=  g2o.OptimizationAlgorithmLevenberg(self.solver)

        self.optimizer = g2o.SparseOptimizer()
        self.optimizer.set_verbose(verbose)
        self.optimizer.set_algorithm(self.solver)    

    def add_vertex(self, id, pose, is_fixed):

        v = g20.VertexSE3()
        v.set_id(id)
        v.set_estimate