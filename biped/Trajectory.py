import pinocchio as se3
import numpy as np
from utils import *

class Trajectory: 
	def __init__(self, start_p, stop_p, start_t, duration):  
		self.start_p = start_p
		self.stop_p = stop_p
		self.start_t = start_t
		self.duration = duration

		self._call = None

		if isinstance(self.start_p, se3.SE3) and isinstance(self.stop_p, se3.SE3): 
			self._call = lambda t: se3.SE3(np.eye(3), 
				np.array([(self.stop_p.translation[0] - self.start_p.translation[0]) / self.duration * (t - self.start_t), 
						  (self.stop_p.translation[1] - self.start_p.translation[1]) / self.duration * (t - self.start_t),
						  (self.stop_p.translation[2] - self.start_p.translation[2]) / self.duration * (t - self.start_t)
						  ]))

		elif isinstance(self.start_p, np.ndarray) and isinstance(self.stop_p, np.ndarray):
			pass

	def __call__(self, t): 
		assert(t >= self.start_t and t <= self.start_t + self.duration)

		return self._call(t)
