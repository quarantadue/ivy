class ProgressCounter:
	def __init__(self,total_frames=None,frame_rate=None,starting_frame=0):
		self.fn=starting_frame
		self.fps=frame_rate
		self.total_frames=total_frames

	def frame(self):
		'''
		get frame number
		''' 
		return self.fn

	def set_frame(self,n):
		self.fn=n
		
	def set_fps(self,fps):
		self.fps=fps
		
	def set_total_frames(self,n):
		self.total_frames=n
	
	def timestamp(self):
		if self.fps is None:
			return 0
		return self.frame/self.fps
		
	def progress(self):
		if self.total_frames is None:
			return 1
		return self.fn/self.total_frames
	
	def remaining_frames(self):
		if self.total_frames is None:
			return 0
		return self.total_frames - self.fn
	
	def incframe(self):
		self.fn+=1
		
_p=ProgressCounter()

def get_ProgressCounter():
	return _p
