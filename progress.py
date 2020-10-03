class ProgressCounter:
	def __init__(self):
		self.fn=0
		self.fps=None
		# note: self.total frames is never used
		self.total_frames=None
		self.start=0
		self.end=None
	
	def config(self,total_frames=None,frame_rate=None,starting_frame=0,ending_frame=None):
		self.fn=starting_frame
		self.fps=frame_rate
		# note: self.total frames is never used
		self.total_frames=total_frames
		self.start=starting_frame
		self.end=ending_frame if ending_frame is not None else total_frames

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
	
	def set_start(start):
		self.start=start
	
	def set_end(end):
		self.end=end
	
	def timestamp(self):
		if self.fps is None:
			return 0
		return self.frame/self.fps
		
	def progress(self):
		if self.end is None:
			return 0.5
		return (self.fn-self.start)/(self.end-self.start)
	
	def remaining_frames(self):
		if self.total_frames is None:
			return 0
		return self.end - self.fn
		
	def processed(self):
		return self.fn-self.start
	
	def incframe(self):
		self.fn+=1
		
_p=ProgressCounter()

def get_ProgressCounter():
	return _p
