# pylint: disable=missing-module-docstring,invalid-name

import time
from util.logger import get_logger
from util.bounding_box import get_centroid
from progress import get_ProgressCounter
import numpy as np

logger = get_logger()
progress=get_ProgressCounter()

# TODO: use numpy arrays for lines and points, this will make the code a lot more cleaner (partially done)

# I can count in two modes:
# 'touch' mode, look if the bound box cross the counting line
# 'cross' mode, calculate movement vector using current and previous frame, look if this vector cross the counting line
# 	this is useful for small or fast objects that can jump over the counting lines
#	it also reduces the chances of duplicated counts in some situations
# mode is derevid from the value of 'lookfor'  

# line['lookfor'] can be:
# in 'touch' mode specify what lines of bounding box are used to count the crossing. 
# 	It can be one of 'left','right','top','bottom','box'(all lines). Default is 'box' (old behavior)
# when in 'cross', specify what point of the bounding boxes are used to create lines (between old and new bb)
# that are then use to test the crossing
#	it can be 'tl' (top left), 'tr' (top right), 'bl' (bottom left), 'br' (bottom right), 'cc' (centroid, default) , 'corners' (any corner)

# line['direction'] can be:
# None (default, old behavior) : do not look for direction
#	if direction is None and I'm using touch mode I have no need for a previous bounding box  
# 'left' : count only object that are moving leftward (left refers to an observer that is on the first point of the line and faces the second)
# 'right' : count only objects that are moving rightward

# line['dir_measurement'] can be
# 'previous' direction is measured using difference from current and previous frame
# 'first' (default) direction is measured using difference from current position and where the blob was first detected

touch_mode=0
cross_mode=1

def test_lines(lines):
	'''
	Test counting lines parameters. So I can avoid some try - except later
	'''
	for line in lines:
		direction=line.get('direction',None)
		if direction not in ['left','right',None]:
			return f"Bad value for direction parameter in counting lines: {direction}"
		lookfor=line.get('lookfor','box')	
		if lookfor not in ['left','right','top','bottom','box','tl','tr','bl','br','cc','corners']:
			return f"Invalid value for counting line 'lookfor' parameter: {lookfor}"
		dir_measurement=line.get('dir_measurement','first')
		if dir_measurement not in ['first', 'previous']:
			return f"Invalid value for counting line 'dir_measurement' parameter: {dir_measurement}"
		return None		


def _line_segments_intersect(line1, line2):
	'''
	See: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
	'''
	def get_orientation(p, q, r):
		# this is (r-q) X (q-p)
		# val should be positive for p q r in counterclokwise orientation (assuming x goes right and y goes down)
		val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
		if val == 0:
			return 0
		return 1 if val > 0 else -1

	def is_on_segment(p, q, r):
		if q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and \
			q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]):
			return True
		return False
	
	p1 = line1[0]
	q1 = line1[1]
	p2 = line2[0]
	q2 = line2[1]

	o1 = get_orientation(p1, q1, p2)
	o2 = get_orientation(p1, q1, q2)
	o3 = get_orientation(p2, q2, p1)
	o4 = get_orientation(p2, q2, q1)

	if o1 != o2 and o3 != o4:
		return True ,o3
		
	# do I really want to ckeck for the extremely rare cases where all points are colinear?

	if o1 == 0 and is_on_segment(p1, p2, q1):
		return True ,o3

	if o2 == 0 and is_on_segment(p1, q2, q1):
		return True ,o3

	if o3 == 0 and is_on_segment(p2, p1, q2):
		return True ,o3

	if o4 == 0 and is_on_segment(p2, q1, q2):
		return True ,o3

	return False,0

#def _has_crossed_counting_line(bbox, line):
#	'''
#	Check if at least one edge of a bounding box is intersected by a counting line.
#	'''
#	x, y, w, h = bbox
#	bbox_line1 = [(x, y), (x + w, y)]
#	bbox_line2 = [(x + w, y), (x + w, y + h)]
#	bbox_line3 = [(x, y), (x, y + h)]
#	bbox_line4 = [(x, y + h), (x + w, y + h)]
#
#	if _line_segments_intersect(bbox_line1, line) or \
#			_line_segments_intersect(bbox_line2, line) or \
#			_line_segments_intersect(bbox_line3, line) or \
#			_line_segments_intersect(bbox_line4, line):
#		return True
#	return False

def _has_crossed_counting_line(counting_line, bloblines,blob, mode):
	'''
	Check if at least one object line cosses the countin line
	'''
	def lines_crossprod(l1,l2):
		v1=l1[1]-l1[0]
		v2=l2[1]-l2[0]
		return np.cross(v1,v2)

	cline=counting_line['line']
	# this function can be made a bit more efficient but less readable
	for bl in bloblines:
		intersect,o=_line_segments_intersect(cline,bl)
		if intersect:
			# test direction
			# note:If I have not forgot something, in all cases, including dir_measurement=='previous' and mode=cross_mode
			# if direction is wrong for one lines it will be wrong for all lines, so when direction is bad I don't need to 'continue', I can return False
			direction=counting_line.get('direction',None)
			if direction is None:
				return True
			dirv=1 if direction=='left' else -1
			if counting_line.get('dir_measurement',None)=='previous':
				if mode==touch_mode:
					dirline=[get_centroid(blob.old_bounding_box),blob.centroid]
				else:
					# in this case I will use the already computed orientation o of current line
					return dirv*o>0
			else: # dir_measurement=='first'
				dirline=[blob.position_first_detected,blob.centroid]
			return lines_crossprod(np.array(dirline),np.array(cline))*dirv>0
	return False

def _get_dynamic_lines(bbox,oldbbox,what):
	x, y, w, h = bbox
	# I know centroid is already saved into blob (but not the old one), but carring an additional parameter around is annoying
	cc=get_centroid(bbox)
	ox, oy, ow, oh = oldbbox
	occ=get_centroid(oldbbox)
	lines={
		'tl':[(ox,oy),(x,y)],
		'tr':[(ox+ow,oy),(x+w,y)],
		'bl':[(ox,oy+oh),(x,y+h)],
		'br':[(ox+ow,oy+oh),(x+w,y+h)],
		'cc':[occ,cc],
	}
	if what == 'corners':
		return [lines[c] for c in ['tl','tr','bl','br']]
	return [lines[what]]

def _get_static_lines(bbox,what):
	x, y, w, h = bbox
	lines={
		'top' : [(x, y), (x + w, y)],
		'right' : [(x + w, y), (x + w, y + h)],
		'left' : [(x, y), (x, y + h)],
		'bottom' : [(x, y + h), (x + w, y + h)],
	}
	if what == 'box':
		return lines.values()
	return [lines[what]]
		

def attempt_count(blob, blob_id, counting_lines, counts):
	'''
	Check if a blob has crossed a counting line.
	'''
	for counting_line in counting_lines:
		label = counting_line['label']
		if label in blob.lines_crossed:
			continue
		try:
			mindist=counting_line['mindist']
			c=blob.centroid
			oc=blob.position_first_detected
			dist2=(c[0]-oc[0])*(c[0]-oc[0])+(c[1]-oc[1])*(c[1]-oc[1])
			if dist2<mindist*mindist:
				continue
		except KeyError as e:
			pass
		direction=counting_line.get('direction',None)
		lookfor=counting_line.get('lookfor','box')	
		if lookfor in ['top','right','left','bottom','box']:
			mode=touch_mode
		else:
			mode=cross_mode
		# if I need old_bounding_box but I don't have it skip this line
		if (direction is not None or mode==cross_mode) and (blob.old_bounding_box is None or blob.old_bounding_box==blob.bounding_box):
				continue
		# TODO
		# _get_static_lines and _get_dynamic_lines can be merged in a _get_lines function		
			 	
		if mode==touch_mode:
			bloblines=_get_static_lines(blob.bounding_box,lookfor)
		else:
			bloblines=_get_dynamic_lines(blob.bounding_box,blob.old_bounding_box,lookfor)	
			
		if _has_crossed_counting_line(counting_line,bloblines,blob,mode):
			if blob.type in counts[label]:
				counts[label][blob.type] += 1
			else:
				counts[label][blob.type] = 1

			blob.lines_crossed.append(label)

			logger.info('Object counted.', extra={
				'meta': {
					'label': 'OBJECT_COUNT',
					'id': blob_id,
					'type': blob.type,
					'counting_line': label,
					'position_first_detected': blob.position_first_detected,
					'position_counted': blob.centroid,
					'counted_at':time.time(),
					'counted_at_frame':progress.frame(),
				},
			})
	blob.old_bounding_box=blob.bounding_box
	return blob, counts
