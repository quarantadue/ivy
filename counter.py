# pylint: disable=missing-module-docstring,invalid-name

import time
from util.logger import get_logger
from util.bounding_box import get_centroid
from progress import get_ProgressCounter

logger = get_logger()
progress=get_ProgressCounter()

# I can count in two modes:
# - static mode, uses only current frame, look if the bound box cross the counting line
# - dynamic mode, calculate movement vector using current and previous frame, can discriminate direction 

# line['direction'] can be:
# None (default, old behavior) : count in static mode
# 'left' : count only object that are moving leftward (left refers to an observer that is on the first point of the line and faces the second)
# 'right' : count only objects that are moving rightward
# 'both' : count in dynamic mode, summing both directions

# line['lookfor'] can be:
# when in static mode, specify what lines of bounding box are used to count the crossing. 
#	It can be one of 'left','right','top','bottom','any'. Default is 'all' (old behavior)
# when in dynamic mode, specify what point of the bounding boxes are used to create lines (between old and new bb)
# that are then use to test the crossing
#	it can be 'tl' (top left), 'tr' (top right), 'bl' (bottom left), 'br' (bottom right), 'cc' (centroid, default) , 'any' (any)

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
		return True,o3
		
	# do I really want to ckeck for the extremely rare cases where all points are colinear?

	if o1 == 0 and is_on_segment(p1, p2, q1):
		return True,o3

	if o2 == 0 and is_on_segment(p1, q2, q1):
		return True,o3

	if o3 == 0 and is_on_segment(p2, p1, q2):
		return True,o3

	if o4 == 0 and is_on_segment(p2, q1, q2):
		return True,o3

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

def _has_crossed_counting_line(counting_line, bloblines,direction):
	'''
	Check if at least one object line cosses the countin line
	'''
	if direction=='left':
		dir=1
	elif direction=='right':
		dir=-1
	else:
		dir=0
	for bl in bloblines:
		intersect,o=_line_segments_intersect(counting_line,bl)
		if intersect and (dir==0 or dir*o>0):
			return True
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
	if what == 'any':
		return lines.values()
	try:
		return [lines[what]]
	except KeyError as e:
		return [lines['cc']]	

def _get_static_lines(bbox,what):
	x, y, w, h = bbox
	lines={
		'top' : [(x, y), (x + w, y)],
		'right' : [(x + w, y), (x + w, y + h)],
		'left' : [(x, y), (x, y + h)],
		'bottom' : [(x, y + h), (x + w, y + h)],
	}
	try:
		return [lines[what]]
	except KeyError as e:
		return lines.values()

def attempt_count(blob, blob_id, counting_lines, counts):
	'''
	Check if a blob has crossed a counting line.
	'''
	for counting_line in counting_lines:
		label = counting_line['label']
		if label in blob.lines_crossed:
			continue
		direction=counting_line.get('direction',None)
		if direction in ['left','right','both']:
			#dynamic mode
			if blob.old_bounding_box is None or blob.old_bounding_box==blob.bounding_box:
				# blob is new, I can't measure movement
				continue
			bloblines=_get_dynamic_lines(blob.bounding_box,blob.old_bounding_box,counting_line.get('lookfor',None))
		else:
			bloblines=_get_static_lines(blob.bounding_box,counting_line.get('lookfor',None))
			bloblines=None

		if _has_crossed_counting_line(counting_line['line'],bloblines,direction):
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
