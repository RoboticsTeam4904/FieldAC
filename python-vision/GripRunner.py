pipeline = False

def run(image):
	global pipeline
	if not pipeline:
		editCode()
	pipeline.process(image)
	return pipeline.filter_contours_output

def calibrate(hsv=False, area=False):
	if hsv:
		pipeline.__hsv_threshold_hue = hsv[0]
		pipeline.__hsv_threshold_saturation = hsv[1]
		pipeline.__hsv_threshold_value = hsv[2]
	if area:
		pipeline.__filter_contours_min_area = area


# This code was not what we planned to write; in fact, we still cringe at its sight
# But, seeing how our code is a little rudamentary, we are forced to add some artificial intelligence. 
# Sadly, we have written code that transcends programming itself. This code writes its own code. 
# This horrid beast has taken our jobs and our enthusiasm. If we never code again, know it was because of this function.
# So Team 4904 presents you the next level of programming:
def editCode():
    import re
    from config import withOpenCV3
    code = open('grip.py', 'r').read()
    if withOpenCV3:
        code = re.sub('    contours, hierarchy =cv2.findContours', '    im2, contours, hierarchy =cv2.findContours', code)
    else:
        code = re.sub('im2, contours, hierarchy =cv2.findContours', 'contours, hierarchy =cv2.findContours', code)
    open('grip_edited.py', 'w').write(code)

    from grip_edited import GripPipeline  # TODO change the default module and class, if needed
    global pipeline
    pipeline = GripPipeline()
