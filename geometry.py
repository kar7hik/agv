import math

def compute_yaw(detection):
    corners = detection.corners
    center = detection.center
    
    top_mid_x = (corners[0][0] + corners[1][0]) / 2
    top_mid_y = (corners[0][1] + corners[1][1]) / 2
    
    dx = top_mid_x - center[0]
    dy = center[1] - top_mid_y
    
    yaw = math.degrees(math.atan2(dy, dx))
    
    return yaw
    
def compute_x_offset(detection):
    return float(detection.pose_t[0][0])