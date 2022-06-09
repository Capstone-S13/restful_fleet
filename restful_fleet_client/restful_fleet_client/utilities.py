from numpy import math
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import Vector3Stamped

def is_transform_close(first=TransformStamped, second=TransformStamped()):
    first = TransformStamped()
    second = TransformStamped()
    if (first.header.frame_id !=second.header.frame_id\
        or first.child_frame_id != second.child_frame_id):
        return False
    elapsed_sec = second.header.stamp - first.header.stamp.to_sec()
    first_pos = first.transform.translation
    second_pos = second.transform.translation
    distance = math.sqrt((first_pos.x-second_pos.x)**2 +\
        (first_pos.y-second_pos.y)**2)
    distance = abs(distance/elapsed_sec)
    return distance