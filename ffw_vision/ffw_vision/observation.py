import math

from ffw_vision_msgs.msg import ObjectObservation, ObjectObservationArray, PixelObservation


def make_selected_observation(config, frame, u, v, target_label=''):
    """Build a manual-selection observation for a frame the observer received."""
    if not (math.isfinite(u) and math.isfinite(v)):
        raise ValueError(f'pixel ({u}, {v}) is not finite')
    if not (-0.5 <= u < frame.width - 0.5 and -0.5 <= v < frame.height - 0.5):
        raise ValueError(f'pixel ({u}, {v}) is outside {frame.width}x{frame.height}')
    msg = PixelObservation()
    msg.header.stamp.sec = frame.sec
    msg.header.stamp.nanosec = frame.nanosec
    msg.header.frame_id = frame.frame_id
    msg.camera_id = config.camera_id
    msg.image_topic = config.image_topic
    msg.image_width = frame.width
    msg.image_height = frame.height
    msg.target_label = target_label
    msg.point_type = PixelObservation.POINT_TYPE_SELECTED_POINT
    msg.source_mode = PixelObservation.SOURCE_MANUAL
    msg.status = PixelObservation.STATUS_VALID
    msg.u = float(u)
    msg.v = float(v)
    return msg


def make_object_array(config, header, width, height, blobs, target_label):
    """Build an auto-detection array; header is the source image header."""
    msg = ObjectObservationArray()
    msg.header = header
    msg.camera_id = config.camera_id
    msg.image_topic = config.image_topic
    msg.image_width = width
    msg.image_height = height
    msg.source_mode = ObjectObservationArray.SOURCE_AUTO
    msg.status = (ObjectObservationArray.STATUS_VALID if blobs
                  else ObjectObservationArray.STATUS_NOT_DETECTED)
    for blob in blobs:
        obj = ObjectObservation()
        obj.target_label = target_label
        obj.pose_label = blob.pose_label
        obj.status = (ObjectObservation.STATUS_AMBIGUOUS if blob.ambiguous
                      else ObjectObservation.STATUS_VALID)
        obj.u, obj.v = blob.u, blob.v
        obj.length, obj.width = blob.length, blob.width
        obj.angle, obj.angle_period = blob.angle, blob.angle_period
        obj.has_score = False
        msg.objects.append(obj)
    return msg
