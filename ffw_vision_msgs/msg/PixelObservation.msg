# One pixel observation taken from one source image.
#
# Pixel convention: (u, v) refer to the source image as received, with no resize or crop.
# u is the column (+ right) and v is the row (+ down). An integer (u, v) is the center of
# that pixel, the same convention as the K matrix in sensor_msgs/CameraInfo.
# Read status first. u and v are NaN unless status is STATUS_VALID.

# Copy of the source image header. Never replaced with the receive or publish time.
#   stamp:    capture time of the source image
#   frame_id: optical frame of the source image
std_msgs/Header header

# Camera identifier from the vision configuration: head, left_wrist or right_wrist
string camera_id

# Image topic the coordinates refer to (tells a raw stream from a rectified one)
string image_topic

# Source image size in pixels
uint32 image_width
uint32 image_height

# Requested or detected target kind. May be empty for manual selection.
string target_label

# Meaning of (u, v). Manual selection uses POINT_TYPE_SELECTED_POINT:
# a point picked on the image, not an object center or a grasp point.
string POINT_TYPE_SELECTED_POINT="selected_point"
string point_type

uint8 SOURCE_UNSET=0
uint8 SOURCE_MANUAL=1
uint8 SOURCE_AUTO=2
uint8 source_mode

uint8 STATUS_UNSET=0
uint8 STATUS_VALID=1
uint8 STATUS_NOT_DETECTED=2
uint8 STATUS_AMBIGUOUS=3
uint8 status

float64 u
float64 v
