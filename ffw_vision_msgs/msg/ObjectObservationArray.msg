# All objects found in one source image.
# STATUS_NOT_DETECTED: the image was processed and objects is empty.

std_msgs/Header header  # source image header (capture stamp, optical frame_id)
string camera_id
string image_topic
uint32 image_width
uint32 image_height

uint8 SOURCE_UNSET=0
uint8 SOURCE_MANUAL=1
uint8 SOURCE_AUTO=2
uint8 source_mode

uint8 STATUS_UNSET=0
uint8 STATUS_VALID=1
uint8 STATUS_NOT_DETECTED=2
uint8 status

ObjectObservation[] objects
