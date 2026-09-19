uint8 FLEXIBLE=0
uint8 MOVABLE=1
uint8 ANCHORED=2
uint8 UNKNOWN=3
uint8 ERROR=4

uint8 VISION_UNKNOWN=0
uint8 VISION_WALL=1
uint8 VISION_CURTAIN=2

std_msgs/Header header
string request_id
uint8 state
float32 contact_probability
float32 mobility_probability
float32 mechanical_probability
float32 object_displacement_m
float32 resistance_ratio
float32 tracking_error
uint8 vision_classification
float32 vision_confidence
bool vision_backend_ok
uint16 vision_votes
uint16 vision_samples
string vision_reason
bool arm_returned
string error_code
string message
