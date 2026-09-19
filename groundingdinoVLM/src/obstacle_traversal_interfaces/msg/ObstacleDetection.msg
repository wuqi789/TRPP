uint8 UNKNOWN=0
uint8 WALL=1
uint8 CURTAIN=2

std_msgs/Header header
string source
uint8 classification
float32 confidence
bool backend_ok
string error_code
string reason
string[] metric_names
float64[] metric_values
