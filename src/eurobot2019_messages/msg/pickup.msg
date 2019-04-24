# Is the target puck vertical?
bool is_vertical

# Have we reached the correct position yet?
# Note that the picker will be told to open up and get to the right z
# Before continuing by setting this to false
bool pos_reached

# If this is true, it signals the robot has backed up
bool has_backed_up

bool is_high

# 0 is "idle at bottom front"
# 1 is red
# 2 is green
# 3 is blue
# 4 is gold
# 5 is "idle at top back"
uint8 colour
