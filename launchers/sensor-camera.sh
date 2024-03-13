#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

roslaunch --wait camera_driver camera_node.launch veh:=$VEHICLE_NAME


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
