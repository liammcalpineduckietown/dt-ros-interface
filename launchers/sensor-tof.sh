#!/bin/bash
# NOTE:
# this is kept for backwards compatibility. It is recommended to use the new sensor-specific launchers.

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# If the environment variable ROBOT_CONFIGURATION is set to DD24, run dt-launcher-sensor-tof-bottom
if [ "$ROBOT_CONFIGURATION" == "DD24" ]; then
    dt-launcher-sensor-tof-bottom
fi


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
