#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# dt-exec echo "This is an empty launch script. Update it to launch your application."
# roscore &
# sleep 5
#dt-exec rosrun my_package my_publisher_node.py
#dt-exec rosrun my_package my_subscriber_node.py
#dt-exec rosrun my_package my_subscriber_node1.py
#dt-exec rosrun my_package my_subscriber_node2.py
roslaunch my_package multiple_nodes.launch

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
