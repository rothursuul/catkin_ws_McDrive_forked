import numpy as np
import rospy

from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand

steeringCommand = NormalizedSteeringCommand()
speedCommand	= SpeedCommand()

steeringCommand.value	 = 1.0
speedCommand.value	 = 0.2


