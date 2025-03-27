import errno
import rospy
import baxter_dataflow


from sensor_msgs.msg import PointCloud
#   header
#   points
#   channel
from std_msgs.msg import Float32
from std_msgs.msg import UInt16


class SonarIO(object):
    #We need to have a subscriber for all the information topics for sonar.
    #And we need to have a publisher for all the lights for sonar.
    #And then we can make public functions to access or set these.
    def __init__(self):
        topic_base = "/robot/sonar/head_sonar"
        
        #State is a dictionary for the elements of PointCloud
        self._state = dict()

        #Make a subscriber for the state
        self._sub_state = rospy.Subscriber(
            topic_base + '/state',
            PointCloud,
            self._on_sonar_state)
        
        #Make sure that we HAVE the state
        baxter_dataflow.wait_for(
            lambda: len(self._state.keys()) != 0,
            timeout=2.0,
            timeout_msg="Failed to get current sonar state from %s" \
            % (topic_base,),
        )

        # Control: 
        # robot/sonar/head_sonar/
        #   approach_alert          Approach/ApproachAlert (No idea what this is, no publishers, only subscriber is realtime_loop)
        #   lights/
        #       green_level             Float32
        #       set_green_level       
        #       red_level               Float32
        #       set_red_level           
        #       state                   UInt16
        #       set_lights
        #   sonars_enabled              UInt16
        #   set_sonars_enabled
        #   state                   sensor_msgs/PointCloud


    #Updates the internal sonar state (it's a PointCloud)
    def _on_sonar_state(self, msg):
        self._state['header'] = msg.header
        self._state['points'] = msg.points
        self._state['channel'] = msg.channel

    def state(self):
        #Return the last points we  got.
        return self._state['points']
        
        
