import errno
import rospy
import baxter_dataflow


from sensor_msgs.msg import PointCloud
#   header
#   points
#   channel
from std_msgs.msg import Float32
from std_msgs.msg import UInt16


class SonarLightsIO(object):
        def __init__(self):
            topic_base = "/robot/sonar/head_sonar/lights"
            self._lights = -1

            #Make a subscriber for the lights
            self._sub_state = rospy.Subscriber(
                topic_base + '/state',
                UInt16,
                self._on_lights)

            #Make sure that we HAVE the state
            baxter_dataflow.wait_for(
                lambda: self._lights == -1,
                timeout=2.0,
                timeout_msg="Failed to get current sonar state from %s" \
                % (topic_base,),
            )

            #Make a publisher for set_lights
            self._pub_lights = rospy.Publisher(
                    topic_base + '/set_lights',
                    UInt16,
                    queue_size=10)

        def _on_lights(self, msg):
            self._lights = msg.data
            print("Got some light update", msg)

        def set_lights(self, value, timeout=2.0):
            #Control which sonar lights are enabled.
            #This must be done at a fast rate or it doesn't work, from documentation i can find that should be 100+
            #@type value: uint16
            #@param value: Int rep of binary for which sonars to have on. (Ex: 0b000000101010, 6th, 4th, and 2nd sonars on.) First sonar is facing foward, they go around clockwise.
            #@type timeout: float
            #@param timeout: Seconds to wait for the io to reflect command.
            #                If 0, just command once and return. [0]
            cmd = value
            #cmd.data = value
            self._pub_lights.publish(cmd)

            #small problem: for some reason, the lights never actually say they are on,,, so... yk
            #if not timeout == 0:
            #    baxter_dataflow.wait_for(
            #        test=lambda: self._lights == value,
            #        timeout=timeout,
            #        rate=100,
            #        timeout_msg=("Failed to command lights to: %d" % (value,)),
            #        body=lambda: self._pub_lights.publish(cmd)
            #    )   

        def get_lights(self):
        #Get the value of representing if the sonars are on or not.
        #Int rep of binary for which sonars are on. (Ex: 0b000000101010, 6th, 4th, and 2nd sonars on.)
            return self._lights
    
        pass