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
            self._red_level = -1
            self._green_level = -1

            #Make a subscriber for the lights
            self._sub_state = rospy.Subscriber(
                topic_base + '/state',
                UInt16,
                self._on_lights)
            
            #Make a subscriber for the red_level
            self._sub_red_level = rospy.Subscriber(
                topic_base + '/red_level',
                Float32,
                self._on_red_level)
            
            #Make a subscriber for the green_level
            self._sub_green_level = rospy.Subscriber(
                topic_base + '/green_level',
                Float32,
                self._on_green_level)

            #Make sure that we HAVE the state
            baxter_dataflow.wait_for(
                lambda: self._lights == -1,
                timeout=2.0,
                timeout_msg="Failed to get current sonar light state from %s" \
                % (topic_base,),
            )

            #Make sure that we HAVE the red_level
            baxter_dataflow.wait_for(
                lambda: self._red_level == -1,
                timeout=2.0,
                timeout_msg="Failed to get current sonar light red_level from %s" \
                % (topic_base,),
            )

            #Make sure that we HAVE the green_level
            baxter_dataflow.wait_for(
                lambda: self._green_level == -1,
                timeout=2.0,
                timeout_msg="Failed to get current sonar light green_level from %s" \
                % (topic_base,),
            )

            #Make a publisher for set_lights
            self._pub_lights = rospy.Publisher(
                    topic_base + '/set_lights',
                    UInt16,
                    queue_size=10)
            
            #Make a publisher for set_red_level
            self._pub_red_level = rospy.Publisher(
                    topic_base + '/set_red_level',
                    Float32,
                    queue_size=10)
            
            #Make a publisher for set_green_level
            self._pub_green_level = rospy.Publisher(
                    topic_base + '/set_green_level',
                    Float32,
                    queue_size=10)

        def _on_lights(self, msg):
            self._lights = msg.data
            #print("Got some light update", msg)

        def _on_red_level(self, msg):
            self._red_level = msg.data

        def _on_green_level(self, msg):
            self._green_level = msg.data

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

            if not timeout == 0:
                baxter_dataflow.wait_for(
                    test=lambda: (self._lights & 4095) == (value & 4095),
                    timeout=timeout,
                    rate=100,
                    timeout_msg=("Failed to command sonar lights to: %d" % (value,)),
                    body=lambda: self._pub_lights.publish(cmd)
                )

        def get_lights(self):
        #Get the value of representing if the lights are being FORCED on or not. it will be 0 even if the sonar is turning a light on.
        #Int rep of binary for which sonars are on. (Ex: 0b000000101010, 6th, 4th, and 2nd sonars on.)
            return self._lights
        
        def set_red_level(self, value, timeout=2.0):
            #Value 0-100 for brightness of the red
            cmd = value
            #cmd.data = value
            self._pub_red_level.publish(cmd)

            #if not timeout == 0:
            #    baxter_dataflow.wait_for(
            #        test=lambda: self._red_level != value,
            #        timeout=timeout,
            #        rate=100,
            #        timeout_msg=("Failed to command sonar red level to: %d" % (value,)),
            #        body=lambda: self._pub_red_level.publish(cmd)
             #   )

        def get_red_level(self):
            return self._red_level
        
        def set_green_level(self, value, timeout=2.0):
            #Value 0-100 for brightness of the red
            cmd = value
            #cmd.data = value
            self._pub_green_level.publish(cmd)

            #if not timeout == 0:
            #    baxter_dataflow.wait_for(
            #        test=lambda: self._green_level != value,
            #        timeout=timeout,
            #        rate=100,
            #        timeout_msg=("Failed to command sonar green level to: %d" % (value,)),
            #        body=lambda: self._pub_green_level.publish(cmd)
            #    )

        def get_green_level(self):
            return self._green_level
        

             
    
        pass