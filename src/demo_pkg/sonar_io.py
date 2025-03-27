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
        self.lights = self._Lights()
        self._sonars = 0
        #State is a dictionary for the elements of PointCloud
        self._state = dict()

        #Make a subscriber for the state
        self._sub_state = rospy.Subscriber(
            topic_base + '/state',
            PointCloud,
            self._on_sonar_state)
        
        #Make a subscriber for sonars_enabled
        self._sub_sonars = rospy.Subscriber(
            topic_base + '/sonars_enabled',
            UInt16,
            self._on_sonars)
        
        #Make sure that we HAVE the state
        baxter_dataflow.wait_for(
            lambda: len(self._state.keys()) != 0,
            timeout=2.0,
            timeout_msg="Failed to get current sonar state from %s" \
            % (topic_base,),
        )

        #Make a publisher for set_sonars_enabled
        self._pub_sonars = rospy.Publisher(
                topic_base + '/set_sonars_enabled',
                UInt16,
                queue_size=10)
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
        self._state['channels'] = msg.channels

    #Updates the internal sonars enabled
    def _on_sonars(self, msg):
        self._sonars = msg.data

    def state(self):
        #Return the last points we  got.
        return self._state['points']
    
    def set_sonars(self, value, timeout=2.0):
        #Control which sonars are enabled.
        #@type value: uint16
        #@param value: Int rep of binary for which sonars to have on. (Ex: 0b000000101010, 6th, 4th, and 2nd sonars on.) First sonar is facing foward, they go around clockwise.
        #@type timeout: float
        #@param timeout: Seconds to wait for the io to reflect command.
        #                If 0, just command once and return. [0]
        cmd = value
        #cmd.data = value
        print(cmd)
        self._pub_sonars.publish(cmd)
        if not timeout == 0:
            baxter_dataflow.wait_for(
                test=lambda: self._sonars == value,
                timeout=timeout,
                rate=100,
                timeout_msg=("Failed to command sonars to: %d" % (value,)),
                body=lambda: self._pub_sonars.publish(cmd)
            )
        
    def get_sonars(self):
        #Get the value of representing if the sonars are on or not.
        #Int rep of binary for which sonars are on. (Ex: 0b000000101010, 6th, 4th, and 2nd sonars on.)
        return self._sonars
    
    class _Lights(object):
        def __init__(self):
            topic_base = "/robot/sonar/head_sonar/lights"
            self._lights = 0
            #State is... just an UInt16. but we wanna make sure its their, so we will see if the 'data' value exists later.
            self._state = dict()

            #Make a subscriber for the state
            self._sub_state = rospy.Subscriber(
                topic_base + '/state',
                UInt16,
                self._on_lights_state)

            #Make sure that we HAVE the state
            baxter_dataflow.wait_for(
                lambda: len(self._state.keys()) != 0,
                timeout=2.0,
                timeout_msg="Failed to get current sonar state from %s" \
                % (topic_base,),
            )

            #Make a publisher for set_lights
            self._pub_lights = rospy.Publisher(
                    topic_base + '/set_lights',
                    UInt16,
                    queue_size=10)

        def _on_lights_state(self, msg):
            #Somewhat redundant...
            self._state['data'] = msg.data
            self._lights = msg.data

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
            print(cmd)
            self._pub_lights.publish(cmd)
            if not timeout == 0:
                baxter_dataflow.wait_for(
                    test=lambda: self._lights == value,
                    timeout=timeout,
                    rate=100,
                    timeout_msg=("Failed to command sonars to: %d" % (value,)),
                    body=lambda: self._pub_lights.publish(cmd)
                )   

        def get_lights(self):
        #Get the value of representing if the sonars are on or not.
        #Int rep of binary for which sonars are on. (Ex: 0b000000101010, 6th, 4th, and 2nd sonars on.)
            return self._lights
    
        pass