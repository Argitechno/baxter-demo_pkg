import errno
import rospy
import baxter_dataflow
from sensor_msgs.msg import Imu

class AccelerometerIO(object):

    def __init__(self, component_id):
        self._id = component_id
        self._component_type = 'accelerometer'
        topic_base = '/robot/' + self._component_type + '/' + self._id

        #State is a dictionary for the elements of Imu
        self._state = dict()

        #Make a subscriber for the state
        self._sub_state = rospy.Subscriber(
            topic_base + '/state',
            Imu,
            self._on_acc_state)
        
        #Make sure that we HAVE the state
        baxter_dataflow.wait_for(
            lambda: len(self._state.keys()) != 0,
            timeout=2.0,
            timeout_msg="Failed to get current accelerometer state from %s" \
            % (topic_base,),
        )

    #Updates the internal imu state
    def _on_acc_state(self, msg):
        print(msg)
        self._state['header'] = msg.header
        self._state['orientation'] = msg.orientation
        self._state['orientation_covariance'] = msg.orientation_covariance
        self._state['angular_velocity'] = msg.angular_velocity
        self._state['angular_velocity_covariance'] = msg.angular_velocity_covariance
        self._state['linear_acceleration'] = msg.linear_acceleration
        self._state['linear_acceleration_covariance'] = msg.linear_acceleration_covariance
    
    