#!/usr/bin/env python3
import collections
import numpy as np
import rospy
import enum
import itertools
from threading import Lock

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading, Segment, SegmentList, Vector2D

from lane_controller.controller import PurePursuitLaneController


class Color(enum.Enum):
    WHITE = Segment.WHITE
    YELLOW = Segment.YELLOW
    RED = Segment.RED

class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocitie.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # Add the node parameters to the parameters dictionary
        self.params = dict()
        self.pp_controller = PurePursuitLaneController(self.params)

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",
                                           Twist2DStamped,
                                           queue_size=1,
                                           dt_topic_type=TopicType.CONTROL)

        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber("~lane_pose",
                                                 LanePose,
                                                 self.cbLanePoses,
                                                 queue_size=1)

        self.log("Initialized!")
        
        self.sub_segments = rospy.Subscriber("/agent/ground_projection_node/lineseglist_out",
                                                 SegmentList,
                                                 self.cbSegList,
                                                 queue_size=1)

        self.log("Initialized!")
        
        # Velocity and Omega
        self.v = 0
        self.omega = 0
        # Velocity and Omega after update
        self.av = 0
        self.ao = 0
        
        # The Lookahead distance for Pure Pursuit
        self.lookahead_dist = 1.0
        
        #Length of the dictionary buffer containing of segment points based on their color in a deque.
        self.blength = 150
        self.seg_points = collections.defaultdict(lambda: collections.deque(maxlen=self.blength))
        
        # Variable to control the velocity of the bot
        self.max_velocity = 0.5
        
        #Setting a timer as suggested by the TA for the car command
        self.d_t = 0.5
        self.cc_timer = rospy.Timer(rospy.Duration.from_sec(self.d_t), self.car_command)
        
        # An offset parameter which is quite useful specially at curves
        self.offset = 0.15
        
        # Thread lock to access points deque
        self.lock_for_points = Lock()
        
        # Target(Follow) point
        self.target = np.asarray([0,0])
        
        # List of White points on the left of the yellow points
        self.wpl = []
        
        # Flag for White points
        self.wflag = 0


    def cbLanePoses(self, input_pose_msg):
        """Callback receiving pose messages

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
        """
        self.pose_msg = input_pose_msg
        #print(self.pose_msg)

        car_control_msg = Twist2DStamped()
        car_control_msg.header = self.pose_msg.header

        # TODO This needs to get changed
        car_control_msg.v = self.av
        car_control_msg.omega = self.ao

        self.publishCmd(car_control_msg)
    
    def cbSegList(self, input_segments):
        """Callback receiving segment messages

        Args:
            input_segments (:obj:`SegmentList`): Message containing information about detected segments.
        """
        
        self.header = input_segments.header
        segments = input_segments.segments
        #self.logdebug("Received {} new segments".format(len(segments)))
        for i, segment in enumerate(segments):
            color = Color(segment.color)
            assert color in [Color.RED, Color.YELLOW, Color.WHITE]
            with self.lock_for_points:
                self.seg_points[color].extend(segment.points)
        """with self.lock_for_points:
            yellow_points = self.seg_points[Color.YELLOW]
            for p in yellow_points:
                if p.y > 0.4:
                    print(" YELLOW DUCK(OR OUTLIER) POINTS: \n", p)"""

    
    def car_command(self, timer):
        #target = 0
        if not self.points_are_available(Color.YELLOW) and not self.points_are_available(Color.WHITE):
            self.logwarn("THERE ARE NO POINTS AVAILABLE")
            if self.v == 0 and self.omega == 0:
                self.logwarn("DUCKIEBOT CANT DETECT ANY SEGMENTS AND IS THEREFORE STAGNANT.")
                self.av = 0.05
                self.ao = 0
                print("COMMAND 1 EXECUTED")
            else:
                self.logwarn("SEGMENTS NOT DETECTED(NOT ABLE TO SEE)")
                self.av = 0
                self.ao = 0
                print("COMMAND 2 EXECUTED")
            #return
        
        else:
            
            if self.counting_points(Color.YELLOW) != 0:
                yellow_centroid = self.calculate_centroid(Color.YELLOW)
                self.target = yellow_centroid
                self.target[1] -= self.offset # moving the follow(target) point to the right.
                print("CONDITION TRUE")
            else:
                white_centroid = self.calculate_centroid(Color.WHITE)
                self.target = white_centroid
                self.target[1] += self.offset # moving the follow(target) point to the left.
                #print("PRINTING TARGET: ",target)
            """
            #print("WHITE RIGHT SEGMENT POINT: ", self.wpl)
            #if self.counting_points(Color.YELLOW) != 0:
            #self.right_white()
            #self.wflag = 1
            yellow_centroid = self.calculate_centroid(Color.YELLOW)
            white_centroid = self.calculate_centroid(Color.WHITE)
            self.target = (white_centroid + yellow_centroid) / 2
            else:
                #self.right_white()
                #white_centroid = self.calculate_centroid(Color.WHITE)
                #self.target = white_centroid
                #self.target[1] += self.offset # shift to the left.
                yellow_centroid = self.calculate_centroid(Color.YELLOW)
                white_centroid = self.calculate_centroid(Color.WHITE)
                self.target = (white_centroid + yellow_centroid) / 2
                self.target[1] += 0.1
                #print("PRINTING TARGET: ",target)"""
                
        print("YELLOW LINE SEGMENTS: ", self.counting_points(Color.YELLOW))
        print("WHITE LINE SEGMENTS: ", self.counting_points(Color.WHITE))
        #print(target)
        #self.target = target
        target_msg = Vector2D()
        target_msg.x = self.target[0]
        target_msg.y = self.target[1]
        #self.pub_follow_point.publish(target_msg)

        #target_dot_product =  self.target.dot(self.target)
        hypothenuse = np.sqrt(self.target.dot(self.target))
        sin_alpha = self.target[1] / hypothenuse
        
        min_speed = 0.1
        max_speed = 1.0

        # TODO: maybe play around with changing V depending on sin_alpha.
        v = self.max_velocity * (1 - abs(sin_alpha))
        v = np.clip(v, min_speed, max_speed)

        omega = 2 * sin_alpha / self.lookahead_dist
        
        self.av = v
        self.ao = omega

        # Emptying all the stored points in the deque
        self.empty_all_points()
    
    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
                
        self.pub_car_cmd.publish(car_cmd_msg)


    def cbParametersChanged(self):
        """Updates parameters in the controller object."""

        self.controller.update_parameters(self.params)
    
    def calculate_centroid(self, color=None):
        with self.lock_for_points:
            if color is None:
                self.logdebug("Averaging points of all colors")
                points_to_average = list(itertools.chain(*self.seg_points.values()))
            elif self.wflag == 1:
                #self.logdebug("Averaging points of color {}".format(color))
                points_to_average = self.wpl
            else:
                self.logdebug("Averaging points of color {}".format(color))
                points_to_average = self.seg_points[color]
            x = np.mean([p.x for p in points_to_average])
            y = np.mean([p.y for p in points_to_average])
        return np.asarray([x, y])
        # centroid = Point(x=x_centroid, y=y_centroid)
        # return centroid
    
    def points_are_available(self, color=None):
        return self.counting_points(color) != 0

    def counting_points(self, color=None):
        with self.lock_for_points:
            if color is None:
                return sum(len(values) for values in self.seg_points.values())
            return len(self.seg_points[color])

    def empty_all_points(self):
        with self.lock_for_points:
            for color, point_list in self.seg_points.items():
                point_list.clear()
    
    def right_white(self):
        with self.lock_for_points:
            yellow_points = self.seg_points[Color.YELLOW]
            white_points = self.seg_points[Color.WHITE]
            for p in white_points:
                #for q in yellow_points:
                    #if p.x > q.x:
                if p.y < 0:
                    self.wpl.append(p)
    """
    def clear_white_points(self,rpoint):
        with self.lock_for_points:
            for color, point_list in self.seg_points.items():
                if color == Color.WHITE:
                    point_list.clear()
                    point_list.extend(rpoint)
    """
if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name='lane_controller_node')
    # Keep it spinning
    rospy.spin()
