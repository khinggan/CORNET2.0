import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point
from nav_msgs.msg import Odometry
import math
import tf_transformations
import numpy as np
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

# Turtlebot3 velocity constraint
MAX_LINEAR_SPEED = 0.22
MAX_ANGULAR_SPEED = 2.84

# controller coefficients
Kp_rho = 3
Kp_alpha = 8
Kp_beta = -1.5

# run frequecy
frequency = 0.05    # [seconds]

# Pure Pursuit Algorithm Parameters
LOOK_AHEAD_DISTANCE = 0.4    # [m]

# Reach threshold, sum formation error is lower then this, move to next target virtual structure
REACH_THRESHOLD = 0.8    # [m]


class VirtualStructure(Node):

    def __init__(self):
        # init node "virtualstructure"
        super().__init__('virtualstructure', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)    

        ######### initialization ##########
        # 1. Robot number, desired distance among robots
        self.N = round(self.get_parameter('N').get_parameter_value().double_value)
        self.ddistance = self.get_parameter('ddistance').get_parameter_value().double_value
        # 2. Waypoint: ellipse or sin path
        self.waypoints = self.get_waypoints("ellipse")
        # 3. Gazebo -> /odom -> position ===> init self.robots_poses
        self.robots_poses = {}
        for r in range(self.N):
            self.create_subscription(Odometry, '/agent_{}/odom'.format(r), lambda data, id=r: self.callback_robots(data, id), 10)
        # 4. /cmd_vel for each topic
        self.velocity_publisher = [self.create_publisher(Twist, '/agent_{}/cmd_vel'.format(i), 10) for i in range(self.N)]
        # 5. Base Virtual Structure positions
        self.base_virtual_structure = self.compute_VS(3)     #  from regular polygon to position, in this case, regular triangle
        
        ######### virtual structure algorithm ##########
        self.target_idx = 0    # target index of waypoints
        # Initial target vs
        self.target_vs = self.get_target_vs()
        self.create_timer(frequency, self.virtual_structure)

        ######### visualization parameters ##########
        self.vs_visualize_publisher = self.create_publisher(Marker, "virtualstructure", 10)
        self.robot_visualize_publisher = [self.create_publisher(Marker, "robot_{}".format(i), 10) for i in range(self.N)]
        self.waypoints_publisher = self.create_publisher(Marker, "virtualstructure", 10)
        self.real_path_publisher = [self.create_publisher(Marker, "robot_{}".format(i), 10) for i in range(self.N)]

        ######### performance evaluation parameters ##########
        self.formation_error_publisher_12 = self.create_publisher(Float32, "e12", 10)
        self.formation_error_publisher_13 = self.create_publisher(Float32, "e13", 10)
        self.formation_error_publisher_23 = self.create_publisher(Float32, "e23", 10)
        # self.performance_evaluation_timer = self.create_timer(0.05, self.performance_evaluation)
    
    def callback_robots(self, data, id):
        self.robots_poses[id] = data.pose.pose    # position and orientation

    def virtual_structure(self):
        if len(self.robots_poses) == self.N:
            if self.check_reach():    # All robot get positino  and robots 
                                                                       # reach the desired virtual structure target position
                if self.target_idx < len(self.waypoints[0]) - 1:
                    self.target_idx += 1
                    self.target_vs = self.get_target_vs()
                else:
                    self.target_idx = len(self.waypoints[0]) - 1
            else:
                for i in range(self.N):
                    vi, wi = self.move2pose(
                        self.robots_poses[i].position.x, 
                        self.robots_poses[i].position.y, 
                        tf_transformations.euler_from_quaternion([self.robots_poses[i].orientation.x, self.robots_poses[i].orientation.y, self.robots_poses[i].orientation.z, self.robots_poses[i].orientation.w])[2],
                        self.target_vs[0, i],  
                        self.target_vs[1, i], 
                        self.target_vs[2, i], 
                    )
                    if abs(vi) > MAX_LINEAR_SPEED:
                        vi= np.sign(vi) * MAX_LINEAR_SPEED
                    if abs(wi) > MAX_ANGULAR_SPEED:
                        wi = np.sign(wi) * MAX_ANGULAR_SPEED
                    msgi = Twist()
                    msgi.linear.x = vi
                    msgi.angular.z = wi
                    self.velocity_publisher[i].publish(msgi)
            self.visualize()
            self.performance_evaluation()

    def compute_VS(self, shape):
        vs = np.zeros((shape, self.N))
        radius = 1.0    # [m]
        for i in range(self.N):
            angle = 2*math.pi / self.N * (i)
            vs[0:2, i] = radius * np.asarray([math.cos(angle), math.sin(angle)], dtype=np.float)
        return vs

    def get_waypoints(self, type):
        path_X, path_Y = None, None
        if type == "sin":
            path_X = np.arange(0, 20, 0.1)
            path_Y = [math.sin(ix / 2.0) * ix / 2.0 for ix in path_X]
        elif type == "ellipse":
            thetas = np.linspace(-np.pi, np.pi, 100)
            a, b = 5, 2.5
            path_X = [a * np.cos(theta) + a for theta in thetas]
            path_Y = [b * np.sin(theta) for theta in thetas]
        else:
            path_X = [0.0]
            path_Y = [0.0]
        return path_X, path_Y

    def get_target_vs(self):    
        '''
        Get target virtual structure's positions from self.target_idx and base virtual structure
        return target virtual structure positions (shape is (3, self.N))
        [
            ( x1 )   ( x2 )    ( x3 )  ...
            ( y1 )   ( y2 )    ( y3 )  ...  N (robot number)
            ( z1 )   ( z2 )    ( z3 )  ...
        ]
        '''
        if self.target_idx < len(self.waypoints[0]):
            target_X, target_Y = self.waypoints[0][self.target_idx], self.waypoints[1][self.target_idx]
        else: 
            self.get_logger().info(str("idx out of range of waypoints"))
            target_X, target_Y = self.waypoints[0][-1], self.waypoints[1][-1]
        increase = np.asarray([target_X, target_Y, 0], dtype=np.float)
        target_vs = np.transpose(np.transpose(self.base_virtual_structure) + increase)    # move base virtual structure
        return target_vs

    def move2pose(self, x, y, theta, x_goal, y_goal, theta_goal):
        """
        Constructs an instantiate of the PathFinderController for navigating a
        3-DOF wheeled robot on a 2D plane
        Parameters
        ----------
        Kp_rho : The linear velocity gain to translate the robot along a line
                towards the goal
        Kp_alpha : The angular velocity gain to rotate the robot towards the goal
        Kp_beta : The offset angular velocity gain accounting for smooth merging to
                the goal angle (i.e., it helps the robot heading to be parallel
                to the target angle.)
        Returns the control command for the linear and angular velocities as
            well as the distance to goal
            Parameters
            ----------
            x_diff : The position of target with respect to current robot position
                    in x direction
            y_diff : The position of target with respect to current robot position
                    in y direction
            theta : The current heading angle of robot with respect to x axis
            theta_goal: The target angle of robot with respect to x axis
            Returns
            -------
            rho : The distance between the robot and the goal position
            v : Command linear velocity
            w : Command angular velocity
            

            Description of local variables:
            - alpha is the angle to the goal relative to the heading of the robot
            - beta is the angle between the robot's position and the goal
            position plus the goal angle
            - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
            the goal
            - Kp_beta*beta rotates the line so that it is parallel to the goal
            angle
            
            Note:
            we restrict alpha and beta (angle differences) to the range
            [-pi, pi] to prevent unstable behavior e.g. difference going
            from 0 rad to 2*pi rad with slight turn
        """
        x_diff = x_goal - x
        y_diff = y_goal - y
        
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = 0.0 if rho < 0.1 else Kp_rho * rho
        w = 0.0 if v == 0 else Kp_alpha * alpha + Kp_beta * beta

        if np.pi / 2 < alpha < np.pi or -np.pi < alpha < -np.pi / 2:
            v = -v     
        return v, w

    def check_reach(self):
        # check whether all robots reach the target position
        sum_dist = 0
        for i in range(self.N):
            pose_x = self.robots_poses[i].position.x
            pose_y = self.robots_poses[i].position.y
            sum_dist += math.hypot(pose_x - self.target_vs[0, i], pose_y - self.target_vs[1, i])
        if sum_dist < REACH_THRESHOLD:
            return True
        else:
            return False

    def visualize(self):
        # initialize virtual structure visions
        points = []
        for p in np.transpose(self.target_vs):
            point = Point()
            point.x = -p[0]
            point.y = -p[1]
            point.z = -p[2]
            points.append(point)
        virtualstructure = Marker()
        virtualstructure.header.frame_id = "odom"
        virtualstructure.header.stamp = self.get_clock().now().to_msg()
        virtualstructure.id = 0
        virtualstructure.action = Marker.ADD
        virtualstructure.type = Marker.POINTS
        virtualstructure.pose.orientation.w = 0.0
        virtualstructure.color.r = 1.0
        virtualstructure.color.g = 0.0
        virtualstructure.color.b = 0.0
        virtualstructure.color.a = 1.0
        virtualstructure.scale.x = 0.1
        virtualstructure.scale.y = 0.1
        virtualstructure.scale.z = 0.1
        virtualstructure.points = points
        
        self.vs_visualize_publisher.publish(virtualstructure)

        # visualize robots
        if len(self.robots_poses) == self.N:
            for i in range(self.N):
                robot = Marker()
                robot.header.frame_id = "odom"
                robot.header.stamp = self.get_clock().now().to_msg()
                robot.id = 0
                robot.type = Marker.SPHERE
                robot.pose = self.robots_poses[i]
                robot.color.r = 0.0
                robot.color.g = 1.0
                robot.color.b = 0.0
                robot.color.a = 1.0
                robot.scale.x = 0.3
                robot.scale.y = 0.3
                robot.scale.z = 0.3
                self.robot_visualize_publisher[i].publish(robot)

        # visualize waypoints
        waypoints = []
        for x, y in zip(self.waypoints[0], self.waypoints[1]):
            waypoint = Point()
            waypoint.x = -x
            waypoint.y = -y
            waypoint.z = 0.0
            waypoints.append(waypoint)

        waypoints_visualize = Marker()
        waypoints_visualize.header.frame_id = "odom"
        waypoints_visualize.header.stamp = self.get_clock().now().to_msg()
        waypoints_visualize.ns = "path"
        waypoints_visualize.id = 0
        waypoints_visualize.action = Marker.ADD
        waypoints_visualize.pose.orientation.w=0.0
        waypoints_visualize.color.r = 0.0
        waypoints_visualize.color.g = 0.0
        waypoints_visualize.color.b = 1.0
        waypoints_visualize.color.a = 1.0
        waypoints_visualize.scale.x = 0.1
        waypoints_visualize.type = 4

        waypoints_visualize.points = waypoints

        self.waypoints_publisher.publish(waypoints_visualize)

        # visualize robots path

    def performance_evaluation(self):
        if len(self.robots_poses) == self.N:
            # e12 = current distance - desired distance, e13, e23 
            e12 = math.hypot(self.robots_poses[0].position.x - self.robots_poses[1].position.x, self.robots_poses[0].position.y - self.robots_poses[1].position.y) - math.sqrt(3)
            e13 = math.hypot(self.robots_poses[0].position.x - self.robots_poses[2].position.x, self.robots_poses[0].position.y - self.robots_poses[2].position.y) - math.sqrt(3)
            e23 = math.hypot(self.robots_poses[1].position.x - self.robots_poses[2].position.x, self.robots_poses[1].position.y - self.robots_poses[2].position.y) - math.sqrt(3)

            msg12, msg13, msg23 = Float32(), Float32(), Float32()
            msg12.data, msg13.data, msg23.data = e12, e13, e23

            self.formation_error_publisher_12.publish(msg12)
            self.formation_error_publisher_13.publish(msg13)
            self.formation_error_publisher_23.publish(msg23)

def main():
    rclpy.init()
    virtual_structure = VirtualStructure()
    rclpy.spin(virtual_structure)
    virtual_structure.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()