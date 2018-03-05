#!/usr/bin/env python
import sys
import roslib
import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose2D
import geodesy.utm
import tf
import math
import collections

roslib.load_manifest('gps_localization')

RADIUS_EARTH = 6378000


class GPSLocalization(object):
    '''
        GPS Localization
    '''
    tf_br = tf.TransformBroadcaster()

    publish_map_tf = True

    frame_id = '/map'
    child_frame_id = '/odom'

    # Adoption rate x, y, theta
    adoption_rate = [1, 1, 1]

    def __init__(self):
        '''
            Initialize the odometry node.
        '''
        self.sensor_offset = np.array([0.6, 0.0, 0.0])

        self.pose = self.sub_relative_pose(np.array([0, 0, 0]), self.sensor_offset)
        self.delta = self.sub_relative_pose(np.array([0, 0, 0]), self.sensor_offset)
        self.measurement_queue = collections.deque()
        self.max_queue_length = 120
        self.accuracy_threshold = 30.0
        self.robot_path = Path()
        self.last_odom = None
        self.odom_settle = 50
        self.odom_skip = 0

        self.odom_sub = rospy.Subscriber("~odom", Odometry, self.on_odometry)
        self.gps_sub = rospy.Subscriber("~gps", NavSatFix, self.on_gps)
        self.init_sub = rospy.Subscriber("~init_pos", Pose2D, self.init_map)

        self.odom_pub = rospy.Publisher("map_odometry", Odometry, queue_size=1)

        self.observation_path_pub = rospy.Publisher("map_observation_path", Path, queue_size=1)
        self.map_pose_path_pub = rospy.Publisher("map_pose_path", Path, queue_size=1)
        self.robot_path_pub = rospy.Publisher("map_robot_path", Path, queue_size=1)

    def init_map(self, data):
        self.pose = np.array([data.x, data.y, data.theta])
        self.measurement_queue.clear()
        self.publish(rospy.Time.now())

    def extract_yaw(self, pose):
        '''
            Extract the yaw.
        '''
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
        return tf.transformations.euler_from_quaternion(quaternion)[2]

    def extract_pose(self, pose):
        '''
            Extract the pose from the pose event.
        '''
        return np.array([pose.pose.position.x, pose.pose.position.y, self.extract_yaw(pose)])

    def relative_pose(self, pose1, pose2):
        '''
            Calculate the relative pose between two poses.
            From pose1 to pose2.
        '''
        x = pose2[0] - pose1[0]
        y = pose2[1] - pose1[1]

        dtheta = pose2[2] - pose1[2]

        delta_x = math.cos(-pose1[2]) * x - math.sin(-pose1[2]) * y
        delta_y = math.sin(-pose1[2]) * x + math.cos(-pose1[2]) * y

        return np.array([delta_x, delta_y, dtheta])

    def add_relative_pose(self, pose1, pose2):
        '''
            Add a relative pose to a normal pose.
        '''

        new_theta = pose2[2] + pose1[2]
        if new_theta > math.pi:
            new_theta -= 2 * math.pi
        if new_theta < -math.pi:
            new_theta += 2 * math.pi

        delta_x = math.cos(pose1[2]) * pose2[0] - math.sin(pose1[2]) * pose2[1]
        delta_y = math.sin(pose1[2]) * pose2[0] + math.cos(pose1[2]) * pose2[1]

        new_x = pose1[0] + delta_x
        new_y = pose1[1] + delta_y

        return np.array([new_x, new_y, new_theta])

    def sub_relative_pose(self, pose1, pose2):
        '''
            Add a relative pose to a normal pose.
        '''

        new_theta = pose1[2] - pose2[2]
        if new_theta > math.pi:
            new_theta -= 2 * math.pi
        if new_theta < -math.pi:
            new_theta += 2 * math.pi

        delta_x = math.cos(new_theta) * pose2[0] - math.sin(new_theta) * pose2[1]
        delta_y = math.sin(new_theta) * pose2[0] + math.cos(new_theta) * pose2[1]

        new_x = pose1[0] - delta_x
        new_y = pose1[1] - delta_y

        return np.array([new_x, new_y, new_theta])

    def calc_mean(self, data):
        mean = [0, 0]
        for x in data:
            mean[0] += x[0]
            mean[1] += x[1]
        mean[0] /= len(data)
        mean[1] /= len(data)
        return mean

    def feature_transform(self, l, r):
        '''
            Calculate the feature transform from l to r.

            Returns the calculated:
            * scale
            * rotation
            * transition in x
            * transition in y
            * center of transformation
        '''
        if len(r) != len(l):
            print("Invalid dataset, l and r must have the same size!")
            return None, None, None, None, None
        l_mean = np.array(self.calc_mean(l))
        r_mean = np.array(self.calc_mean(r))

        cs = 0
        ss = 0
        rr = 0
        ll = 0
        for i in range(len(l)):
            l_i = np.array(l[i]) - l_mean
            r_i = np.array(r[i]) - r_mean

            cs +=  r_i[0] * l_i[0] + r_i[1] * l_i[1]
            ss += -r_i[0] * l_i[1] + r_i[1] * l_i[0]
            rr +=  r_i[0] * r_i[0] + r_i[1] * r_i[1]
            ll +=  l_i[0] * l_i[0] + l_i[1] * l_i[1]

        # scale = math.sqrt(rr/ll)
        scale = 1

        denom = math.sqrt(cs * cs + ss * ss)
        if abs(denom) < 0.00001:
            print("Numerical instability, denominator is: " + str(denom))
            return None, None, None, None, None
        c = cs / denom
        s = ss / denom
        theta = math.atan2(s, c)
        tx = r_mean[0] - scale * (c * l_mean[0] - s * l_mean[1])
        ty = r_mean[1] - scale * (s * l_mean[0] + c * l_mean[1])

        return scale, theta, tx, ty, l_mean

    def apply_feature_transform_on_pose(self, theta, tx, ty, center, pose):
        # Convert center to pose
        center = np.array([center[0], center[1], 0])

        c = math.cos(theta)
        s = math.sin(theta)

        new_x = (pose[0] * c - pose[1] * s) + tx
        new_y = (pose[0] * s + pose[1] * c) + ty
        new_theta = theta + pose[2]

        if new_theta > math.pi:
            new_theta -= 2 * math.pi
        if new_theta < -math.pi:
            new_theta += 2 * math.pi

        return np.array([new_x, new_y, new_theta])

    def on_odometry(self, data):
        '''
            Capture where the odometry thinks we are.
        '''
        if self.odom_settle > 0:
            self.odom_settle -= 1
            return

        cur_pose = self.extract_pose(data.pose)
        if self.last_odom is not None:
            last_pose = self.extract_pose(self.last_odom.pose)

            delta = self.relative_pose(last_pose, cur_pose)
            self.pose = self.add_relative_pose(self.pose, delta)

        if self.odom_skip < 0:
            self.odom_skip = 20
            self.robot_path.poses.append(self.pose_to_StampedPose(cur_pose, data.header.stamp))
        self.odom_skip -= 1

        self.last_odom = data
        self.publish(data.header.stamp)

    def on_gps(self, data):
        '''
            Capture the gps event and update the map transform.
            It's not magic just least squares.
        '''

        if self.last_odom is None:
            return

        odom_frame = self.extract_pose(self.last_odom.pose)
        current_sensor_pose = self.add_relative_pose(odom_frame, self.sensor_offset)

        if    data.position_covariance[0] > self.accuracy_threshold \
           or data.position_covariance[1] > self.accuracy_threshold \
           or data.position_covariance[3] > self.accuracy_threshold \
           or data.position_covariance[4] > self.accuracy_threshold:
            return

        # Calculate the position of the measurement.
        geo_point = geodesy.utm.fromLatLong(data.latitude, data.longitude).toPoint()
        #TODO check valid
        measured_x = geo_point.x
        measured_y = geo_point.y

        self.measurement_queue.append(
            {
                "stamp": self.last_odom.header.stamp,
                "map": [measured_x, measured_y],
                "pose": [current_sensor_pose[0], current_sensor_pose[1], current_sensor_pose[2]]
            })

        # Calculate matching.
        if len(self.measurement_queue) > 2:
            while(len(self.measurement_queue) > self.max_queue_length):
                self.measurement_queue.popleft()

            # Create left and right feature matching array.
            pose_list = []
            gps_list = []
            for x in self.measurement_queue:
                p = self.add_relative_pose(self.delta, x["pose"])
                m = [float(x["map"][0]), float(x["map"][1])]
                pose_list.append([p[0], p[1]])
                gps_list.append(m)

            # Calculate feature transform and apply it on the odometry
            scale, theta, tx, ty, center = self.feature_transform(pose_list, gps_list)
            if scale is None:
                return

            tmp = self.apply_feature_transform_on_pose(theta, tx, ty, center, self.pose)
            for i in range(len(self.pose)):
                self.pose[i] = (1.0 - self.adoption_rate[i]) * self.pose[i] + self.adoption_rate[i] * tmp[i]
        self.publish(self.last_odom.header.stamp)

    def pose_to_StampedPose(self, pose, time, frame=None):
        ps = PoseStamped()
        ps.header.stamp = time
        if frame is None:
            ps.header.frame_id = self.child_frame_id
        else:
            ps.header.frame_id = frame
        ps.pose.position = Point(pose[0], pose[1], 0)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        ps.pose.orientation = Quaternion(*quaternion)
        return ps

    def publish(self, time):
        '''
            Publish the state.
        '''
        msg = Odometry()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = 'base_link'

        odom = self.extract_pose(self.last_odom.pose)

        # Calculate tf for /map -> /odom but seems to be a bit wrong (fix the following two lines)
        self.delta = self.sub_relative_pose(self.pose, odom)
        msg.pose.pose.position = Point(self.delta[0], self.delta[1], 0)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.delta[2])
        msg.pose.pose.orientation = Quaternion(*quaternion)

        pos = (msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z)

        ori = (msg.pose.pose.orientation.x,
               msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z,
               msg.pose.pose.orientation.w)

        # Also publish tf if necessary
        if self.publish_map_tf:
            self.tf_br.sendTransform(pos, ori, msg.header.stamp, self.child_frame_id, self.frame_id)

        msg.pose.pose.position = Point(self.pose[0], self.pose[1], 0)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])
        msg.pose.pose.orientation = Quaternion(*quaternion)

        # Publish odometry message
        self.odom_pub.publish(msg)

        # Publish the path
        self.robot_path.header.frame_id = self.child_frame_id
        self.robot_path.header.stamp = time
        self.robot_path_pub.publish(self.robot_path)

        if len(self.measurement_queue) > 0:
            measurement_path = Path()
            measurement_path.header.frame_id = self.frame_id
            measurement_path.header.stamp = time
            for x in self.measurement_queue:
                map_pos = x["map"]
                measurement_path.poses.append(self.pose_to_StampedPose([map_pos[0], map_pos[1], 0], x["stamp"], self.frame_id))

            self.observation_path_pub.publish(measurement_path)

            measurement_path = Path()
            measurement_path.header.frame_id = self.child_frame_id
            measurement_path.header.stamp = time
            for x in self.measurement_queue:
                p = self.add_relative_pose(self.delta, x["pose"])
                measurement_path.poses.append(self.pose_to_StampedPose(p, x["stamp"]))

            self.map_pose_path_pub.publish(measurement_path)

    def kill(self):
        '''
            Kill the odometry fusion node.
        '''
        pass


def main(args):
    '''
        Main function.
    '''

    rospy.init_node('gps_localization', anonymous=False)
    gpslocalization = GPSLocalization()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    gpslocalization.kill()


if __name__ == '__main__':
    main(sys.argv)
