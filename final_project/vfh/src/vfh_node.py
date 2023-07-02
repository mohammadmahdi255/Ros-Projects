#!/home/mahdi255/.pyenv/shims/python

import math

import numpy as np
from matplotlib import pyplot as plt

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from vfh.srv import angle, angleResponse, angleRequest


class VFH(object):
    def __init__(self):

        self.goal_dir = 32

        # laser variables
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.laser_range = []
        self.laser_angle_min = 0
        self.laser_angle_max = 0
        self.laser_angle_increment = 0

        # odom variables
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot_pose_x = 0
        self.robot_pose_y = 0
        self.robot_yaw = 0

        # cmd_vel variables
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.max_speed = 0.4
        self.max_turn_rate = 0.4
        self.move_cmd = Twist()

        # VFH variables
        self.sector_width = math.radians(5)
        self.sector_count = int(2 * math.pi / self.sector_width)
        self.max_dist = 4.0
        self.s_max = 10  # 10
        self.hist_resolution = 100
        self.hist_max_value = 20
        self.hist_threshold = 4  # 4
        self.smooth_hist = []

        # Initialize the plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([0, self.sector_count])
        self.ax.set_ylim([0, self.hist_max_value])
        self.ax.set_xlabel('Sector')
        self.ax.set_ylabel('Occupancy')
        self.ax.set_title('VFH Histogram')
        self.best_dir = 0.0

        rospy.init_node('vfh_node')
        rospy.Service("/get_angle", angle, self.service_callback)
        self.r = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.run()
            self.r.sleep()

    def service_callback(self, request: angleRequest):
        response = angleResponse()
        self.goal_dir = math.floor(request.dir / self.sector_width)
        response.radians = self.best_dir

        return response

    def laser_callback(self, laser_msg):
        self.laser_range = laser_msg.ranges
        self.laser_angle_min = laser_msg.angle_min
        self.laser_angle_max = laser_msg.angle_max
        self.laser_angle_increment = laser_msg.angle_increment

    def odom_callback(self, odom_msg):
        self.robot_pose_x = odom_msg.pose.pose.position.x
        self.robot_pose_y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def get_polar_histogram(self, a, b, c):
        hist = [0] * self.sector_count
        for i in range(len(self.laser_range)):
            d = self.laser_range[i]
            if d <= self.max_dist:
                y = self.laser_range[i] * math.sin(i * self.laser_angle_increment)
                x = self.laser_range[i] * math.cos(i * self.laser_angle_increment)
                k = int((math.atan2(y, x) + math.pi) / self.sector_width)

                occupancy = c / pow(d, 1) * (a - b * d)
                hist[k] += occupancy

        # hist = np.array(hist) / np.linalg.norm(hist)
        min_v = np.min(hist)
        hist = (np.array(hist) - min_v)
        hist[:10] = 2 * self.hist_threshold
        hist[-9:] = 2 * self.hist_threshold
        return hist

    @staticmethod
    def smoothing_function(hist, length) -> np.ndarray:
        window_size = 2 * length + 1
        weights_left = np.arange(1, length + 1)
        weights_right = np.arange(length, 0, -1)
        weights = np.concatenate((weights_left, [length], weights_right))
        weights = weights / window_size

        padded_hist = np.pad(hist, (length, length), mode='wrap')
        smooth_hist = np.convolve(padded_hist, weights, mode='valid')

        return smooth_hist

    # def get_candidate_section(self, hist):
    #     valley_index = np.where(hist <= self.hist_threshold)[0]
    #     x = self.goal['x'] - self.robot_pose_x
    #     y = self.goal['y'] - self.robot_pose_y
    #
    #     if len(valley_index) == 0:
    #         return -1
    #
    #     theta_target = math.atan2(y, x) - self.robot_yaw
    #
    #     if abs(theta_target) > math.pi:
    #         theta_target += (1 - 2 * sgn(theta_target)) * math.pi
    #     else:
    #         theta_target += math.pi
    #
    #     k_target = math.floor(theta_target / self.sector_width)
    #
    #     # k_target = 32
    #
    #     if hist[k_target] <= self.hist_threshold:
    #         kn = k_target
    #         kf = k_target + 1
    #         for i in range(self.s_max):
    #             if hist[kn] <= self.hist_threshold:
    #                 hist[kn] = 2 * self.hist_threshold
    #                 kn = kn - 1
    #             if hist[kf % len(hist)] <= self.hist_threshold:
    #                 hist[kf % len(hist)] = 2 * self.hist_threshold
    #                 kf = kf + 1
    #
    #             if abs(kf - kn) >= self.s_max:
    #                 self.update_plot(k_target, kf, kn)
    #                 return math.ceil((kf + kn) / 2 + len(hist)) % len(hist)
    #
    #         return self.get_candidate_section(hist)
    #
    #     kf = kn = valley_index[np.argmin(np.abs(valley_index - k_target))]
    #
    #     for i in range(self.s_max):
    #         if hist[kf % len(hist)] > self.hist_threshold:
    #             return self.get_candidate_section(hist)
    #         hist[kf % len(hist)] = 2 * self.hist_threshold
    #         if kn < k_target:
    #             kf -= 1
    #         else:
    #             kf = (kf + 1)
    #
    #     self.update_plot(k_target, kn, kf)
    #     return math.ceil((kf + kn) // 2 + len(hist)) % len(hist)

    def get_candidate_section_argrelextrema(self, hist):
        # valley_index = argrelextrema(hist, np.less_equal)[0]
        # mask = hist[valley_index] <= self.hist_threshold
        # valley_index = valley_index[mask]
        valley_index = np.where(hist <= self.hist_threshold)[0]
        # x = self.goal['x'] - self.robot_pose_x
        # y = self.goal['y'] - self.robot_pose_y

        k_target = self.goal_dir

        if hist[k_target] <= self.hist_threshold:
            kn = k_target
            kf = kn - self.s_max + 1
            weights = [self.get_weight_mean(hist, kf + i, kn + i) for i in range(self.s_max)]
            min_i = np.argmin(weights)
            kn += min_i
            kf += min_i

            if weights[min_i] <= self.hist_threshold:
                self.update_plot(k_target, kn, kf)
                return math.ceil((kf + kn) / 2 + len(hist)) % len(hist)

        # if hist[k_target] <= self.hist_threshold:
        #     kn = k_target
        #     kf = kn - self.s_max + 1
        #     weights = np.array([self.get_weight_mean(hist, kf + i, kn + i) for i in range(self.s_max)])
        #     weights_deviation = np.array([self.get_weight_deviation(hist, kf + i, kn + i) for i in range(self.s_max)])
        #     indices = np.where(weights <= self.hist_threshold)[0]
        #     # rospy.loginfo(weights_deviation[indices])
        #     # indices = np.where(weights_deviation[indices] <= 2.5)[0]
        #     if indices.size > 0:
        #         thetas = np.ceil((kf + kn) / 2 + indices + len(hist)) % len(hist)
        #         min_i = indices[np.argmin(np.abs(k_target - thetas))]
        #         kn += min_i
        #         kf += min_i
        #         self.update_plot(k_target, kn, kf)
        #         return math.ceil((kf + kn) / 2 + len(hist)) % len(hist)

        while len(valley_index) != 0:
            kn = valley_index[np.argmin(np.abs(valley_index - k_target))]
            if kn < k_target:
                kf = kn - self.s_max + 1
                if self.get_weight_mean(hist, kf, kn) <= self.hist_threshold:
                    self.update_plot(k_target, kf, kn)
                    return math.ceil((kf + kn) / 2 + len(hist)) % len(hist)
            else:
                kf = kn + self.s_max - 1
                if self.get_weight_mean(hist, kn, kf) <= self.hist_threshold:
                    self.update_plot(k_target, kn, kf)
                    return math.ceil((kf + kn) / 2 + len(hist)) % len(hist)

            valley_index = np.delete(valley_index, np.where(valley_index == kn))

        return -1

    def get_weight_mean(self, hist, kn, kf):
        indices = np.arange(kn, kf + 1) % len(hist)
        values = hist[indices]
        # if np.where(values >= 1.9 * self.hist_threshold)[0].size > self.s_max * 0.1:
        #     return 2 * self.hist_threshold
        return np.mean(values)

    @staticmethod
    def get_weight_deviation(hist, kn, kf):
        indices = np.arange(kn, kf + 1) % len(hist)
        values = hist[indices]
        mean = np.mean(values)
        variance = np.sqrt(np.mean((values - mean) ** 2))
        return variance

    def get_direction(self, section_theta):
        theta = section_theta * self.sector_width - math.pi
        if theta > math.pi:
            theta -= 2 * math.pi

        return theta

    def update_plot(self, k_target, kn, kf):
        x_range = len(self.smooth_hist)
        x_values = [i for i in range(x_range)]
        self.ax.clear()
        self.ax.set_xlim([0, self.sector_count])
        self.ax.set_ylim([0, self.hist_max_value])
        self.ax.set_xlabel('Sector')
        self.ax.set_ylabel('Occupancy')
        self.ax.set_title('VFH Histogram')
        plt.bar(x_values, self.smooth_hist[::-1], width=1)

        # Add a vertical line
        k_target = x_range - k_target
        theta = x_range - ((kf + kn) / 2 + x_range) % x_range
        kn = x_range - (kn + x_range) % x_range
        kf = x_range - (kf + x_range) % x_range

        self.ax.axvline(x=k_target, color='red')
        self.ax.axvline(x=theta, color='green')
        self.ax.axvline(x=kn, color='black')
        self.ax.axvline(x=kf, color='black')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self):
        hist = self.get_polar_histogram(a=1.0, b=0.25, c=1)
        self.smooth_hist = self.smoothing_function(hist, 2)
        # section_theta = self.get_candidate_section(np.copy(self.smooth_hist))
        section_theta = self.get_candidate_section_argrelextrema(np.copy(self.smooth_hist))
        if section_theta == -1:
            self.cmd_vel_pub.publish(Twist())
            self.best_dir = 0
            return

        self.best_dir = self.get_direction(section_theta)


def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


if __name__ == '__main__':
    vfh = VFH()
