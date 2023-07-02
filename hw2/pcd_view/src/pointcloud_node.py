#!/home/mahdi/.pyenv/shims/python

import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2


class PointCloudNode:
    def __init__(self) -> None:
        rospy.init_node("assemble_scans_to_cloud")
        rospy.wait_for_service("assemble_scans2")
        self.assemble_scans = rospy.ServiceProxy("assemble_scans2", AssembleScans2)
        self.pub = rospy.Publisher("/laser_pointcloud", PointCloud2, queue_size=10)
        self.rate = rospy.Rate(1)
        rospy.sleep(1)

    def run(self):
        while not rospy.is_shutdown():
            resp = self.assemble_scans(rospy.Time(0, 0), rospy.get_rostime())
            print("Got cloud with %u points" % len(resp.cloud.data))
            self.pub.publish(resp.cloud)
            self.rate.sleep()


if __name__ == "__main__":
    PointCloudNode().run()
