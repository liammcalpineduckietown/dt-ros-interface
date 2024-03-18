#!/usr/bin/env python3

import asyncio

import rospy
from geometry_msgs.msg import Vector3
# from hardware_test_imu import HardwareTestIMU
from sensor_msgs.msg import (
    Imu as ROSImu,
    Temperature as ROSTemperature
)

from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown.dtros import DTROS, NodeType
from duckietown_messages.standard.dictionary import Dictionary


class IMUNode(DTROS):
    def __init__(self):
        # Node Init
        super(IMUNode, self).__init__(node_name="imu_node", node_type=NodeType.DRIVER)
        self._robot_name = get_robot_name()
        # publishers initialization
        self.pub_imu = rospy.Publisher('~data', ROSImu, queue_size=10)
        self.pub_therm = rospy.Publisher('~temperature', ROSTemperature, queue_size=10)
        # user hardware test
        # self._hardware_test = HardwareTestIMU()
        # ---
        self.loginfo("Initialized.")

    async def publish(self, data: RawData):
        # TODO: only publish if somebody is listening
        # decode data
        imu: Dictionary = Dictionary.from_rawdata(data)
        # create IMU message
        imu_msg: ROSImu = ROSImu(
            header=rospy.Header(
                # TODO: reuse the timestamp from the incoming message
                stamp=rospy.Time.now(),
                frame_id=imu.header.frame,
            ),
            linear_acceleration=Vector3(*imu.data["linear_accelerations"]),
            angular_velocity=Vector3(*imu.data["angular_velocities"]),
        )
        # create temperature message
        therm_msg: ROSTemperature = ROSTemperature(
            header=rospy.Header(
                # TODO: reuse the timestamp from the incoming message
                stamp=rospy.Time.now(),
                frame_id=imu.header.frame,
            ),
            temperature=imu.data["temperature"],
        )
        # publish messages
        self.pub_imu.publish(imu_msg)
        self.pub_therm.publish(therm_msg)

    async def worker(self):
        # create switchboard context
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # IMU queue
        imu = await (switchboard / "sensors" / "imu" / "raw").until_ready()
        # subscribe
        await imu.subscribe(self.publish)
        # ---
        await self.join()

    async def join(self):
        while not self.is_shutdown:
            await asyncio.sleep(1)

    def spin(self):
        asyncio.run(self.worker())


if __name__ == "__main__":
    # initialize the node
    node = IMUNode()
    # keep the node alive
    node.spin()
