#!/usr/bin/env python

#
# Copyright (c) 2018, Willow Garage, Inc.
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

import numpy

from carla_ros_bridge.sensor import Sensor, create_cloud

from sensor_msgs.msg import PointCloud2, PointField


class Lidar(Sensor):

    """
    Actor implementation details for lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Lidar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

        self.lidar_publisher = node.new_publisher(PointCloud2,
                                                  self.get_topic_prefix(),
                                                  qos_profile=10)
        self.listen()

    def destroy(self):
        super(Lidar, self).destroy()
        self.node.destroy_publisher(self.lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        # we take the opposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data[:, 1] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data)
        self.lidar_publisher.publish(point_cloud_msg)


class SemanticLidar(Sensor):

    """
    Actor implementation details for semantic lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(SemanticLidar, self).__init__(uid=uid,
                                            name=name,
                                            parent=parent,
                                            relative_spawn_pose=relative_spawn_pose,
                                            node=node,
                                            carla_actor=carla_actor,
                                            synchronous_mode=synchronous_mode)

        self.semantic_lidar_publisher = node.new_publisher(
            PointCloud2,
            self.get_topic_prefix(),
            qos_profile=10)
        self.listen()

    def destroy(self):
        super(SemanticLidar, self).destroy()
        self.node.destroy_publisher(self.semantic_lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform a received semantic lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla semantic lidar measurement object
        :type carla_lidar_measurement: carla.SemanticLidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='CosAngle', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ObjIdx', offset=16, datatype=PointField.UINT32, count=1),
            PointField(name='ObjTag', offset=20, datatype=PointField.UINT32, count=1)
        ]

        lidar_data = numpy.fromstring(bytes(carla_lidar_measurement.raw_data),
                                      dtype=numpy.dtype([
                                          ('x', numpy.float32),
                                          ('y', numpy.float32),
                                          ('z', numpy.float32),
                                          ('CosAngle', numpy.float32),
                                          ('ObjIdx', numpy.uint32),
                                          ('ObjTag', numpy.uint32)
                                      ]))

        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data.tolist())
        self.semantic_lidar_publisher.publish(point_cloud_msg)

class FMCWLidar(Sensor):

    """
    Actor implementation details for FMCW lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(FMCWLidar, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)

        self.fmcw_lidar_publisher = node.new_publisher(
            PointCloud2,
            self.get_topic_prefix(),
            qos_profile=10)
        self.listen()

    def destroy(self):
        super(FMCWLidar, self).destroy()
        self.node.destroy_publisher(self.fmcw_lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform a received FMCW lidar measurement into a ROS point cloud message

        FMCW (Frequency-Modulated Continuous-Wave) lidars provide additional information
        compared to standard lidars, including velocity measurements and enhanced
        object detection capabilities.

        :param carla_lidar_measurement: carla FMCW lidar measurement object
        :type carla_lidar_measurement: carla.FMCWLidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)

        # Define the data structure for FMCW lidar raw data
        # This matches the structure provided by Carla's FMCW lidar sensor
        raw_dtype = numpy.dtype([
            ('azimuth',    numpy.float32),    # Horizontal angle in degrees
            ('elevation',  numpy.float32),    # Vertical angle in degrees
            ('range',      numpy.float32),    # Distance in meters
            ('intensity',  numpy.float32),    # Reflection intensity
            ('velocity',   numpy.float32),    # Radial velocity in m/s
            ('cos_angle',  numpy.float32),    # Cosine of incident angle
            ('object_idx', numpy.uint32),     # Object index for segmentation
            ('object_tag', numpy.uint32),     # Object tag/class
            ('point_idx',  numpy.uint32),     # Point index within the measurement
            ('beam_idx',   numpy.uint8),      # Beam index
            ('valid',      numpy.uint8),      # Validity flag
            ('dynamic',    numpy.uint8)       # Dynamic object flag
        ], align=False)

        # Parse raw data from the FMCW lidar measurement
        raw_data = numpy.frombuffer(carla_lidar_measurement.raw_data,
                                    dtype=raw_dtype)

        # Early return if no data received
        if len(raw_data) == 0:
            return

        # Extract relevant fields
        azimuth = raw_data['azimuth']
        elevation = raw_data['elevation']
        range_data = raw_data['range']
        intensity = raw_data['intensity']
        velocity = raw_data['velocity']
        valid = raw_data['valid']

        # Filter out invalid points
        valid_mask = valid.astype(bool)
        azimuth = azimuth[valid_mask]
        elevation = elevation[valid_mask]
        range_data = range_data[valid_mask]
        intensity = intensity[valid_mask]
        velocity = velocity[valid_mask]
        valid = valid[valid_mask]

        # Convert spherical coordinates to Cartesian coordinates
        # Note: Carla uses left-handed coordinate system, ROS uses right-handed
        # Azimuth: negative to convert coordinate system
        # Elevation: add 90 degrees to convert from elevation to polar angle
        azimuth_rad = numpy.deg2rad(-azimuth)
        elevation_rad = numpy.deg2rad(90 + elevation)

        # Spherical to Cartesian conversion
        x = range_data * numpy.sin(elevation_rad) * numpy.cos(azimuth_rad)
        y = range_data * numpy.sin(elevation_rad) * numpy.sin(azimuth_rad)
        z = -range_data * numpy.cos(elevation_rad)

        # Define point cloud fields including FMCW-specific data
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='azimuth', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='elevation', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='range', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=28, datatype=PointField.FLOAT32, count=1),
            PointField(name='valid', offset=32, datatype=PointField.FLOAT32, count=1)
        ]

        # Prepare data array for point cloud creation
        point_cloud_data = numpy.column_stack([
            x, y, z,
            azimuth, elevation, range_data,
            intensity, velocity, valid.astype(numpy.float32)
        ])

        # Create and publish the point cloud message
        point_cloud_msg = create_cloud(header, fields, point_cloud_data.tolist())
        self.fmcw_lidar_publisher.publish(point_cloud_msg)