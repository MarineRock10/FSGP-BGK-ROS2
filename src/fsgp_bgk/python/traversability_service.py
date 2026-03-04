#!/usr/bin/env python3
'''
MIT License
Copyright (c) 2025 Senming Tan (senmingtan5@gmail.com)

ROS2 service wrapper for the FSGP-BGK traversability algorithm.
This node exposes the existing TraversabilityAnalyzer as a ROS2 service,
allowing the C++ infrastructure node to call it for core computation.

The algorithm code in f_sgp_bgk.py remains completely unchanged.
'''

import rclpy
from rclpy.node import Node
import numpy as np
from f_sgp_bgk import TraversabilityAnalyzer
from fsgp_bgk_msgs.srv import ComputeTraversability
import yaml
import traceback


class TraversabilityService(Node):
    def __init__(self):
        super().__init__('traversability_service')

        # Load config path parameter
        self.declare_parameter('config_path', '../config/params.yaml')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        self.get_logger().info(f'Loading config from: {config_path}')

        # Initialize the existing algorithm (unchanged)
        try:
            self.analyzer = TraversabilityAnalyzer(config_path=config_path)
            self.get_logger().info('TraversabilityAnalyzer initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize TraversabilityAnalyzer: {e}')
            raise

        # Create the service
        self.srv = self.create_service(
            ComputeTraversability,
            'compute_traversability',
            self.compute_traversability_callback
        )

        self.get_logger().info('TraversabilityService ready, waiting for requests...')

    def compute_traversability_callback(self, request, response):
        '''
        Service callback: receives pose + point cloud, runs the algorithm,
        returns grid traversability results.
        '''
        try:
            # Parse pose
            pose = list(request.pose)

            # Parse point cloud from flat array to Nx3
            num_points = request.num_points
            points_flat = np.array(request.points, dtype=np.float32)

            if len(points_flat) != num_points * 3:
                response.success = False
                response.message = (
                    f'Point count mismatch: expected {num_points * 3} floats, '
                    f'got {len(points_flat)}'
                )
                return response

            local_points = points_flat.reshape(num_points, 3)

            # Call the existing algorithm (unchanged API)
            self.analyzer.update_map(pose, local_points)

            # Extract results
            traversability = self.analyzer.traversability
            grid = self.analyzer.grid
            mean = self.analyzer.mean

            if traversability is None or grid is None or mean is None:
                response.success = False
                response.message = 'Algorithm returned None results'
                return response

            n = len(traversability)

            # Pack grid points as flat array [gx0,gy0, gx1,gy1, ...]
            grid_points = np.column_stack((
                grid[:, 0].astype(np.float32),
                grid[:, 1].astype(np.float32)
            )).flatten().tolist()

            response.grid_points = grid_points
            response.num_grid_points = n
            response.mean_heights = mean.astype(np.float32).flatten().tolist()
            response.traversability = traversability.astype(np.float32).flatten().tolist()
            response.success = True
            response.message = f'Computed traversability for {n} grid cells'

            self.get_logger().debug(
                f'Processed {num_points} points -> {n} grid cells'
            )

        except Exception as e:
            response.success = False
            response.message = f'Exception: {str(e)}'
            self.get_logger().error(
                f'Error in compute_traversability: {e}\n{traceback.format_exc()}'
            )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
