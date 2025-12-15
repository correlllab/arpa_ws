#!/usr/bin/env python3
"""
Timing Parameter Fixer for ROS2 MoveIt in Docker

This script automatically sets appropriate timeout values when move_group starts.
Run this in the background before launching your motion control node.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import time
import sys


class TimingParameterFixer(Node):
    def __init__(self):
        super().__init__('timing_parameter_fixer')
        
        self.get_logger().info('Starting Timing Parameter Fixer...')
        
        # Parameters to fix
        self.target_nodes = {
            '/move_group': {
                'robot_description_planning.joint_state_timeout': 0.5,
                'use_sim_time': False,
            },
            '/robot_state_publisher': {
                'use_sim_time': False,
            }
        }
        
        self.fixed_nodes = set()
        
        # Create a timer to check and fix parameters
        self.timer = self.create_timer(2.0, self.check_and_fix_parameters)
        
    def check_and_fix_parameters(self):
        """Check if target nodes exist and fix their parameters"""
        
        # Get list of nodes
        node_names = self.get_node_names()
        
        for node_name, params in self.target_nodes.items():
            # Skip if already fixed
            if node_name in self.fixed_nodes:
                continue
                
            # Check if node exists (remove leading / for comparison)
            node_name_clean = node_name.lstrip('/')
            if node_name_clean not in node_names:
                continue
            
            self.get_logger().info(f'Found node {node_name}, applying timing fixes...')
            
            # Create parameter client
            param_client = self.create_client(
                rclpy.parameter.srv.SetParameters,
                f'{node_name}/set_parameters'
            )
            
            # Wait for service
            if not param_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'Parameter service for {node_name} not available')
                continue
            
            # Set parameters
            success = True
            for param_name, param_value in params.items():
                if self.set_parameter(param_client, node_name, param_name, param_value):
                    self.get_logger().info(f'✓ Set {node_name}/{param_name} = {param_value}')
                else:
                    self.get_logger().warn(f'✗ Failed to set {node_name}/{param_name}')
                    success = False
            
            if success:
                self.fixed_nodes.add(node_name)
                self.get_logger().info(f'✓ Successfully fixed timing parameters for {node_name}')
        
        # Check if all nodes are fixed
        if len(self.fixed_nodes) == len(self.target_nodes):
            self.get_logger().info('All target nodes fixed! Shutting down fixer.')
            self.timer.cancel()
            rclpy.shutdown()
    
    def set_parameter(self, client, node_name, param_name, param_value):
        """Set a parameter on a remote node"""
        try:
            # Create parameter object
            if isinstance(param_value, bool):
                param = Parameter(param_name, Parameter.Type.BOOL, param_value)
            elif isinstance(param_value, int):
                param = Parameter(param_name, Parameter.Type.INTEGER, param_value)
            elif isinstance(param_value, float):
                param = Parameter(param_name, Parameter.Type.DOUBLE, param_value)
            elif isinstance(param_value, str):
                param = Parameter(param_name, Parameter.Type.STRING, param_value)
            else:
                self.get_logger().error(f'Unsupported parameter type for {param_name}')
                return False
            
            # Create request
            request = rclpy.parameter.srv.SetParameters.Request()
            request.parameters = [param.to_parameter_msg()]
            
            # Call service
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                response = future.result()
                if response and len(response.results) > 0:
                    return response.results[0].successful
            
            return False
            
        except Exception as e:
            self.get_logger().error(f'Exception setting parameter: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        fixer = TimingParameterFixer()
        rclpy.spin(fixer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
