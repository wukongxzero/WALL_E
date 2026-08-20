import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import time 

def main():
    rclpy.init()
    node = Node('camera_tf_check')
    
    # Buffer stores received transforms; listener feeds it in the background
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    # Give the listener time to populate the buffer
    print("Spinning node to collect transforms...")
    t0 = time.time()
    while time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    try:
        # 1. Lookup the transform (Requires 3 arguments in ROS 2)
        t = tf_buffer.lookup_transform(
            'odom', 
            'camera_color_optical_frame', 
            rclpy.time.Time()
        )
        
        # 2. Print the translation translation
        print("Position:", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        
        # 3. Extract quaternion components
        qxyz = np.array([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z])
        w = t.transform.rotation.w
        
        # The optical frame's forward axis is Z-forward (0, 0, 1)
        v = np.array([0, 0, 1]) 
        
        # 4. Perform Quaternion vector rotation formula:
        # v' = v + 2*w*(qxyz x v) + 2*(qxyz x (qxyz x v))
        v_cross = np.cross(qxyz, v)
        v_prime = v + 2 * w * v_cross + 2 * np.cross(qxyz, v_cross)
        
        print("Camera forward direction in odom space:", v_prime)
        
    except Exception as e:
        print("Failed to complete TF logic. Error:", e)
        
    # Clean shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

