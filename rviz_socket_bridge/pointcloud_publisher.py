import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.node import Node
from sensor_msgs_py.point_cloud2 import create_cloud
import numpy as np
import socket
import pickle

class PointCloudPublisher(Node):

    def __init__(self):
        super().__init__('pc_publisher')
        self.pub = self.create_publisher(PointCloud2, 'point_cloud2', 10)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        self.s = socket.socket()
        port = 12348
        self.s.bind(("", port))
        self.s.listen(5)
        


    def listen_and_publish(self):

        while rclpy.ok():
            c, addr = self.s.accept()                  
            print("Socket is listening")

            data_bytes = c.recv(10240)
            points_array = pickle.loads(data_bytes)
            print('Received array from the client: ')
            print(points_array)
            msg_header = Header(frame_id='map')
            point_cloud_msg = create_cloud(msg_header, self.fields, self.convert_points(points_array))
            self.pub.publish(point_cloud_msg)
            print("PointCloud published!")
            c.close()                

#                c, addr = self.s.accept()                  


        self.s.close()            
        dummy_points = self.get_dummy_points()
        msg_header = Header(frame_id='map')
        point_cloud_msg = create_cloud(msg_header, self.fields, dummy_points)

        # Publishing
        self.pub.publish(point_cloud_msg)

    def convert_points(self, points_array):
        # Generate dummy points randomly
        points = []
        for i in range(points_array.shape[0]):
            p = points_array[i]
            x = p[0]
            y = p[1]
            z = p[2]
            r = int(p[3])
            g = int(p[4])
            b = int(p[5])
            rgb = np.array([p[3], p[4], p[5]], dtype=np.uint8)  # Red color
            rgb = np.array((r << 16) | (g << 8 ) | (b << 0),dtype=np.uint32)
            color = rgb.reshape(-1,1)
            color.dtype = np.float32            
#            rgb_uint32 = np.dot(rgb, [1, 256, 256*256])  # RGB to uint32
            points.append([x, y, z, color])
        return points

def main(args=None):
    rclpy.init(args=args)
    pc_publisher = PointCloudPublisher()
    pc_publisher.listen_and_publish()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
