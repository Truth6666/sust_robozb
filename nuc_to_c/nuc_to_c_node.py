import rclpy
from rclpy.node import Node
import serial
import struct
import math

from sensor_msgs.msg import Imu,Range,JointState

class CBoardDriverNode(Node):
    def __init__(self):
        super().__init__('c_board_driver_node')

        #设置串口设备和波特率
        self.port_name = '/dev/ttyUSB0'
        self.baud_rate = 115200
        try:
            self.ser = serial.Serial(self.port_name,self.baud_rate,timeout=0.01)
            self.get_logger().info("成功连接C板串口")
        except Exception as e:
            self.get_logger().error(f"串口打开失败:{e}")
            raise SystemExit
        
        #imu的发布者和创建者，电机转速的发布者
        self.imu_pub = self.create_publisher(Imu,'c_board/imu',10)
        self.laser_pub = self.create_publisher(Range,'c_board/laser_range',10)
        self.joint_pub = self.create_publisher(JointState,'c_board/wheel_speeds',10)

        # struct 解析格式说明:
        # <  : 小端模式 (STM32 和 x86 默认都是小端)
        # h  : int16 (2字节，对应电机 RPM)
        # f  : float (4字节，对应 IMU 角度)
        # H  : uint16 (2字节，对应激光距离)
        # 总计解包 22 个有效数据字节
        self.unpack_format = '<hhhhfffH'
        self.payload_size = struct.calcsize(self.unpack_format)  #计算有效字节数，正常情况下22字节

        
        self.timer = self.create_timer(0.01,self.read_serial_data)




    def read_serial_data(self):
        #如果缓冲区数据不够一帧(26字节)，直接返回
        if self.ser.in_waiting <26:
            return
        
        try:
            #帧头0x55和0xAA
            if self.ser.read(1) == b'\x55':
                if self.ser.read(1) == b'\xaa':
                    ##帧头正确就把剩下所有数据读完
                    frame_data = self.ser.read(24)

                    payload_bytes = frame_data[:22]
                    received_checksum = frame_data[22]#校验和在第23字节
                    tail = frame_data[23] #帧尾

                    #检查帧尾和校验和
                    if tail == 0x0D:
                        if self.calculate_checksum(b'\x55\xaa' + payload_bytes) == received_checksum:
                            #解包
                            unpacked_data = struct.unpack(self.unpack_format,payload_bytes)
                            self.publish_ros_messages(unpacked_data)
                        else:
                            self.get_logger().warn("校验和错误")
        except Exception as e:
            self.get_logger().error(f"读取异常:{e}")

    def calculate_checksum(self,data_bytes):
        #计算校验和，取低八位(就是对256取模)
        return sum(data_bytes) & 0xFF
    
    def publish_ros_messages(self,data):
        m1,m2,m3,m4,pitch,roll,yaw,laser_dist = data
        current_time = self.get_clock().now().to_msg()
        
        #发布底盘电机转速
        joint_msg = JointState()
        joint_msg.header.stamp = current_time
        joint_msg.name = ['wheel_1_joint','wheel_2_joint','wheel_3_joint','wheel_4_joint']
        joint_msg.velocity = [float(m1),float(m2),float(m3),float(m4)]
        self.joint_pub.publish(joint_msg)
        
        #发布IMU姿态
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu_link"
        qx,qy,qz,qw = self.euler_to_quaternion(roll,pitch,yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        self.imu_pub.publish(imu_msg)
        
        #发布激光距离
        range_msg = Range()
        range_msg.header.stamp = current_time
        range_msg.header.frame_id = "laser_link"  #设置雷达的参考坐标系
        range_msg.radiation_type = Range.INFRARED   
        range_msg.min_range = 0.02
        range_msg.max_range = 2.0
        range_msg.range = laser_dist / 1000.0 #毫米转为米
        self.laser_pub.publish(range_msg)
        
    def euler_to_quaternion(self,roll,pitch,yaw):
        r,p,y = math.radians(roll),math.radians(pitch),math.radians(yaw)
        qx = math.sin(r/2) * math.cos(p/2) * math.cos(y/2) - math.cos(r/2) * math.sin(p/2) * math.sin(y/2)
        qy = math.cos(r/2) * math.sin(p/2) * math.cos(y/2) + math.sin(r/2) * math.cos(p/2) * math.sin(y/2)
        qz = math.cos(r/2) * math.cos(p/2) * math.sin(y/2) - math.sin(r/2) * math.sin(p/2) * math.cos(y/2)
        qw = math.cos(r/2) * math.cos(p/2) * math.cos(y/2) + math.sin(r/2) * math.sin(p/2) * math.sin(y/2)
        return qx,qy,qz,qw
        



def main():
    rclpy.init()
    node = CBoardDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("退出")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
