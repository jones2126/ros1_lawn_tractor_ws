'''
THis is a code smippet, not an entire program.
This code uses the ros::NodeHandle object nh to read the parameter initial_yaw from 
the launch file using the nh.param method. The initial_yaw value is then passed as an 
argument to the publishImuMessage function along with the IMU data.
'''

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "chip_imu_driver");
  
  //ros::NodeHandle node;
  ros::Time::init();

  /* Node */
  ros::NodeHandle node("imu");
  
  /* publisher */
  ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("data", 1);

  port = "/dev/ttyUSB0";
  baud = 115200;
  ros::NodeHandle nh("~");
  nh.param<std::string>("port", port, port);

  /* Get initial heading from launch file */
  double initial_yaw = 0.0;
  nh.param<double>("initial_yaw", initial_yaw, 0.0);

  /* Open serial connection */
  imu_sensor::IMUSerial serial(port.c_str(), baud);
  if (openConnection(serial)) {
    ROS_INFO("Connection Succesful");
  } else {
    ROS_WARN("Problem connecting to serial device (number of attempts is 100)");
    return 1;
  }

  /* Main cycle */
  ros::Rate r(500); // 500 hz
  while (ros::ok())
  {
    serial.readAndParse();
    if (serial.hasNewData()) {
      publishImuMessage(imu_pub, serial.getData(), initial_yaw);
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

'''
sample launch file:
<launch>
  <node name="chip_imu_driver" pkg="chip_imu_driver" type="imu_serial_node">
    <param name="initial_yaw" type="double" value="1.57"/> <!-- set initial yaw to 90 degrees -->
  </node>
</launch>


'''