#include <ros/ros.h>
#include <ros/console.h>
#include "gladiator.h"
#include "ThreadedObject.h"
#include "std_msgs/String.h"
#include "icd_imu.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>
#include <ctime>

//Data is reported in fixed point, adjust for reporting in floating point
#define GYRO_TO_DEG_S (0.001f) // Gyro X,Y,Z
#define GYRO_TO_RAD_S (1.745329e-5f) // 0.001 * PI / 180.0
#define ACCL_TO_G (0.0001f) // Accel X,Y,Z
#define ACCL_TO_M_S2 (9.81e-4f) // 0.0001 * 9.81

int main(int argc, char **argv)
{
  ros::init(argc,argv,"gladiator_node");
  ros::NodeHandle n("~");
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu",1000);
  ROS_INFO("Initializing Gladiator IMU");
  unsigned int seq = 1;
  ros::Time now;
  string portName;
  int32_t baud;
  
  n.param("port", portName, string("/dev/gladiator"));
  n.param("baud", baud, (int32_t) 921600);
  
  ROS_INFO("Using port: %s at %d", portName.c_str(), baud);
  GladiatorIMU *drv = new GladiatorIMU(portName.c_str(), baud);
  drv->start();
  
  gladiator_cmd_t factoryReloadCmd(drv, CMD_RELOAD_FACTORY, 0);
  factoryReloadCmd.submit();

  /*
  gladiator_cmd_t setSpeedCmd(drv, CMD_SETRATE, 500);
  setSpeedCmd.submit();
  */
  
  /*
  gladiator_cmd_t testModeCmd(drv, CMD_TESTMODE, 0);
  testModeCmd.submit();

  gladiator_cmd_t getCoefCmd(drv, CMD_GETCOEFFS, 0);
  getCoefCmd.submit();
  ROS_INFO("Coefficient dump complete");
  */
  
  while (ros::ok())
      {
	//  	ROS_INFO("Top of Loop");
  	imu_data_t* rawIMU = (imu_data_t*) drv->WaitData(); // Threaded Object
	if (!rawIMU)
	  break;
	
  	//Translate the imu packet from the fixed point to the floating point
  	 float accel_x = rawIMU->accel_x*ACCL_TO_M_S2;
  	 float accel_y = rawIMU->accel_y*ACCL_TO_M_S2;
  	 float accel_z = rawIMU->accel_z*ACCL_TO_M_S2;
	 // ROS_INFO_STREAM("Accel_x: " << accel_x);
	 // ROS_INFO_STREAM("Accel_y: " << accel_y);
	 // ROS_INFO_STREAM("Accel_z: " << accel_z);
	 
  	 float gyro_x = rawIMU->gyro_x*GYRO_TO_RAD_S;
  	 float gyro_y = rawIMU->gyro_y*GYRO_TO_RAD_S;
  	 float gyro_z = rawIMU->gyro_z*GYRO_TO_RAD_S;
  	 double halStamp = rawIMU->tv_sec + ((double)rawIMU->tv_nsec)/1e9;

	 
	 // MAKE THE MESSAGE
  	 sensor_msgs::Imu imu;
	 
	 // Make header
	 imu.header.seq = seq;
	 imu.header.stamp = ros::Time::now();
	 imu.header.frame_id = "Gladiator IMU";
	 //	 ROS_INFO_STREAM("Time: " << imu.header.stamp);

	 // Quaternions
	 imu.orientation.x = 0;
	 imu.orientation.y = 0;
	 imu.orientation.z = 0;
	 imu.orientation.w = 1.0;
	 imu.orientation_covariance = {0, 0, 0,
				       0, 0, 0,
				       0, 0, 0};

	 // Linear Acceleration
	 imu.linear_acceleration.x = accel_x;
	 imu.linear_acceleration.y = accel_y;
	 imu.linear_acceleration.z = accel_z;
	 imu.linear_acceleration_covariance = {0, 0, 0,
					       0, 0, 0,
					       0, 0, 0};
	 // Angular Velocity
	 imu.angular_velocity.x = gyro_x;
	 imu.angular_velocity.y = gyro_y;
	 imu.angular_velocity.z = gyro_z;
	 imu.angular_velocity_covariance = {0, 0, 0,
					    0, 0, 0,
					    0, 0, 0};

	 imu_pub.publish(imu);
	 seq++;
  	 ros::spinOnce();
      }

      return 0;
}
