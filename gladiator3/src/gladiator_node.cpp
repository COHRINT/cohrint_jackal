#include "ros/ros.h"
#include "gladiator.h"
#include "ThreadedObject.h"
#include "std_msgs/String.h"
#include "icd_imu.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>

//Data is reported in fixed point, adjust for reporting in floating point
#define GYRO_TO_DEG_S (0.001f) // Gyro X,Y,Z
#define GYRO_TO_RAD_S (1.745329e-5f) // 0.001 * PI / 180.0
#define ACCL_TO_G (0.0001f) // Accel X,Y,Z
#define ACCL_TO_M_S2 (9.81e-4f) // 0.0001 * 9.81

int main(int argc, char **argv)
{
  ros::init(argc,argv,"gladiator_node");
  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu",1000);
  
  GladiatorIMU *drv = new GladiatorIMU("/dev/gladiator"); // udev rule
  drv->start();
  
    while (ros::ok())
      {
	imu_data_t* rawIMU = (imu_data_t*) drv->WaitData(); // Threaded Object
	//Translate the imu packet from the fixed point to the floating point
	 float accel_x = rawIMU->accel_x*ACCL_TO_M_S2;
	 float accel_y = rawIMU->accel_y*ACCL_TO_M_S2;
	 float accel_z = rawIMU->accel_z*ACCL_TO_M_S2;
	 
	 float gyro_x = rawIMU->gyro_x*GYRO_TO_RAD_S;
	 float gyro_y = rawIMU->gyro_y*GYRO_TO_RAD_S;
	 float gyro_z = rawIMU->gyro_z*GYRO_TO_RAD_S;
	 double halStamp = rawIMU->tv_sec + ((double)rawIMU->tv_nsec)/1e9;

	 sensor_msgs::Imu imu;
	 std_msgs::Header header;
	 geometry_msgs::Quaternion orient;
	 geometry_msgs::Vector3 lin_acc;
	 geometry_msgs::Vector3 ang_acc;
	 
	 // add covariances from data sheet
	 
	 ros::spinOnce();
      }
      return 0;
}
