#include <murom_node/murom_ros.h>
#include <tf/transform_datatypes.h>

namespace murom
{
  MuromRos::MuromRos(std::string& node_name)
  {
    this->name = node_name;
  }

  MuromRos::~MuromRos()
  {
  }

  bool MuromRos::init(ros::NodeHandle& nh)
  {
    murom.init(nh);

    // initialize publishers
    advertiseTopics(nh);

    // initialize subscribers
    subscribeTopics(nh);

    publishVersionInfoOnce(); 

    this->prev_update_time = ros::Time::now();
    return true;
  }


  void MuromRos::advertiseTopics(ros::NodeHandle& nh)
  {
    // turtlebot required
    this->publisher["joint_states"]  = nh.advertise<sensor_msgs::JointState>("joint_states",100);
// в данный топик писать
    // kobuki esoterics
    this->publisher["version_info"] = nh.advertise<kobuki_msgs::VersionInfo>("version_info",100,true);

    // odometry
    this->publisher["odom"] = nh.advertise<nav_msgs::Odometry>("odom",100);


    /*
    // event publishers
    std::string evt = "events/";
    event_publisher["button"]         = nh.advertise<kobuki_msgs::SensorState>        (evt + "button",        100);
    event_publisher["bumper"]         = nh.advertise<kobuki_msgs::BumperEvent>        (evt + "bumper",        100);
    event_publisher["cliff"]          = nh.advertise<kobuki_msgs::CliffEvent>         (evt + "cliff",         100);
    event_publisher["wheel_drop"]     = nh.advertise<kobuki_msgs::WheelDropEvent>     (evt + "wheel_drop",    100);
    event_publisher["power_system"]   = nh.advertise<kobuki_msgs::PowerSystemEvent>   (evt + "power_system",  100);
    event_publisher["digital_input"]  = nh.advertise<kobuki_msgs::DigitalInputEvent>  (evt + "digital_input", 100);
    event_publisher["robot_state"]    = nh.advertise<kobuki_msgs::RobotStateEvent>    (evt + "robot_state",   100,true); // latched

  // sensor publishers
    std::string sen = "sensors/";
    sensor_publisher["core"]  = nh.advertise<kobuki_msgs::SensorState> (sen + "core", 100); 
    sensor_publisher["dock_ir"]  = nh.advertise<kobuki_msgs::SensorState> (sen + "dock_ir", 100); 
    sensor_publisher["imu_data"]  = nh.advertise<kobuki_msgs::SensorState> (sen + "imu_data", 100); 
    */
  }

  void MuromRos::subscribeTopics(ros::NodeHandle& nh)
  {
    std::string cmd = "commands/";
    this->subscriber["velocity"] = nh.subscribe(cmd + "velocity", 10, &MuromRos::subscribeVelocityCommand, this);
    this->subscriber["motor_power"] = nh.subscribe(cmd + "motor_power", 10, &MuromRos::subscribeMotorPowerCommand,this);
  }

  void MuromRos::publishVersionInfoOnce()
  {
    this->publisher["version_info"].publish(this->murom.versioninfo);
  }

  void MuromRos::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
  {
    this->last_cmd_vel_time = ros::Time::now();
    this->murom.wheel_speed_cmd[LEFT]  = msg->linear.x - msg->angular.z * this->murom.wheel_separation / 2;
    this->murom.wheel_speed_cmd[RIGHT] = msg->linear.x + msg->angular.z * this->murom.wheel_separation / 2;
      //Сделать отправку скорости в муром
  }

  void MuromRos::subscribeMotorPowerCommand(const kobuki_msgs::MotorPowerConstPtr msg)
  {
    if((msg->state == kobuki_msgs::MotorPower::ON) && (!this->murom.motor_enabled))
    {
      this->murom.motor_enabled = true;
      ROS_INFO_STREAM("Motors fire up. [" << this->name << "]");
    }
    else if((msg->state == kobuki_msgs::MotorPower::OFF) && (this->murom.motor_enabled))
    {
      this->murom.motor_enabled = false;
      ROS_INFO_STREAM("Motors take a break. [" << this->name << "]");
    }
  }

  void MuromRos::updateJoint(unsigned int index,double& w,ros::Duration step_time)
  {
    double v; 
    v = this->murom.wheel_speed_cmd[index];
    w = v / (this->murom.wheel_diameter / 2);
    this->murom.joint_states.velocity[index] = w;
    this->murom.joint_states.position[index]= this->murom.joint_states.position[index] + w * step_time.toSec();
  }

  void MuromRos::updateOdometry(double w_left,double w_right,ros::Duration step_time)
  {
    double d1,d2;
    double dr,da;
    d1 = d2 = 0;
    dr = da = 0;

    d1 = step_time.toSec() * (this->murom.wheel_diameter / 2) * w_left;
    d2 = step_time.toSec() * (this->murom.wheel_diameter / 2) * w_right;

    if(isnan(d1))
    {
      d1 = 0;
    }
    if(isnan(d2))
    {
      d2 = 0;
    }

    dr = (d1 + d2) / 2;
    da = (d2 - d1) / this->murom.wheel_separation;

    // compute odometric pose
    this->murom.odom_pose[0] += dr * cos(this->murom.odom_pose[2]);
    this->murom.odom_pose[1] += dr * sin(this->murom.odom_pose[2]);
    this->murom.odom_pose[2] += da;

    // compute odometric instantaneouse velocity
    this->murom.odom_vel[0] = dr / step_time.toSec();
    this->murom.odom_vel[1] = 0.0;
    this->murom.odom_vel[2] = da / step_time.toSec();

    this->murom.odom.pose.pose.position.x = this->murom.odom_pose[0];
    this->murom.odom.pose.pose.position.y = this->murom.odom_pose[1];
    this->murom.odom.pose.pose.position.z = 0;
    this->murom.odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->murom.odom_pose[2]);

    // We should update the twist of the odometry
    this->murom.odom.twist.twist.linear.x = this->murom.odom_vel[0];
    this->murom.odom.twist.twist.angular.z = this->murom.odom_vel[2];
  }

  void MuromRos::updateTF(geometry_msgs::TransformStamped& odom_tf)
  {
    odom_tf.header = this->murom.odom.header;
    odom_tf.child_frame_id = this->murom.odom.child_frame_id;
    odom_tf.transform.translation.x = this->murom.odom.pose.pose.position.x;
    odom_tf.transform.translation.y = this->murom.odom.pose.pose.position.y;
    odom_tf.transform.translation.z = this->murom.odom.pose.pose.position.z;
    odom_tf.transform.rotation = this->murom.odom.pose.pose.orientation;
  }


  bool MuromRos::update()
  {
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - this->prev_update_time;
    this->prev_update_time = time_now;

    // zero-ing after timeout
    if(((time_now - this->last_cmd_vel_time).toSec() > this->murom.cmd_vel_timeout) || !this->murom.motor_enabled)
    {
      this->murom.wheel_speed_cmd[LEFT] = 0.0;
      this->murom.wheel_speed_cmd[RIGHT] = 0.0;
    }

    // joint_states
    double w_left,w_right;
    updateJoint(LEFT,w_left,step_time);
    updateJoint(RIGHT,w_right,step_time);
    this->murom.joint_states.header.stamp = time_now;
    this->publisher["joint_states"].publish(this->murom.joint_states);

    // odom
    updateOdometry(w_left,w_right,step_time);
    this->murom.odom.header.stamp = time_now;
    this->publisher["odom"].publish(this->murom.odom);
    
    // tf
    geometry_msgs::TransformStamped odom_tf;
    updateTF(odom_tf);
    this->tf_broadcaster.sendTransform(odom_tf);

    return true;
  }
}
