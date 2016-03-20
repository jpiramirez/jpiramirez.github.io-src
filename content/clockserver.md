Title: Writing a clock server for ROS simulations
Date: 2016-03-20 12:00
Modified: 2015-03-20 12:00
Category: ROS
Tags: robotics, ros, cpp
Slug: ros-clockserver
Authors: Juan-Pablo Ramirez
Summary: How to write a clock server for ROS


One advantage of using [ROS](http://ros.org) is the fact that nodes [can be agnostic to the source of their timing signals](http://wiki.ros.org/roscpp/Overview/Time). The ROS system calls such as `ros::Time::now()` from roscpp will grab the current time stamp from the OS clock or from a simulated `/clock` signal, depending on the availability of `/clock` and on the parameter `/use_sim_time`. Users can force their nodes to query the OS for the current time by using functions from the `ros::WallTime*` types.

A commonly used simulated clock source comes from Gazebo, which will always publish `/clock`. But sometimes Gazebo can consume resources that are better used somewhere else. Besides, it is interesting to write a node whose sole purpose is to publish messages to `/clock`.

The `/clock` topic is of type `rosgraph_msgs/Clock`, and it simply consists of a single time stamp. The time data type is just two integers, one for seconds and another for nanoseconds. Implementing a clock server simply consists of using functions from the `ros::WallTime*` types to read the base time signal, process that signal by multiplying or dividing it by some factor, and publishing the `rosgraph_msgs/Clock` messages that result. More details about clock messages can be found [here](http://wiki.ros.org/Clock).

Here is an example of a clock server.

	:::c++
	// Clock server
	//
	// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
	// The University of Texas at Dallas
	// Sensing, Robotics, Vision, Control and Estimation Lab
	// (SeRViCE) 2012-2016

	#include <ros/ros.h>
	#include <rosgraph_msgs/Clock.h>

	class clockserver
	{
	ros::NodeHandle nh_;
	ros::Publisher clockpub;
	ros::Time ctime;

	double tfactor, realfactor;

	ros::WallTimer timer;

	public:
	clockserver()
	{

		nh_.param("timefactor", tfactor, 1.0);
		if(tfactor < 0.0)
		tfactor = 1.0;
		clockpub = nh_.advertise<rosgraph_msgs::Clock>("/clock", 2);
		ROS_INFO_STREAM("Clock server created");

		nh_.param("tickspersec", realfactor, 250.0);
		timer = nh_.createWallTimer(ros::WallDuration(1.0/realfactor), &clockserver::broadcastTime, this);

		ctime.sec = 0;
		ctime.nsec = 0;
	}

	~clockserver()
	{
	}

	void broadcastTime(const ros::WallTimerEvent& te)
	{
		ros::Duration d(tfactor/realfactor);
		ctime = ctime + d;

		rosgraph_msgs::Clock clockmsg;
		clockmsg.clock.sec = ctime.sec;
		clockmsg.clock.nsec = ctime.nsec;

		clockpub.publish(clockmsg);
	}

	};

	int main(int argc, char** argv)
	{
	ros::init(argc, argv, "clockserver");
	clockserver dr;
	ros::spin();
	return 0;
	}
	
In the code two parameters are read: `timefactor` is the speed-up or slow-down of the simulated clock signal with respect to "wall" time, and `tickspersec` is the frequency with which the simulated clock signal will be publish (relative to wall time). This last parameter is important, because the nodes that use the `/clock` signal may need specific frequencies depending on the application. This is equivalent to the `real_time_update_rate` from Gazebo world files.

