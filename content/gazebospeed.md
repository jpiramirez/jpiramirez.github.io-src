Title: Lowering the computational load from Gazebo
Date: 2015-12-17 19:30
Modified: 2015-12-17 19:30
Category: ROS
Tags: robotics, gazebo
Slug: gazebo-speedup
Authors: Juan-Pablo Ramirez
Summary: Speeding up Gazebo with ROS

The [Gazebo](http://gazebosim.org/) simulator is a great platform for algorithm testing, and it is very easy to use through [ROS](http://ros.org). However, Gazebo is quite precise in its calculations by default, which is sometimes cause for slowdowns. This happens often when attempting to simulate multiple robots at once.

Here are some tips to reduce the load of the simulator when dealing with these situations.

### Reduce the sampling rate and change the ODE solver
It seems that the default settings for Gazebo try to get the simulation running at 1 KHz, which quite frankly is a very fast sampling rate, considering the complexity of some simulation scenarios. One first step to reduce the load from Gazebo would be to reduce this rate by half or even more. It is also possible to choose a different ODE solver. I do not know the specifics of the "quick" solver, but it does seem faster and less accurate.

The aforementioned changes can be done by modifying the .world file for our simulation.

	:::xml
	<physics type='ode'>
	<max_step_size>0.004000</max_step_size>
	<real_time_factor>1.000000</real_time_factor>
	<real_time_update_rate>250.000000</real_time_update_rate>
	<gravity>0.000000 0.000000 -9.800000</gravity>
	<ode>
		<solver>
			<type>quick</type>
			<iters>200</iters>
		</solver>
	</ode>
	</physics>


The real_time_factor tag specifies how fast the simulation should run, with respect to real (wall) time. Keeping it as 1 means that Gazebo will try to run the simulation in "real time", but that is not guaranteed and depends on the system load. 

The tags max_step_size and real_time_update_rate should multiply to real_time_factor. In the code snippet we see that the simulation update rate is now 250 Hz, which should be handled a bit better than 1 KHz. An important caveat: if you are using your own controllers for a robot you will have to adjust your gains to the modified sampling rates.

### Change the sampling rate from the sensors

Some simulated sensors can cause a huge load on the system. Cameras, RGB-D sensors and lidars are examples of this. Sometimes the default configuration files for such sensors will have values that are aimed at providing accuracy or precision. It may be the case that our application can get away with much lower sampling rates, especially if the robot dynamics are "slow". In such cases, we can configure the sensors for slower sampling.

Here is an example from one of the .xacro files I use.

	:::xml
	<xacro:include filename="$(find hector_sensors_description)/urdf/generic_camera.urdf.xacro" />
	<xacro:generic_camera name="downward_cam" parent="base_link" ros_topic="camera/image" cam_info_topic="camera/camera_info" update_rate="10" res_x="640" res_y="480" image_format="R8G8B8" hfov="49.1343426412">


This snippet depends on the ROS package hector_sensors_description. The important attribute here is update_rate, from the xacro:generic_camera tag. The original value was 25 Hz and I lowered it to 10 Hz. Since my simulation is using four of these cameras, changing this parameter smoothed things quite a bit. My application may even let me lower it to 5 Hz, but 10 Hz provides a good balance. 

### Simplify the world as much as possible

While simulations that use digital elevation maps and detailed 3D environments are very pleasing to the eye, they may also be unnecessarily slow. If you can get away with simpler meshes and less textures, you could get nice benefits in execution rates.

If you want to take this notion to the extreme, you can run Gazebo in headless mode and avoid any visualization slowdowns. This can also help if your graphics card is lacking rendering power. Let's look at a snippet from one of my .launch files.

	:::xml
	<include file="$(find gazebo_ros)/launch/empty_world.launch">
	<arg name="world_name" default="$(find service_utd)/worlds/service.world"/>
	<arg name="headless" value="true" />
	<arg name="gui" value="false" />
	</include>


The package service_utd contains my world files. The important part here are the arguments headless and gui. Note that while we enable the headless mode, the GUI has to be turned off like this or it will start anyway.