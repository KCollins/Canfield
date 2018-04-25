#include <ros/ros.h> //ALWAYS need to include this
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <math.h>

#define M_TAU (M_PI * 2)

// some "magic number" global params:
const double Kp = 10.0; //controller gains
const double Kv = 3;
const double dt = 0.01;

// He's champin' for a clampin'!
double clamp(double a, double b, double c) {
	return min(max(a, b), c)
}

// a simple saturation function; provide saturation threshold (`sat_val'), and
// `val' to be saturated
double sat(double val, double sat_val) {
	return clamp(-sat_val, val, +sat_val)
}

double norm_angle(double theta) {
	while(theta > +M_PI) theta -= M_TAU;
	while(theta < -M_PI) theta += M_TAU;
	return theta;
}

double g_pos_cmd = 0.0; // position command input

void posCmdCB(const std_msgs::Float64& pos_cmd_msg) {
	ROS_INFO("received value of pos_cmd is: %f", pos_cmd_msg.data);
	g_pos_cmd = pos_cmd_msg.data;
}

bool test_services() {
	if(!ros::service::exists("/gazebo/apply_joint_effort", true)) {
		ROS_WARN("waiting for apply_joint_effort service");
		return false;
	}
	if(!ros::service::exists("/gazebo/get_joint_properties", true)) {
		ROS_WARN("waiting for /gazebo/get_joint_properties service");
		return false;
	}
	ROS_INFO("services are ready");
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "canfield_joint_controller");
	ros::NodeHandle nh;
	ros::Duration half_sec(0.5);

	// Wait for services
	while(!test_services()) {
		ros::spinOnce();
		half_sec.sleep();
	}

	// Service clients used to frob Gazebo
	ros::ServiceClient set_trq_client =
		nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	ros::ServiceClient get_jnt_state_client =
		nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

	ros::Publisher pos_publisher = nh.advertise<std_msgs::Float64MultiArray>("jnt_pos", 1);
	ros::Publisher vel_publisher = nh.advertise<std_msgs::Float64MultiArray>("jnt_vel", 1);
	ros::Publisher trq_publisher = nh.advertise<std_msgs::Float64MultiArray>("jnt_trq", 1);
	ros::Publisher jnt_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Subscriber pos_cmd_subscriber = nh.subscribe("pos_cmd", 1, posCmdCB); // TODO getting this to work correctly is the only thing left

	// Note that gazebo_msgs::{AJE,GJP} can't frob multiple joints in the same
	// command, but sensor_msgs::JS can report multiple things simultaneously.
	// Hence the need for three of each of two, and one of the third.
	gazebo_msgs::ApplyJointEffort   effort_cmds[3];
	gazebo_msgs::GetJointProperties joint_props_reqs[3];
	sensor_msgs::JointState         joint_state_msg;
	std_msgs::Float64MultiArray     q1_msg, q1dot_msg, trq_msg;

	// position, velocity, error, and output torque, for each joint
	double q1[3], q1dot[3], q1_err[3], trq[3];

	// "joint1".."joint3"
	char joint_names[3][7];

	// Timestep stuff
	ros::Duration duration(dt);
	ros::Rate rate_timer(1 / dt);

	int i;

	joint_state_msg.header.stamp = ros::Time::now();
	for(i = 0; i < 3; ++i) {
		sprintf(joint_names[i], "joint%d", i + 1);

		effort_cmds[i].request.joint_name = joint_names[i];
		effort_cmds[i].request.effort     = 0.0;
		effort_cmds[i].request.duration   = duration;

		joint_props_reqs[i].request.joint_name = joint_names[i];

		// Allocate new values for each joint
		joint_state_msg.name.push_back(joint_names[i]);
		joint_state_msg.position.push_back(0.0);
		joint_state_msg.velocity.push_back(0.0);
	}

	// Logically, this loop should never run, because ROS is pretty much the
	// worst.
	while(ros::ok()) {
		// Set up output messages for this timestep
		q1_msg.data.clear();
		q1dot_msg.data.clear();
		trq_msg.data.clear();
		joint_state_msg.header.stamp = ros::Time::now();

		// Do the PID thing.  Push data into output messages for republishing,
		// while we're at it.
		for(i = 0; i < 3; ++i) {
			// Get position and velocity from Gazebo
			get_jnt_state_client.call(joint_props_reqs[i]);
			q1[i]    = joint_props_reqs[i].response.position[0];
			q1dot[i] = joint_props_reqs[i].response.rate[0];

			// Pack it back into a message to republish
			q1_msg.data.push_back(q1[i]);
			q1dot_msg.data.push_back(q1dot[i]);

			// Pack it into yet another message to republish, because because.
			joint_state_msg.position[i] = q1[i];
			joint_state_msg.velocity[i] = q1dot[i];

			// Calculate the error, and turn it into an output torque for
			// Gazebo
			q1_err[i] = norm_angle(g_pos_cmd - q1[i]);
			trq[i]    = Kp*q1_err[i] - Kv*q1dot[i];
			trq_msg.data.push_back(trq[i]);
		}

		// Send it all off at the same time
		pos_publisher.publish(q1_msg);    // jnt_pos
		vel_publisher.publish(q1dot_msg); // jnt_vel
		trq_publisher.publish(trq_msg);   // jnt_trq
		jnt_publisher.publish(joint_state_msg);

		// Send each joint's desired effort to Gazebo
		for(i = 0; i < 3; ++i) {
			effort_cmds[i].request.effort = trq[i];
			set_trq_client.call(effort_cmds[i]);

			// Make sure the service call was successful
			if(!effort_cmds[i].response.success)
				ROS_WARN("service call to apply_joint_effort failed!");
		}

		// Begrudgingly give control back to ROS
		ros::spinOnce();
		rate_timer.sleep();
	}
}
