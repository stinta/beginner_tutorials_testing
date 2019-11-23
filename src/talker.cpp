/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file talker.cpp
 * @author Sandra Tinta
 * @copyright 2019 BSD License
 *
 * @brief Publisher and Server Node
 **/

#include <sstream>
#include <stdexcept>
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials_testing/updateStr.h"

/* variable global to the program so that callback function has access to it */
extern std::string customStr = "No custom string provided as an argument";

/**
 * @brief Function implemented as a helper for updating string via a service
 * 
 * @param request is a request for the updateStr service
 * @param response is a response for the updateStr service
 * 
 * @return true
 */

bool updateStr(beginner_tutorials_testing::updateStr::Request &req,
                beginner_tutorials_testing::updateStr::Response &res) {
  ROS_DEBUG_STREAM("Starting to execute updateStr service method");
  res.lclStr = req.lclStr;
  customStr = req.lclStr;
  ROS_DEBUG_STREAM("Assigned request string to local variable customStr");
  if (customStr.empty()) {
    ROS_WARN_STREAM("request string provided was empty");
  }
  return true;
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  auto n = ros::NodeHandle();
  /**
   * TF variables
   */
  tf::TransformBroadcaster br;
  tf::Transform transform;

  double rate = 10.0;
  std::string::size_type ssz;
  if (argc >= 2) {
    customStr = std::string(argv[1]);
    try {
      rate = std::stod(argv[2], &ssz);
    }
    catch (const std::invalid_argument& ia) {
      ROS_FATAL_STREAM("could not convert to float; invalid argument");
      ROS_INFO_STREAM("setting rate to default value");
      rate = 0.5;
    }
  }

    if (argc < 1) {
      ROS_WARN_STREAM("Empty Argument provided"
               "setting to defaults"
               "is EMPTY setting to default ");
      customStr = "EMPTY string provided";
      rate = 10.0;
    }

  ros::ServiceServer service = n.advertiseService("updateStr", updateStr);
  ROS_INFO_STREAM("Ready to update string.");

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(rate);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  auto count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    auto msg = std_msgs::String();

    auto ss = std::stringstream();
    ss << customStr << " "<< count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    transform.setOrigin(tf::Vector3(0.1, 0.2, 0.5));
    tf::Quaternion q;
    q.setRPY(0.5, 0.5, 0.5);
    transform.setRotation(q);
    br.sendTransform(
         tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
