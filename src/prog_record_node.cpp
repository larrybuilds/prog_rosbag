/** prog_record_node.cpp - Implements programmatic control of rosbag
  *
  *  Author: Lawrence Papincak
  *  Carnegie Mellon University
  *
  */

#include <ros/ros.h>
#include <prog_rosbag/recorder.h>
#include <std_msgs/UInt8.h>
#include <fstream>

static bool shouldStartBag = false;

void extractTopicNames(rosbag::RecorderOptions* opts, std::string const* topic_name_file) {

    std::ifstream fd(topic_name_file->c_str());
    std::string line;

    if( !fd ) {
        ROS_ERROR("Topic input file name invalid, recording everything");
        opts->record_all = true;
    } else {
        while( std::getline(fd,line) ) {
            opts->topics.push_back(line);
        }
    }
}

void startCallback(const std_msgs::UInt8::ConstPtr& msg) {
    shouldStartBag = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "prog_record", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string topic_name_file, data_dir, file_prefix;

    nh_private.param<std::string>("topic_name_file",topic_name_file,"");
    nh_private.param<std::string>("data_directory",data_dir,"");
    nh_private.param<std::string>("file_prefix",file_prefix,"");

    rosbag::RecorderOptions opts;

    // Extract topic names, if there are any
    if( topic_name_file == "" ) {
        ROS_WARN("Recieved no topic names, assuming recording all.");
        opts.record_all = true;
    } else {
        // Parse out topic names
        extractTopicNames(&opts,&topic_name_file);

        // Sanity check in case topic format is wrong
        if( opts.topics.empty() ) {
            opts.record_all = true;
        }
    }

    // Handle file directory/prefix parameters
    if( data_dir == "" ) {
        ROS_WARN("Recieved no data base directory, using ~/.ros");
    } else {
        opts.prefix = data_dir + "/";
    }

    if( file_prefix == "" ) {
        ROS_WARN("Recieved no file name prefix, ignoring.");
    } else {
        opts.prefix += file_prefix;
    }

    ROS_INFO("Saving bag to %s",opts.prefix.c_str());

    // Setup subscribers
    ros::Subscriber start_sub = nh.subscribe("/record/start", 1, startCallback);

    ros::Rate loop_rate(10);

    // Run the recorder
    rosbag::Recorder recorder(opts);
    int result;

    // Run while master is still running
    while( ros::ok() ) {

        // Handle starting of bag recording
        if( shouldStartBag ) {
            ROS_INFO("Starting bag recorder...");
            result = recorder.run();
            shouldStartBag = false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 1;
}
