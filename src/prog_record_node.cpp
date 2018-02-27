/** prog_record_node.cpp - Implements programmatic control of rosbag
  *
  *  Author: Lawrence Papincak
  *  Carnegie Mellon University
  *
  */

#include <ros/ros.h>
#include <prog_rosbag/pt_recorder.h>
#include <std_msgs/UInt8.h>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>

static bool shouldStartBag = false;
static bool shouldStopBag = false;

void extractTopicNames(PRecorderOptions* opts, std::string const* topic_name_file) {

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

void stopCallback(const std_msgs::UInt8::ConstPtr& msg) {
    shouldStopBag = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "prog_record", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string topic_name_file, data_dir, file_prefix, start_topic, stop_topic;

    nh_private.param<std::string>("topic_name_file",topic_name_file,"");
    nh_private.param<std::string>("data_directory",data_dir,"");
    nh_private.param<std::string>("file_prefix",file_prefix,"");
    nh_private.param<std::string>("start_topic",start_topic,"");
    nh_private.param<std::string>("stop_topic",stop_topic,"");

    // Make sure the start topic was provided
    if( start_topic == "" ) {
        ROS_WARN("No start topic name specified, aborting data recorder!");
        return -1;
    }

    if( stop_topic == "" ) {
        ROS_WARN("No stop topic name specified, aborting data recorder!");
        return -1;
    }

    // Handle the output directory already existing
    boost::filesystem::path out_dir(data_dir.c_str());
    // Check for existance
    if(!boost::filesystem::exists(out_dir)) {
       ROS_WARN("Output directory doesnt exists, creating it.");
       boost::filesystem::create_directory(out_dir);
    }

    // Create rosbag options
    PRecorderOptions opts;

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
    ros::Subscriber start_sub = nh.subscribe(start_topic, 1, startCallback);
    ros::Subscriber stop_sub  = nh.subscribe(stop_topic, 1, stopCallback);

    ros::Rate loop_rate(10);

    // Run the recorder
    PRecorder recorder(opts);
    int result;

    // Run while master is still running
    while( ros::ok() ) {

        // Handle starting of bag recording
        if( shouldStartBag ) {
            ROS_INFO("Starting bag for topics found in %s", topic_name_file.c_str());
            result = recorder.run();
            shouldStartBag = false;
        }
        if( shouldStopBag ) {
            recorder.stop();
            shouldStopBag = false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 1;
}
