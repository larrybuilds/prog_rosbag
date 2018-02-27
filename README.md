# prog_rosbag
Provides programmatic control over rosbag recording via C++ interface.

## File Structure

config: contains topic configuration file with newline delimited full topic names to be recorded
include/prog_rosbag: altered headers from rosbag source code
launch: example launch file that shows how to launch the record node
src: programmatic rosbag source files

## Specifying Recorded Topics

The topic names that should be recorded are saved in a config file in the config folder. Each line of the file must contain a single topic with a single newline character at the end.

## Controlling the recording

The node requires specification of a start and stop topic. Any message published to these topics is interpreted as a indication that the bag should be start/stopped.
