# Task 1

***Try to solve this task on your own and do not look at the solution until you have completed this task!***

Create a launch file that spawns a single MiR100 robot platform in the "empty_world" map. Than write a node that moves the robot 1 m in x direction with C++ or Python.

### Hints

- Create a publisher and subscriber in one node to receive/send the messages from/to the robot. 
Use the WritingPublisherSubscriber ROS examples for [C++](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 "WritingPublisherSubscriber(c++)") or [Python](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 "WritingPublisherSubscriber(python)")
- To find out which one to use, use the `rostopic list` command
- Use `rostopic type "topic"` to see the data type of a topic
- Set up your CMakeLists.txt correctly
- If you want to test your programm you have to start your `.launch` file first
