To run this, start roscore on one terminal.
On a separate terminal, do rosrun ros_cv_inference ros_cv_laptop
In a separate terminal, do rosrun ros_cv_inference test_subscriber
The test subscriber should get the values from cv_inference

ros_cv_infere.py doesn't work, it attempts to use opencv 2, but we're using opencv 3 
on the jetson now
