
#ifndef TOPIC_NAMES
#define TOPIC_NAMES

#define LEFT_IMAGE_TOPIC "/left_image_raw" 
#define RIGHT_IMAGE_TOPIC "/right_image_raw"
#define LIDAR_RAW_TOPIC "/lidar_raw"
#define ODOM_TOPIC "/odometry_filtered"
#define POINT_CLOUD_SECTION_TOPIC "/point_cloud_requested"
#define POINT_CLOUD_SECTION_REQUEST_TOPIC "/send_point_cloud"
#define CONE_DETECTED_TOPIC "/stereo_cones"
#define CONES_FULL_TOPIC "/mur/slam/cones"
#define CONTROL_MAP_TOPIC "/mur/planner/map"
#define CONTROL_PATH_TOPIC "/mur/planner/path"
#define CONTROL_TRANSITION_TOPIC "/mur/control/transition"
#define CONTROL_OUTPUT_TOPIC "/husky_velocity_control/cmd_vel"

#endif