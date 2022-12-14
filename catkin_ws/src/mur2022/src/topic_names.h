
#ifndef TOPIC_NAMES_MUR
#define TOPIC_NAMES_MUR

#define LEFT_IMAGE_TOPIC "/left_image_raw" 
#define RIGHT_IMAGE_TOPIC "/right_image_raw"

#define LIDAR_RAW_TOPIC "/lidar_raw"
#define LEGO_LOAM_POSE_TOPIC "/integrated_to_init"

#define CONTROL_ODOM_TOPIC "/control_odom"
#define POINT_CLOUD_SECTION_TOPIC "/point_cloud_requested"
#define POINT_CLOUD_SECTION_REQUEST_TOPIC "/send_point_cloud"
#define CONE_DETECTED_TOPIC "/stereo_cones"
#define CONES_FULL_TOPIC "/mur/slam/cones"
#define CONTROL_MAP_TOPIC "/mur/planner/map"
#define CONTROL_PATH_TOPIC "/mur/planner/path"
#define CONTROL_TRANSITION_TOPIC "/mur/control/transition"
#define CONTROL_OUTPUT_TOPIC "/husky_velocity_control/cmd_vel"

#define SYSTEM_START_TOPIC "/mur_start_system"
#define CONES_RVIZ_TOPIC "/rviz_cones"
#endif