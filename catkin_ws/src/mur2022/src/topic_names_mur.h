
#ifndef TOPIC_NAMES
#define TOPIC_NAMES

// Camera/Image Topics
#define LEFT_IMAGE_TOPIC "/CameraLeft/image_raw" 
#define RIGHT_IMAGE_TOPIC "/CameraRight/image_raw"

// Cone Topics
#define CONE_DETECTED_TOPIC "/stereo_cone"
#define CONES_FULL_TOPIC "/full_cones"

// Lidar Topics
#define LIDAR_RAW_TOPIC "/ouster/points"

// LeGO-LOAM Topics - If you change these, you will need to change them in LeGO-LOAM-BOR.
// Don't do it
#define LEGO_LOAM_ODOM "/integrated_to_init"
#define LEGO_LOAM_POINTCLOUD "/laser_cloud_surround"


// Control:
#define CMDVEL_TOPIC "/mur2022internal/cmd_vel"   
#define CONE_TOPIC "/mur/slam/cones"
#define FASTLAP_READY_TOPIC "/mur/control/transition"
#define FINISHED_MAP_TOPIC "/mur/planner/map"
#define HEALTH_TOPIC "/mur/planner/topic_health"
#define HUSKY_ODOM_TOPIC "/husky_odometry"
#define MAP_TOPIC "/mur/planner/map"
#define MUR_ODOM_TOPIC "/mur/slam/Odom"
#define PATH_CONES_TOPIC "/mur/planner/path_cones"
#define PATH_MARKER "/mur/planner/path_marks"
#define PATH_TOPIC "/mur/planner/path"
#define SORTED_LCONES_TOPIC "/mur/planner/left_sorted_cones"
#define SORTED_RCONES_TOPIC "/mur/planner/right_sorted_cones"
#define SORTING_MARKER "/mur/planner/sortCones"
#define TRANSITION_TOPIC FASTLAP_READY_TOPIC


// Rviz:
#define PATH_VIZ_TOPIC "/mur/planner/path_viz"
#define GOALPT_VIZ_TOPIC "/mur/follower/goatpt_viz"
#define RVIZ_TOPIC "/mpcc_RVIZ_topic"
#define RVIZ_FRAME "odom"

#endif