#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGridPtr map(new nav_msgs::OccupancyGrid);

int rClearence_ = 50;
int oLength_ = 10; 
int oCount_ = 100;

void initializeMap();

void updateMap();

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("ares/map/obstacles", 1, true);

    /* Fetch parameters from parameter server */
    nh.getParam("/ares/map/rClearence", rClearence_);
    nh.getParam("/ares/map/oCount", oCount_);
    nh.getParam("/ares/map/oLength", oLength_);

    /* Initialize map from local bitmap. */
    initializeMap();

    ros::Rate rate(2);
    while (ros::ok()) {
        pub.publish(map);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void initializeMap() {
    /* Initialize header */
    map->header.seq = 0;
    map->header.stamp = ros::Time::now();
    map->header.frame_id = "map";

    /* Initialize metadata */
    map->info.map_load_time = ros::Time::now();

    /* Initialize map dimensions */
    map->info.width = 512;
    map->info.height = 512;
    map->info.resolution = 0.1;
    map->info.origin.position.x = -(512)*(0.1)/2.0;
    map->info.origin.position.y = -(512)*(0.1)/2.0;
    map->info.origin.position.z = 0.0;
    map->info.origin.orientation.x = 0.0;
    map->info.origin.orientation.y = 0.0;
    map->info.origin.orientation.z = 0.0;
    map->info.origin.orientation.w = 1.0;

    /* Initialize map data */
    map->data = std::vector<int8_t>(512*512, 0);

    /* Draw random rectangle spots*/
    for (int i=0; i<oCount_; i++) {
        int x = rand() % 512;
        int y = rand() % 512;
        for (int j=-oLength_/2; j<oLength_/2; j++) {
            for (int k=-oLength_/2; k<oLength_/2; k++) {
                map->data[(y+j)*512 + (x+k)] = 100;
            }
        }
    }

    /* Clear the center point of obstructions */
    for (int i=-rClearence_/2; i<rClearence_/2; i++) {
        for (int j=-rClearence_/2; j<rClearence_/2; j++) {
            map->data[(256+i)*512 + (256+j)] = 0;
        }
    }
}

void updateMap() {
    /* Update headers */
    map->header.stamp = ros::Time::now();
    map->header.seq++;
}