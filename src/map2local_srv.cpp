#include "ros/ros.h"
#include "iusc_maze/map2local.h"
#include <sensor_msgs/NavSatFix.h>
#include "GPS_CoTF.h"

bool gps_init_done=false;
GPS_CoTF* cotf_ptr = nullptr;

bool map2local_cb(iusc_maze::map2local::Request& request,iusc_maze::map2local::Response& response){
    double x_map = request.x_map;
    double y_map = request.y_map;
    double x_local = 0.0;
    double y_local = 0.0;

    cotf_ptr->MSN_to_ENU(x_map,y_map,x_local,y_local);
    response.x_local = x_local;
    response.y_local = y_local;
    return true;
}

void lalo_callback(const sensor_msgs::NavSatFix::ConstPtr &lalo, double *lat, double *lon)
{
    *lat = lalo->latitude;
    *lon = lalo->longitude;
    if(!gps_init_done)
    {
        gps_init_done = true;
    }
}

int main(int argc, char **argv){

    // if (argc != 3){
    //     ROS_ERROR_ONCE("Wrong number of arguments");
    //     return 1;
    // }
    ros::init(argc,argv,"map2local_server");
    ros::NodeHandle nh;
    ros::service::waitForService("mavros/get_loggers",10.0);
    int uav_id = 0 ;
    uav_id = atoi(argv[1]);

    // 读取初始位置的经纬度
    ros::Rate loop_rate(50);
    ros::Rate rate1(1.0);
    double init_lat = 0.0, init_lon = 0.0;
    ros::Subscriber lalo_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, boost::bind(&lalo_callback, _1, &init_lat, &init_lon));
    cout << " --- uav " << uav_id << " ----Waiting GPS in map 2 local service----" << endl;
    while(!gps_init_done)
    {
        rate1.sleep();
        cout << " uav " << uav_id << " wait for gps init done" << endl;
        ros::spinOnce();
    }
    cout << "--- uav " << uav_id << "----gps init done!! in map 2 local service----" << endl;
    cout << "lat: " << init_lat << ", lon: " << init_lon << endl;
    lalo_sub.shutdown();

    // 初始化坐标转换器
    Eigen::Vector2d ENU_LALO(init_lat, init_lon);
    vector<Eigen::Vector2d> MSN_LALO;
    vector<Eigen::Vector2d> MSN_XY;
    // wait for message setdown
    
    string param_name = "/uav" + std::to_string(uav_id) + "/point_num";
    while( !ros::param::has(param_name) ) {
        cout << " wait for param : " << param_name << endl; 
        rate1.sleep();
    }
    int point_num = 0;
    nh.getParam(param_name, point_num);
    cout << "--- uav " << uav_id <<  "point_num in map 2 local service= " << point_num << endl;

    for(int i = 0; i< point_num; i++)
    {
        double lat = 0.0, lon = 0.0;
        double x = -1.0, y = -1.0;
        nh.getParam("point_"+to_string(i)+"_lat", lat);
        nh.getParam("point_"+to_string(i)+"_lon", lon);
        nh.getParam("point_"+to_string(i)+"_x", x);
        nh.getParam("point_"+to_string(i)+"_y", y);
        MSN_LALO.push_back(Eigen::Vector2d(lat, lon));
        MSN_XY.push_back(Eigen::Vector2d(x, y));
        cout << "lat = " << lat << ", lon = " << lon << ", x = " << x << ", y = " << y << endl;
    }
    // 初始化坐标转换器
    cotf_ptr = new GPS_CoTF(0.0, ENU_LALO, MSN_LALO, MSN_XY);
    //
    ros::ServiceServer server = nh.advertiseService("map2local_server",map2local_cb);
    ROS_INFO(" ---  map2local server is ready --- ");
    ros::spin();
    return 0;
}