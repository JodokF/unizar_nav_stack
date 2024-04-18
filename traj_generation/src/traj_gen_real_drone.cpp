#include <traj_gen_real_drone.h>

/******************** Constructors ******************/

poly_traj_plan::poly_traj_plan(ros::NodeHandle& nh){    

    odom_topic = "/mavros/odometry/out";
    //odom_topic = "/optitrack/pose"; // = cine_mpc drone pose (when the optitrack launch file is executed)
    //goal_topic = "/vrpn_client_node/goal_optitrack/pose";

    nh.param("/planner_service", planner_service, std::string("/voxblox_rrt_planner/plan"));


    takeoff_altitude = 1;
    nmbr_of_states = 0;
    curr_state = 0;
    waypoint_cntr = 0;
    marker_cntr = 0;
    sampling_interval = 0.05;

    // for the simulation: (instead of the optitrack goal)
        goal.pose.position.x = 2; 
        goal.pose.position.y = -1; 
        goal.pose.position.z = 0.75; 

    vel_pub = nh.advertise<geometry_msgs::Twist>
                ("/vel_cmd_2_drone",10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>
                ("/command/pose", 10);
    mav_traj_markers_pub = nh.advertise<visualization_msgs::MarkerArray>
                ("/trajectory_markers", 0);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>
                ("/waypoint_markers", 10);
    pose_sub = nh.subscribe<nav_msgs::Odometry>
                (odom_topic,10,&poly_traj_plan::poseCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>
                (goal_topic,10,&poly_traj_plan::goalCallback,this);
    plan_sub = nh.subscribe<geometry_msgs::PoseArray>
                ("/waypoint_list",10,&poly_traj_plan::planCallback,this);
    path_plan_client = nh.serviceClient<std_srvs::Empty>
                ("/voxblox_rrt_planner/publish_path");
    motor_enable_service = nh.serviceClient<hector_uav_msgs::EnableMotors>
                ("/enable_motors");

    waypoints_received = false;
    odom_received = false;
    goal_recieved = true;
    planner_service_called = false;
}

/******************** Functions ******************/


void poly_traj_plan::planCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    
    vxblx_waypoints = *msg;
    // vxblx_waypoints = msg->poses;
    ROS_INFO("RRT Planner waypoints received.");
    waypoints_received = true;
    
}

void poly_traj_plan::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){

    goal = *msg;
    if (goal_recieved == false) ROS_INFO("Goal pose received from optitrack ");
    
    //request.request.goal_pose.pose.position.x = goal.position.x;
    //request.request.goal_pose.pose.position.y = goal.position.y;
    //request.request.goal_pose.pose.position.z = goal.position.z;

    goal_recieved = true;

}

void poly_traj_plan::poseCallback(const nav_msgs::Odometry::ConstPtr &msg){    
    odom_info = *msg;
    odom_received = true;
    // std::cout << "\n--- odom received ---\n";
    
}

geometry_msgs::Pose poly_traj_plan::calculateMidpoint(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    geometry_msgs::Pose midpoint;

    midpoint.position.x = (pose1.position.x + pose2.position.x) / 2;
    midpoint.position.y = (pose1.position.y + pose2.position.y) / 2;
    midpoint.position.z = (pose1.position.z + pose2.position.z) / 2;

    return midpoint;
}

void poly_traj_plan::drawMarkerArray(geometry_msgs::PoseArray waypoints, int color, int offset){
     
    // offset -> if else markers would be on the same spot
    
    // color -> 1 = RED
    // color -> 2 = GREEN
    // color -> 3 = BLUE

    // Create a publisher for the marker array
    
    // Create a marker array message
    visualization_msgs::MarkerArray marker_array;

    // Create a marker for each pose in the PoseArray
    for (size_t i = 0; i < waypoints.poses.size(); ++i) {
        // Create a new marker for each pose
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.id = marker_cntr; // same id are getting overwritten 
        marker_cntr++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        if (offset == 0) marker.pose = waypoints.poses[i]; // Set the pose of the marker
        if (offset == 1 ){
            marker.pose.position.x = waypoints.poses.at(i).position.x + 0.5;
            marker.pose.position.y = waypoints.poses.at(i).position.y;
            marker.pose.position.z = waypoints.poses.at(i).position.z;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;
        }
        marker.scale.x = 0.1; 
        marker.scale.y = 0.1;
        marker.scale.z = 0.1; 
        marker.color.a = 1.0; // Alpha (transparency)
        

        switch (color)
        {
        case 1:
            marker.color.r = 1.0;
            marker.color.g = 0.0; 
            marker.color.b = 0.0; 
            break;
        
        case 2:
            marker.color.r = 0.0;
            marker.color.g = 1.0; 
            marker.color.b = 0.0; 
            break;

        case 3:
            marker.color.r = 0.0;
            marker.color.g = 0.0; 
            marker.color.b = 1.0; 
            break;
        
        default:
            break;
        }
        

        // Add the marker to the marker array
        marker_array.markers.push_back(marker);
    }

    // Publish the marker array
    marker_pub.publish(marker_array);
}

void poly_traj_plan::drawMAVTrajectoryMarkers(){

    visualization_msgs::MarkerArray markers;
    int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
    double sampling_time = 0.1;
    mav_msgs::EigenTrajectoryPoint::Vector states;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);   // sampling_time
    int id_marker = 0;

    for(int i=0; i< states.size(); i++) {
        visualization_msgs::Marker marker_aux;
        marker_aux.header.frame_id = "odom";
        //marker_aux.header.stamp = ros::Time::now();
        marker_aux.header.stamp = ros::Time(0);
        marker_aux.id = id_marker;
        id_marker++;
        marker_aux.ns = "point";
        marker_aux.type = visualization_msgs::Marker::CUBE;
        marker_aux.pose.position.x = states[i].position_W[0] ;
        marker_aux.pose.position.y = states[i].position_W[1] ;
        marker_aux.pose.position.z = states[i].position_W[2] ;
        marker_aux.pose.orientation.x = 0;
        marker_aux.pose.orientation.y = 0;
        marker_aux.pose.orientation.z = 0;
        marker_aux.pose.orientation.w = 1;
        marker_aux.scale.x = 0.03;
        marker_aux.scale.y = 0.03;
        marker_aux.scale.z = 0.03;
        marker_aux.color.r = 0.0f;
        marker_aux.color.g = 0.0f;
        marker_aux.color.b = 1.0f;
        marker_aux.color.a = 1.0;
        marker_aux.lifetime = ros::Duration();
        markers.markers.push_back(marker_aux);
    }
    mav_traj_markers_pub.publish(markers);
}

/*
double poly_traj_plan::goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal){
    double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
    double euc_dis = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
    return euc_dis;
}

double poly_traj_plan::get_yaw_from_quat(const geometry_msgs::Quaternion q){
    double roll, pitch, yaw;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(q, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
    return yaw;
}
*/

int poly_traj_plan::run_navigation_node(){

    ROS_INFO("Navigation Node Starts");

    int while_loop_ctrl = 0;
    while(odom_received == false || goal_recieved == false ){
        ros::Duration(1).sleep();
        std::cout << "Waiting for odom or goal.\n";
        while_loop_ctrl++;
        if (while_loop_ctrl == 15) 
            return -1;
    }
    while_loop_ctrl = 0;

    // Debug Info:
    std::cout << "---\nDrooone position for planner (x, y, z): (" << odom_info.pose.pose.position.x << ", " 
                                                                << odom_info.pose.pose.position.y << ", " 
                                                                << odom_info.pose.pose.position.z << ") \n"; 

    std::cout << "Goal  position (x, y, z) for planner: ("  << goal.pose.position.x << ", " 
                                                            << goal.pose.position.y << ", " 
                                                            << goal.pose.position.z << ") \n---\n";


    // start_pose is assigened in the odom callback
    request.request.start_pose.pose.position.x = odom_info.pose.pose.position.x;
    request.request.start_pose.pose.position.y = odom_info.pose.pose.position.y;
    request.request.start_pose.pose.position.z = odom_info.pose.pose.position.z; 

    request.request.goal_pose.pose.position.x = goal.pose.position.x;
    request.request.goal_pose.pose.position.y = goal.pose.position.y;
    request.request.goal_pose.pose.position.z = goal.pose.position.z;

    // requesting voxblox-rrt-planner to plan:
    try {
        if (!ros::service::call(planner_service, request)) {
            ROS_WARN_STREAM("Couldn't call service: " << planner_service);
            return -1;
        }
        else ROS_INFO("Planner Service called.");
    } 
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Service Exception: " << e.what());
    }
    
    // requesting waypoints from planner to be published:
    if (path_plan_client.call(path_plan_req)) {
        ROS_INFO("Planer Publisher called successfully");
    } else {
        ROS_ERROR_STREAM("Failed to call service publish_path");
    }

    // waiting for waypoints to recive
    while(!waypoints_received){
        std::cout << "Waiting for RRT Waypoints.\n";
        ros::Duration(1).sleep();
        while_loop_ctrl++;
        if (while_loop_ctrl == 5) return -1;
    }
    while_loop_ctrl = 0;

    if (generate_trajectory() == true) return 0;
    else return -1;


}

bool poly_traj_plan::generate_trajectory() {

    //constants
    const int dimension = 3; //we only compute the trajectory in x, y and z
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP

    // Definition of the trajectory beginning, end and intermediate constraints
    mav_trajectory_generation::Vertex start(dimension), middle_points(dimension), end(dimension);

        // make start point: 
    start.makeStartOrEnd(Eigen::Vector3d(odom_info.pose.pose.position.x,odom_info.pose.pose.position.y,odom_info.pose.pose.position.z), derivative_to_optimize);

    std::cout << "\n---\n";
    std::cout << "Start for traj. x, y, z: \t"    << odom_info.pose.pose.position.x << ", " 
                                                << odom_info.pose.pose.position.y << ", " 
                                                << odom_info.pose.pose.position.z << std::endl;
    vertices.push_back(start);


    //drawMarkerArray(vxblx_waypoints, 2, 1);

    // insert points between each waypoint to get a stricter trajectory:
    for(int i = 0; i < 2; i++){
        for (size_t i = 0; i < vxblx_waypoints.poses.size() - 1; ++i) {
            // Calculate the midpoint between the current pose and the next pose
            geometry_msgs::Pose midpoint = calculateMidpoint(vxblx_waypoints.poses[i], vxblx_waypoints.poses[i + 1]);

            // Insert the midpoint pose between the current and next pose
            vxblx_waypoints.poses.insert(vxblx_waypoints.poses.begin() + i + 1, midpoint);

            // Increment the index to account for the newly inserted midpoint pose
            ++i;
        }
    }
    
    // delete the first and last entry of the recived trajectory waypoints 
    // becaue they are already considered in start and goal vertices
    vxblx_waypoints.poses.erase(vxblx_waypoints.poses.begin());
    vxblx_waypoints.poses.erase(vxblx_waypoints.poses.end() - 1);

    drawMarkerArray(vxblx_waypoints, 1, 0);

        
    for (const auto& waypoint : vxblx_waypoints.poses) {

        mav_trajectory_generation::Vertex middle_points(dimension);
        
        // Extract the position from the waypoints
        Eigen::Vector3d wp_pos_eigen(
            waypoint.position.x,
            waypoint.position.y,
            waypoint.position.z
        );
                
        std::cout << waypoint_cntr << ". Waypoint x, y, z: \t" 
                  << waypoint.position.x << ", "
                  << waypoint.position.y << ", " 
                  << waypoint.position.z << std::endl;
        waypoint_cntr++;

        // Add the position constraint for the waypoint
        middle_points.addConstraint(mav_trajectory_generation::derivative_order::POSITION, wp_pos_eigen);
        
        // Add the vertex to the trajectory vertices
        vertices.push_back(middle_points);

    }

    // make end point, z + 1 meter: 
    end.makeStartOrEnd(Eigen::Vector3d(goal.pose.position.x,goal.pose.position.y,goal.pose.position.z), derivative_to_optimize);
    vertices.push_back(end);
    std::cout << "Goal for traj. x, y, z: \t" << goal.pose.position.x << ", " 
                                            << goal.pose.position.y << ", " 
                                            << goal.pose.position.z << std::endl;
    std::cout << "\n---\n";

    
    // TODO understand this:
    // Provide the time constraints on the vertices
    //Automatic time computation 
    std::vector<double> segment_times;
    vel_threshold = 1;
    const double v_max = vel_threshold; 
    const double a_max = vel_threshold;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    
    // Solve the optimization problem
    const int N = 10; //Degree of the polynomial, even number at least two times the highest derivative
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    //Obtain the trajectory
    trajectory.clear();
    opt.getTrajectory(&trajectory);

    //Sample the trajectory (to obtain positions, velocities, etc.)
    sampling_interval = 0.05; //How much time between intermediate points
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &traj_states);

//  ************** DEBUG PRINTS AND MARKERS **************

    // Example to access the data
    std::cout << "---\n";
    std::cout << "Trajectory time Max = " << trajectory.getMaxTime() << std::endl;
    std::cout << "Trajectory time Min = " << trajectory.getMaxTime() << std::endl;
    std::cout << "Number of states = " << traj_states.size() << std::endl;
    std::cout << "Estimated Segment times = " << segment_times.size() << std::endl;
    std::cout << "Sampling intervall = " << sampling_interval << std::endl;
    std::cout << "---\n\n";
    //std::cout << "Position (world frame) at stamp " << 25 << ", x = " << traj_states[25].position_W.x() << std::endl;
    //std::cout << "Velocity (world frame) at stamp " << 25 << ", x = " << traj_states[25].velocity_W.x() << std::endl;

    //AROB visualization
    drawMAVTrajectoryMarkers();

//  ************** DEBUG PRINTS AND MARKERS **************

    return success;

}

void poly_traj_plan::send_vel_commands() {
    
    if(curr_state <= traj_states.size()){ 
        vel_msg.linear.x = traj_states[curr_state].velocity_W.x();
        vel_msg.linear.y = traj_states[curr_state].velocity_W.y();
        vel_msg.linear.z = traj_states[curr_state].velocity_W.z();
        vel_msg.angular.x = traj_states[curr_state].angular_velocity_W.x();
        vel_msg.angular.y = traj_states[curr_state].angular_velocity_W.y();
        vel_msg.angular.z = traj_states[curr_state].angular_velocity_W.z();

        // Publish the command
        vel_pub.publish(vel_msg);

    }else{
        vel_msg.linear.x = 0.0;
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;
        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = 0.0;
        
        vel_pub.publish(vel_msg);
        
    }
        curr_state++;
}


int check_trajectory(){

        std::cout << "Do you want to continue with this trajectory? (y/n): ";
        
        // Declare a variable to store user input
        char userInput;
        std::cin >> userInput; // Read user input

        // Check if the user wants to continue or abort
        if (userInput == 'y' || userInput == 'Y') return 0; 
        else if (userInput == 'n' || userInput == 'N') {
            return 1;
        } 
        else {
            // Handle invalid input
            ROS_ERROR("Invalid input donkey.");
            return 2; // Exit the program with an error code
        }

}

/******************** main() ******************/



int main(int argc, char** argv){
    
    ros::init(argc, argv, "traj_gen_this_name_is_not_used_anywhere_so_nevermind");
    ros::AsyncSpinner ich_spinne(1);
    ich_spinne.start();
    ros::NodeHandle node_handle("~");
    poly_traj_plan ptp(std::ref(node_handle));
    int current_traj_state = 0;


    bool traj_checker = false;
    do{
        // run the navigation node and get the trajectory    
        std::cout << ("\n\n---\n\n");
        int nav_func_checker = ptp.run_navigation_node();

        if (nav_func_checker == 0) ROS_INFO("Navigation succesfully");
        else if (nav_func_checker == -1 ){
            ROS_ERROR("Navigation error");
            return -1;
        }
        
        if(check_trajectory() == 0) traj_checker = true;
    
    
    } while (traj_checker == false);
    
    
    // sending vel commands to the real drone
    while(ros::ok()){
        ptp.send_vel_commands();
        ros::Duration(ptp.sampling_interval).sleep();
        // ros::Duration(0.05).sleep();
    }


    return 0;

}


