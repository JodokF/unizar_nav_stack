#include <traj_gen_real_drone.h>

/******************** Constructors ******************/

poly_traj_plan::poly_traj_plan(ros::NodeHandle& nh){    

    //default_odom_topic = "/ground_truth/state";
    //default_odom_topic = "/vrpn_client_node/cine_mpc/pose";
    default_odom_topic = "/optitrack/pose"; // = cine_mpc drone pose (when right launch file is executed)
    default_goal_topic = "/vrpn_client_node/goal_optitrack/pose";

    nh.param("/odom_topic", odom_topic, default_odom_topic);
    nh.param("/goal_topic", goal_topic, default_goal_topic);
    nh.param("/planner_service", planner_service, std::string("/voxblox_rrt_planner/plan"));


    takeoff_altitude = 1;
    nmbr_of_states = 0;
    curr_state = 0;
    waypoint_cntr = 0;
    marker_cntr = 0;
    sampling_interval = 0.1;
    vel_threshold = 0.3;

    //odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 10, &poly_traj_plan::poseCallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>
                ("/cmd_vel_control",10);
    vel_pub_2_real_drone = nh.advertise<geometry_msgs::Twist>
                ("/vel2real_drone",10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>
                ("/command/pose", 10);
    mav_traj_markers_pub = nh.advertise<visualization_msgs::MarkerArray>
                ("/trajectory_markers", 0);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>
                ("/waypoint_markers", 10);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                (odom_topic,10,&poly_traj_plan::poseCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>
                (goal_topic,10,&poly_traj_plan::goalCallback,this);
    //plan_sub = nh.subscribe<geometry_msgs::PoseArray>("/rrt_planner/path",1,&poly_traj_plan::planCallback,this);
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

real_drone_connection::real_drone_connection(ros::NodeHandle& nh){

    real_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    planned_vel_sub = nh.subscribe<geometry_msgs::Twist>
            ("/vel2real_drone", 10, &real_drone_connection::vel_cb, this);

    current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/optitrack/pose",10, &real_drone_connection::pose_cb, this);

    state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, &real_drone_connection::state_cb, this);
    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");

    set_cmd_vel_frame = nh.serviceClient<mavros_msgs::SetMavFrame>
            ("/mavros/setpoint_velocity/mav_frame");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    // ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("mavros/setpoint_velocity/cmd_vel", 10);

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0; 


}

/******************** Functions ******************/

void real_drone_connection::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    std::cout << "--- Curr. State: " << current_state.mode << "\n";
}

void real_drone_connection::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

void real_drone_connection::vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
    planned_vel = *msg;
}

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

void poly_traj_plan::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){    
    odom_info = *msg;
    odom_received = true;
    // std::cout << "\n--- odom received ---\n";
    
}

double poly_traj_plan::get_yaw_from_quat(const geometry_msgs::Quaternion q){
    double roll, pitch, yaw;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(q, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
    return yaw;

}

geometry_msgs::Pose poly_traj_plan::calculateMidpoint(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    geometry_msgs::Pose midpoint;

    midpoint.position.x = (pose1.position.x + pose2.position.x) / 2;
    midpoint.position.y = (pose1.position.y + pose2.position.y) / 2;
    midpoint.position.z = (pose1.position.z + pose2.position.z) / 2;

    return midpoint;
}

bool poly_traj_plan::generate_trajectory() {

    //constants
    const int dimension = 3; //we only compute the trajectory in x, y and z
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP

    // Definition of the trajectory beginning, end and intermediate constraints
    mav_trajectory_generation::Vertex start(dimension), middle_points(dimension), end(dimension);

        // make start point: 
    start.makeStartOrEnd(Eigen::Vector3d(odom_info.pose.position.x,odom_info.pose.position.y,0.8), derivative_to_optimize);

    std::cout << "Start x, y, z: \t"    << odom_info.pose.position.x << ", " 
                                        << odom_info.pose.position.y << ", " 
                                        << odom_info.pose.position.z << std::endl;
    vertices.push_back(start);


    //drawMarkerArray(vxblx_waypoints, 2, 1);

    // insert points between each waypoint to get a stricter trajectory:
    
    for (size_t i = 0; i < vxblx_waypoints.poses.size() - 1; ++i) {
        // Calculate the midpoint between the current pose and the next pose
        geometry_msgs::Pose midpoint = calculateMidpoint(vxblx_waypoints.poses[i], vxblx_waypoints.poses[i + 1]);

        // Insert the midpoint pose between the current and next pose
        vxblx_waypoints.poses.insert(vxblx_waypoints.poses.begin() + i + 1, midpoint);

        // Increment the index to account for the newly inserted midpoint pose
        ++i;
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

    // make end point: 
    end.makeStartOrEnd(Eigen::Vector3d(goal.pose.position.x,goal.pose.position.y,goal.pose.position.z), derivative_to_optimize);
    vertices.push_back(end);
    std::cout << "Goal x, y, z: \t" << goal.pose.position.x << ", " 
                                    << goal.pose.position.y << ", " 
                                    << goal.pose.position.z << std::endl;

    
    // TODO understand this:
    // Provide the time constraints on the vertices
    //Automatic time computation 
    std::vector<double> segment_times;
    const double v_max = vel_threshold; 
    const double a_max = vel_threshold;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    std::cout << "Segment times = " << segment_times.size() << std::endl;
    
    // Solve the optimization problem
    const int N = 10; //Degree of the polynomial, even number at least two times the highest derivative
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    //Obtain the trajectory
    trajectory.clear();
    opt.getTrajectory(&trajectory);

    //Sample the trajectory (to obtain positions, velocities, etc.)
    sampling_interval = 0.1; //How much time between intermediate points
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &traj_states);

//  ************** DEBUG PRINTS AND MARKERS **************

    // Example to access the data
    std::cout << "Trajectory time = " << trajectory.getMaxTime() << std::endl;
    std::cout << "Number of states = " << traj_states.size() << std::endl;
    //std::cout << "Position (world frame) at stamp " << 25 << ", x = " << traj_states[25].position_W.x() << std::endl;
    //std::cout << "Velocity (world frame) at stamp " << 25 << ", x = " << traj_states[25].velocity_W.x() << std::endl;

    //AROB visualization
    drawMAVTrajectoryMarkers();

//  ************** DEBUG PRINTS AND MARKERS **************

    return success;

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
    sampling_interval = 0.1;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);   
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

double poly_traj_plan::goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal){
    double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
    double euc_dis = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
    return euc_dis;
}

int poly_traj_plan::takeoff(){

    motor_enable_service_msg.request.enable = true;
    motor_enable_service.call(motor_enable_service_msg);

    while(!odom_received) // waiting for odom
    {
        ros::Duration(0.25).sleep(); // safety first
    }

    vel_msg.linear.z = 0.5;
    while(odom_info.pose.position.z < takeoff_altitude) // drone velocity in 'z' is set to 0.5 until takeoff_altitude is reached
    {
        std::cout << "Z - Pos drone:" << odom_info.pose.position.z << std::endl;
        vel_pub.publish(vel_msg);
        ros::Duration(0.25).sleep();
    }        
    vel_msg.linear.z = 0.0;
    vel_pub.publish(vel_msg);

    geometry_msgs::PoseStamped standStill;
    standStill.pose.position.x = odom_info.pose.position.x;
    standStill.pose.position.y = odom_info.pose.position.y;
    standStill.pose.position.z = odom_info.pose.position.z;
    standStill.header.frame_id = "world";

    // publish a few times to reach gazebo (only ome time didn't work)
    for(int i = 0; i < 5; i++){
        pose_pub.publish(standStill);
        ros::Duration(0.05).sleep();
    }

    return 1;
}

int poly_traj_plan::run_navigation_node(){

    ROS_INFO("Navigation Node Starts");

    ros::Rate rate(10);

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
    std::cout << "---\nDrooone position (x, y, z): (" << odom_info.pose.position.x << ", " 
                                                    << odom_info.pose.position.y << ", " 
                                                    << odom_info.pose.position.z << ") \n"; 

    std::cout << "Goal  position (x, y, z): (" << goal.pose.position.x << ", " 
                                                << goal.pose.position.y << ", " 
                                                << goal.pose.position.z << ") \n---\n";


    // start_pose is assigened in the odom callback
    request.request.start_pose.pose.position.x = odom_info.pose.position.x;
    request.request.start_pose.pose.position.y = odom_info.pose.position.y;
    //request.request.start_pose.header.stamp = odom_info.header.stamp;
    //request.request.start_pose.header.frame_id = odom_info.header.frame_id;
    request.request.start_pose.pose.position.z = 1.2;//odom_info.pose.position.z;

    request.request.goal_pose.pose.position.x = goal.pose.position.x;
    request.request.goal_pose.pose.position.y = goal.pose.position.y;
    request.request.goal_pose.pose.position.z = 1.2;

    // requesting voxblox-rrt-planner to plan:
    try {
        if (!ros::service::call(planner_service, request)) {
            ROS_WARN_STREAM("Couldn't call service: " << planner_service);
            return -1;
        }
        else ROS_ERROR_STREAM("Planner Service fail.");
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

int real_drone_connection::establish_drone_connection(){
    
    //tf2_ros::Buffer tf_buffer_;
    //tf2_ros::TransformListener tf_listener_(tf_buffer_, nh);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    int debugg_int = 0;
    while(ros::ok() && !current_state.connected){
        debugg_int++;
        if (debugg_int % 10 == 0) std::cout << "Waiting for connection to mav ros\n";
        ros::spinOnce();
        rate.sleep();
    }


    //Stablish the cmd_vel frame
    mavros_msgs::SetMavFrame set_frame_msg;
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_BODY_NED;
    set_cmd_vel_frame.call(set_frame_msg);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        real_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    // --------ARMING AND OFFBOARD --------------
    //REQUEST OFFBOARD FLIGHT MODE
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //REQUEST ARM DRONE
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    //LOOP UNTIL POSITION AND ARMED
    ros::Rate wait_rate(1.0);
    
    while(ros::ok() && !current_state.armed && current_state.mode != "POSITION"){
        ROS_INFO("Arm manually and enable position mode before going offboard");
        ros::spinOnce();
        wait_rate.sleep();
    }
    
    bool offboard = false;

    //LOOP UNTIL OFFBOARD
    do{
        ROS_INFO("Trying to go offboard");
        //Pass offboard
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
            offboard = true;
        }else{
            if(!current_state.armed){
                ROS_INFO("NOT ARMED, ARM FIRST");
            }
        }
    }while(ros::ok() && !offboard);
    // --------END OF ARMING AND OFFBOARD --------------

    return 0;

}

bool real_drone_connection::send_vel_commands(  const mav_msgs::EigenTrajectoryPoint& traj_state, 
                                                double threshold_from_trajectory){
    
    bool threshold = threshold_from_trajectory;
    bool vel_checker = false;

    if (traj_state.velocity_W.x()         < threshold &&
        traj_state.velocity_W.y()         < threshold &&
        traj_state.velocity_W.z()         < threshold &&
        traj_state.angular_velocity_W.x() < threshold &&
        traj_state.angular_velocity_W.y() < threshold &&
        traj_state.angular_velocity_W.z() < threshold)
        {   vel_checker = true;
            
    } else {
            ROS_ERROR("Velocity too high");
            return false;
    }

    if(vel_checker == true){ 
        vel_msg.linear.x = traj_state.velocity_W.x();
        vel_msg.linear.y = traj_state.velocity_W.y();
        vel_msg.linear.z = traj_state.velocity_W.z();
        vel_msg.angular.x = traj_state.angular_velocity_W.x();
        vel_msg.angular.y = traj_state.angular_velocity_W.y();
        vel_msg.angular.z = traj_state.angular_velocity_W.z();

        // Publish the command
        
        // real_vel_pub.publish(vel_msg);
        std::cout << "--- Published Vels: \n" << vel_msg << "\n";
    }
    return true;

}


/******************** main() ******************/

int main(int argc, char** argv){
    
    ros::init(argc, argv, "traj_gen_this_name_is_not_used_anywhere_so_nevermind");
    ros::AsyncSpinner ich_spinne(1);
    ich_spinne.start();
    ros::NodeHandle node_handle("~");
    poly_traj_plan ptp(std::ref(node_handle));
    int current_traj_state = 0;

    // if (ptp.takeoff() == 1) ROS_INFO("Take-off succesfull");

    /*
    // run the navigation node and get the trajectory    
    int nav_function_checker = ptp.run_navigation_node();
    if (nav_function_checker == 0) ROS_INFO("Navigation succesfully");
    else if (nav_function_checker == -1 ){
        ROS_ERROR("Navigation error");
        return -1;
    }
    
    
    std::cout << "\n\n\n !!! Check the trajectory in RViz before take off!!! \n\n\n";
    ros::Duration(3).sleep();
    std::cout << "Press a key to continue.";
    // Wait for any input from the user
    std::cin.get();
    */

    // setting up the real drone
    real_drone_connection drone(std::ref(node_handle));
    
    if (drone.establish_drone_connection() != 0) {
        ROS_ERROR("Drone connection not possible");
        return -1;
    }

/*
    // sending vel commands to the real drone
    while(ros::ok() && current_traj_state < ptp.traj_states.size()){

        if (current_traj_state < ptp.traj_states.size()){
            drone.send_vel_commands(ptp.traj_states.at(current_traj_state), ptp.vel_threshold);
        }
        // debug output:
        //if (current_traj_state % 5 == 0) std::cout << "Velocitie published! \n";
    
        current_traj_state++;
        // sampling_interval comes from the trajectory generation
        ros::Duration(ptp.sampling_interval).sleep();
    }

*/

    // I guess debugg:
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = -0.2; 

    while(ros::ok()){
        drone.real_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    return 0;

}


