#include <traj_gen_real_drone.h>

/******************** Constructors ******************/

poly_traj_plan::poly_traj_plan(ros::NodeHandle& nh)
                : tf_listener(tf_buffer, nh) 
{    

    //odom_topic = "/mavros/odometry/out";
    // pose_nav_msg_odom_msg_topic = "/optitrack/pose"; // = cine_mpc drone pose (when the optitrack launch file is executed)
    goal_topic = "/vrpn_client_node/goal_optitrack/pose";

    nh.param("/planner_service", planner_service, std::string("/voxblox_rrt_planner/plan"));
    nh.getParam("/drone_connection_node/tracking_camera", tracking_camera); // check if tracking camera is used
    nh.getParam("/drone_connection_node/vxblx_active", vxblx_active);

    nmbr_of_states = 0;
    curr_state = 0;
    waypoint_cntr = 0;
    marker_cntr = 0;
    sampling_interval = 0.05;


    vel_pub = nh.advertise<geometry_msgs::Twist>
                ("/vel_cmd_2_drone",10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>
                ("/pose_cmd_2_drone", 10);
    mav_traj_markers_pub = nh.advertise<visualization_msgs::MarkerArray>
                ("/trajectory_markers", 0);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>
                ("/waypoint_markers", 10);
    pose_sub_geomtry_msg_pose = nh.subscribe<geometry_msgs::PoseStamped>
                ("/optitrack/pose",10,&poly_traj_plan::poseCallback_geomtry_msg_pose,this);
    pose_sub_nav_msg_odom = nh.subscribe<nav_msgs::Odometry>
                ("/mavros/odometry/out",10,&poly_traj_plan::poseCallback_nav_msg_odom,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>
                (goal_topic,10,&poly_traj_plan::goalCallback,this);
    plan_sub = nh.subscribe<geometry_msgs::PoseArray>
                ("/waypoint_list",10,&poly_traj_plan::planCallback,this);
    path_plan_client = nh.serviceClient<std_srvs::Empty>
                ("/voxblox_rrt_planner/publish_path");
    // motor_enable_service = nh.serviceClient<hector_uav_msgs::EnableMotors>
    //             ("/enable_motors");

    waypoints_received = false;
    odom_received = false;
    planner_service_called = false;

    if(vxblx_active == true) goal_recieved = false;

    else if (vxblx_active == false){
        // for the hard coded trajectory: (when no optitrack goal is detected)
        altitude_factor = 1.0; // to *reduce* the hight of the flying ocho -> if it is 0 flying high is between 1 and 3 m -> start & goal at 2 m
        x_offset = 1.0; // to move the trajectory in x
        goal.pose.position.x = -1.5; 
        goal.pose.position.y = 1.5; 
        goal.pose.position.z = 1.5; 
        goal_recieved = true;
    }

    target_frame = "odom";
    source_frame = "base_link";
    temp_pose.pose.orientation.w = 1.0;
    odom_info_geo_msg.pose.orientation.w = 1.0;
}

/******************** Functions ******************/


void poly_traj_plan::planCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    
    vxblx_waypoints = *msg;
    ROS_INFO("RRT Planner waypoints received.");
    waypoints_received = true;
    
}

void poly_traj_plan::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){

    goal = *msg;
    goal.pose.position.z = 1.2; // so it's not on the floor...
    if (goal_recieved == false) ROS_INFO("Goal pose received from optitrack ");
    
    goal_recieved = true;

}

void poly_traj_plan::poseCallback_nav_msg_odom(const nav_msgs::Odometry::ConstPtr &msg){    
    
    if (tracking_camera == false){
        odom_info = *msg;
        odom_received = true;
    }

    // The following is necessary because the tracking camera publishes only a topic in respect to the camera_odom_frame 
    // but since this frame is at (0.16, 0, 0.205) at start up and not (0, 0, 0) we need the pose of the cam
    // in respect to the odom frame
    if(tracking_camera == true){ 
        // to evade some error msgs at the startup
        if (tf_buffer.canTransform(target_frame, source_frame, ros::Time(0))) {
            try{
                tf_odom_to_camera = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
            } catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
            tf2::doTransform(temp_pose, odom_info_geo_msg, tf_odom_to_camera); // temp_pose has to be an geometry_msgs::PoseStamped...
        
            odom_info.header = odom_info_geo_msg.header; 
            odom_info.pose.pose = odom_info_geo_msg.pose;
            odom_received = true;
        }
        else {
        ROS_WARN("Transform not available yet, waiting...");
        ros::Duration(0.5).sleep();
        }

    }
}

void poly_traj_plan::poseCallback_geomtry_msg_pose(const geometry_msgs::PoseStamped::ConstPtr &msg){
    
    odom_info.header = msg->header;
    odom_info.pose.pose = msg->pose;    

    odom_received = true;
}

geometry_msgs::Pose poly_traj_plan::calculateMidpoint(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    geometry_msgs::Pose midpoint;

    midpoint.position.x = (pose1.position.x + pose2.position.x) / 2;
    midpoint.position.y = (pose1.position.y + pose2.position.y) / 2;
    midpoint.position.z = (pose1.position.z + pose2.position.z) / 2;

    return midpoint;
}

void poly_traj_plan::drawMarkerArray(std::vector<nav_msgs::Odometry> waypoints, int color, int offset){
     
    // offset -> if else markers would be on the same spot
    
    // color -> 1 = RED
    // color -> 2 = GREEN
    // color -> 3 = BLUE

    // Create a publisher for the marker array
    
    // Create a marker array message
    visualization_msgs::MarkerArray marker_array;

    // Create a marker for each pose in the PoseArray
    for (size_t i = 0; i < waypoints.size(); ++i) {
        // Create a new marker for each pose
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.id = marker_cntr; // same id are getting overwritten 
        marker_cntr++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        if (offset == 0) marker.pose = waypoints.at(i).pose.pose; // Set the pose of the marker
        if (offset == 1 ){
            marker.pose.position.x = waypoints.at(i).pose.pose.position.x + 0.5;
            marker.pose.position.y = waypoints.at(i).pose.pose.position.y;
            marker.pose.position.z = waypoints.at(i).pose.pose.position.z;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;
        }
        marker.scale.x = 0.05; 
        marker.scale.y = 0.05;
        marker.scale.z = 0.05; 
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
        marker_aux.type = visualization_msgs::Marker::SPHERE;
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
        marker_aux.color.g = 0.5f;
        marker_aux.color.b = 1.0f;
        marker_aux.color.a = 0.75;
        marker_aux.lifetime = ros::Duration();
        markers.markers.push_back(marker_aux);
    }
    mav_traj_markers_pub.publish(markers);
}

int poly_traj_plan::run_navigation_node(){

    ROS_INFO("Navigation Node Starts");
    std::cout << "\n---\nUsing Voxblox: "<< vxblx_active << "\n---\n";

    int while_loop_ctrl = 0;
    while(odom_received == false || goal_recieved == false ){
        ros::Duration(1).sleep();
        std::cout << "Waiting for odom or goal.\n";
        while_loop_ctrl++;
        if (while_loop_ctrl == 10) 
            return -1;
    }
    while_loop_ctrl = 0;

    // Debug Info:
    std::cout << "---\nDrone position for planner (x, y, z): (" << odom_info.pose.pose.position.x << ", " 
                                                                << odom_info.pose.pose.position.y << ", " 
                                                                << odom_info.pose.pose.position.z << ") \n"; 

    std::cout << "Goal  position (x, y, z) for planner: ("  << goal.pose.position.x << ", " 
                                                            << goal.pose.position.y << ", " 
                                                            << goal.pose.position.z << ") \n---\n";


    if(vxblx_active == false){
        if(generate_hardcoded_traj() == true) return 0;
    }

    if(vxblx_active == true)
    {
        request.request.start_pose.pose.position.x = odom_info.pose.pose.position.x;
        request.request.start_pose.pose.position.y = odom_info.pose.pose.position.y;
        request.request.start_pose.pose.position.z = odom_info.pose.pose.position.z; 

        request.request.goal_pose.pose.position.x = goal.pose.position.x;
        request.request.goal_pose.pose.position.y = goal.pose.position.y;
        request.request.goal_pose.pose.position.z = goal.pose.position.z; // because z < ~0.3 is probably occupied in the planner

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

        while(!waypoints_received){
            std::cout << "Waiting for RRT Waypoints.\n";
            ros::Duration(1).sleep();
            while_loop_ctrl++;
            if (while_loop_ctrl == 5) return -1;
        }
        while_loop_ctrl = 0;

        if (generate_voxblox_traj() == true) return 0;
    }

    return -1;
}

bool poly_traj_plan::generate_voxblox_traj() 
{
    ROS_INFO("Starting voxblox trajectory generation!");
    //constants
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP

    ROS_INFO("Using the RRT trajectory from Voxblox");    

/*  
    for (size_t i = 0; i < vxblx_waypoints.poses.size() - 1; ++i) {
        // Calculate the midpoint between the current pose and the next pose
        geometry_msgs::Pose midpoint = calculateMidpoint(vxblx_waypoints.poses[i], vxblx_waypoints.poses[i + 1]);

        // Insert the midpoint pose between the current and next pose
        vxblx_waypoints.poses.insert(vxblx_waypoints.poses.begin() + i + 1, midpoint);

        // Increment the index to account for the newly inserted midpoint pose
        ++i;
    }
*/
    
    traj_wp.clear();
    // converting the vxblx waypoints into an nav_msgs::Odometry Vector
    // so we are able to use the dravMarkerArray function...
    for (const auto& pose : vxblx_waypoints.poses) {
        std::cout << "Transforming waypoints to odom msg...\n";
        nav_msgs::Odometry odom;
        odom.pose.pose = pose;
        odom.header = vxblx_waypoints.header; // or set a new header if needed

        traj_wp.push_back(odom);
    }

    //drawMarkerArray(traj_wp, 2, 0.1);
    


// Definition of the trajectory beginning, end and intermediate constraints
    mav_trajectory_generation::Vertex start(dimension), middle_points(dimension), end(dimension);

    drawMarkerArray(traj_wp, 1, 0);
    
    // deleting the first and last entry of the recived trajectory waypoints 
    // becaue they are already considered in start and goal vertices
    traj_wp.erase(traj_wp.begin());
    traj_wp.erase(traj_wp.end() - 1);

    // make start point: 
    start.makeStartOrEnd(Eigen::Vector3d(odom_info.pose.pose.position.x,
                                         odom_info.pose.pose.position.y,
                                         odom_info.pose.pose.position.z), 
                                         derivative_to_optimize);

    std::cout << "\n---\n";
    std::cout << "Start for traj. x, y, z: \t"  << odom_info.pose.pose.position.x << ", " 
                                                << odom_info.pose.pose.position.y << ", " 
                                                << odom_info.pose.pose.position.z << std::endl;
    vertices.push_back(start);

    //for (const auto& waypoint : traj_wp) {
    for (const auto& waypoint : traj_wp) {

        mav_trajectory_generation::Vertex middle_points(dimension);
        
        // Extract the position from the waypoints
        Eigen::Vector3d pose_cnstr(
            waypoint.pose.pose.position.x,
            waypoint.pose.pose.position.y,
            waypoint.pose.pose.position.z
        );

        // Add the position constraint for the waypoint
        middle_points.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pose_cnstr);
        
        // Add the vertex to the trajectory vertices
        vertices.push_back(middle_points);

        // DEBUG PRINT:
        std::cout   << waypoint_cntr << ". Waypoint pose x, y, z & yaw: \t" 
                    << waypoint.pose.pose.position.x << ", "
                    << waypoint.pose.pose.position.y << ", "
                    << waypoint.pose.pose.position.z << " & "
                    << waypoint.pose.pose.orientation.z << "\n";

        waypoint_cntr++;

    }

    // make end point, z + 1 meter: 
    end.makeStartOrEnd(Eigen::Vector3d( goal.pose.position.x,
                                        goal.pose.position.y,
                                        goal.pose.position.z), 
                                        derivative_to_optimize);
    vertices.push_back(end);
    std::cout << "Goal for traj. x, y, z: \t" << goal.pose.position.x << ", " 
                                              << goal.pose.position.y << ", " 
                                              << goal.pose.position.z << std::endl;
    std::cout << "\n---\n";

    
    // TODO understand this:
    // Provide the time constraints on the vertices
    //Automatic time computation 
    std::vector<double> segment_times;
    vel_threshold = 0.5;
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

bool poly_traj_plan::generate_hardcoded_traj() 
{
    ROS_INFO("Starting hardcoded trajectory generation!");
    //constants
    const int dimension = 4;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP


    vel_threshold = 0.6; // geneal vel. limit for the traj. gen. see further down - 0.6 seems to work good  
    double x_y_vel = vel_threshold * 0.44; //0.44 worked the best for the flying ocho
    double z_vel_lin = 0;       // sets the steepnes of the fyling ocho 
    double z_vel_ang = 0.1;
    double mas_menos = 0.2;

    geometry_msgs::Pose pose;
    nav_msgs::Odometry wp0, wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8; // wp = waypoint, wp0 & wp8 = start & end

// Square

// middle point:
    wp1.pose.pose.position.x = -1.5;
    wp1.pose.pose.position.y = 0;
    wp1.pose.pose.position.z = 1.5;
    wp1.pose.pose.orientation.z = -M_PI/2; // we use it now as if it were yaw and not a quaternion
    wp1.twist.twist.linear.x = 0;
    wp1.twist.twist.linear.y = - x_y_vel - mas_menos; 
    wp1.twist.twist.linear.z = 0;
    wp1.twist.twist.angular.z = 0;

// corner point:
    wp2.pose.pose.position.x = -1.5;
    wp2.pose.pose.position.y = -1.5;
    wp2.pose.pose.position.z = 1.5;
    wp2.pose.pose.orientation.z = -M_PI/2;
    wp2.twist.twist.linear.x = 0;
    wp2.twist.twist.linear.y = - x_y_vel + mas_menos; 
    wp2.twist.twist.linear.z = 0;
    wp2.twist.twist.angular.z = z_vel_ang;

// middle point:
    wp3.pose.pose.position.x = 0;
    wp3.pose.pose.position.y = -1.5;
    wp3.pose.pose.position.z = 1.5;
    wp3.pose.pose.orientation.z = 0; 
    wp3.twist.twist.linear.x = x_y_vel + mas_menos;
    wp3.twist.twist.linear.y = 0; 
    wp3.twist.twist.linear.z = 0;
    wp3.twist.twist.angular.z = 0;

// corner point:
    wp4.pose.pose.position.x = 1.45;
    wp4.pose.pose.position.y = -1.5;
    wp4.pose.pose.position.z = 1.5;
    wp4.pose.pose.orientation.z = 0; 
    wp4.twist.twist.linear.x = x_y_vel - mas_menos;
    wp4.twist.twist.linear.y = 0; 
    wp4.twist.twist.linear.z = 0;
    wp4.twist.twist.angular.z = z_vel_ang;

// middle point:
    wp5.pose.pose.position.x = 1.5;
    wp5.pose.pose.position.y = 0;
    wp5.pose.pose.position.z = 1.5;
    wp5.pose.pose.orientation.z = M_PI/2; 
    wp5.twist.twist.linear.x = 0;
    wp5.twist.twist.linear.y = x_y_vel + mas_menos; 
    wp5.twist.twist.linear.z = 0;
    wp5.twist.twist.angular.z = 0;

// corner point:
    wp6.pose.pose.position.x = 1.5;
    wp6.pose.pose.position.y = 1.4;
    wp6.pose.pose.position.z = 1.5;
    wp6.pose.pose.orientation.z = M_PI/2; 
    wp6.twist.twist.linear.x = 0;
    wp6.twist.twist.linear.y = x_y_vel - mas_menos; 
    wp6.twist.twist.linear.z = -z_vel_lin;
    wp6.twist.twist.angular.z = z_vel_ang;

// middle point:
    wp7.pose.pose.position.x = 0;
    wp7.pose.pose.position.y = 1.5;
    wp7.pose.pose.position.z = 1.5;
    wp7.pose.pose.orientation.z = M_PI; 
    wp7.twist.twist.linear.x = -x_y_vel - mas_menos;
    wp7.twist.twist.linear.y =  0; 
    wp7.twist.twist.linear.z = 0;
    wp7.twist.twist.angular.z = 0;


    traj_wp = {wp0, wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8};


    // Definition of the trajectory beginning, end and intermediate constraints
    mav_trajectory_generation::Vertex start(dimension), middle_points(dimension), end(dimension);

    drawMarkerArray(traj_wp, 1, 0);
    
    // deleting the first and last entry of the recived trajectory waypoints 
    // becaue they are already considered in start and goal vertices
    traj_wp.erase(traj_wp.begin());
    traj_wp.erase(traj_wp.end() - 1);

    // make start point: 
    start.makeStartOrEnd(Eigen::Vector4d(odom_info.pose.pose.position.x,
                                         odom_info.pose.pose.position.y,
                                         odom_info.pose.pose.position.z, 
                                         -M_PI/2), 
                                         derivative_to_optimize);

    std::cout << "\n---\n";
    std::cout << "Start for traj. x, y, z: \t"  << odom_info.pose.pose.position.x << ", " 
                                                << odom_info.pose.pose.position.y << ", " 
                                                << odom_info.pose.pose.position.z << std::endl;
    vertices.push_back(start);


    //for (const auto& waypoint : traj_wp) {
    for (const auto& waypoint : traj_wp) {

        mav_trajectory_generation::Vertex middle_points(dimension);
        
        // Extract the position from the waypoints
        Eigen::Vector4d pose_cnstr(
            waypoint.pose.pose.position.x,
            waypoint.pose.pose.position.y,
            waypoint.pose.pose.position.z,
            waypoint.pose.pose.orientation.z // we use it now as if it were yaw and not a quaternion
        
        );

        // Add the position constraint for the waypoint
        middle_points.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pose_cnstr);

        // Extract the velocity from the waypoints
        Eigen::Vector4d vel_cnstr(  
            waypoint.twist.twist.linear.x,
            waypoint.twist.twist.linear.y,
            waypoint.twist.twist.linear.z,
            waypoint.twist.twist.angular.z); 
        
        // Add the velocity constraint for the waypoint
        middle_points.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel_cnstr);
    
        // Add the vertex to the trajectory vertices
        vertices.push_back(middle_points);

        // DEBUG PRINT:
        std::cout   << waypoint_cntr << ". Waypoint pose x, y, z & yaw: \t" 
                    << waypoint.pose.pose.position.x << ", "
                    << waypoint.pose.pose.position.y << ", "
                    << waypoint.pose.pose.position.z << " & "
                    << waypoint.pose.pose.orientation.z << "\n";

        std::cout   << waypoint_cntr << ". Waypoint vel. x, y, z & yaw: \t" 
                    << waypoint.twist.twist.linear.x  << ", "
                    << waypoint.twist.twist.linear.y << ", "
                    << waypoint.twist.twist.linear.z << " & "
                    << waypoint.twist.twist.angular.z << "\n\n";

        waypoint_cntr++;

    }

    // make end point, z + 1 meter: 
    end.makeStartOrEnd(Eigen::Vector4d( goal.pose.position.x,
                                        goal.pose.position.y,
                                        goal.pose.position.z, 
                                        M_PI), // M_PI/4), 
                                        derivative_to_optimize);
    vertices.push_back(end);
    std::cout << "Goal for traj. x, y, z: \t" << goal.pose.position.x << ", " 
                                            << goal.pose.position.y << ", " 
                                            << goal.pose.position.z << std::endl;
    std::cout << "\n---\n";

    
    // TODO understand this:
    // Provide the time constraints on the vertices
    //Automatic time computation 
    std::vector<double> segment_times;
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
    
    if(curr_state + 1 <= traj_states.size()){  // + 1 because if not it sends one 0 0 0 pose...
        cmd_vel.linear.x = traj_states[curr_state].velocity_W.x();
        cmd_vel.linear.y = traj_states[curr_state].velocity_W.y();
        cmd_vel.linear.z = traj_states[curr_state].velocity_W.z();
        cmd_vel.angular.x = traj_states[curr_state].angular_velocity_W.x();
        cmd_vel.angular.y = traj_states[curr_state].angular_velocity_W.y();
        cmd_vel.angular.z = traj_states[curr_state].angular_velocity_W.z();

        cmd_pose.pose.position.x = traj_states[curr_state].position_W.x();
        cmd_pose.pose.position.y = traj_states[curr_state].position_W.y();
        cmd_pose.pose.position.z = traj_states[curr_state].position_W.z();
        cmd_pose.pose.orientation.x = traj_states[curr_state].orientation_W_B.x();
        cmd_pose.pose.orientation.y = traj_states[curr_state].orientation_W_B.y();
        cmd_pose.pose.orientation.z = traj_states[curr_state].orientation_W_B.z();
        cmd_pose.pose.orientation.w = traj_states[curr_state].orientation_W_B.w();


        // Publish the command
        pose_pub.publish(cmd_pose);
        vel_pub.publish(cmd_vel);


    }else{
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        cmd_pose.pose.position.x = traj_states[traj_states.size()-1].position_W.x();
        cmd_pose.pose.position.y = traj_states[traj_states.size()-1].position_W.y();
        cmd_pose.pose.position.z = traj_states[traj_states.size()-1].position_W.z();
        cmd_pose.pose.orientation.x = traj_states[traj_states.size()-1].orientation_W_B.x();
        cmd_pose.pose.orientation.y = traj_states[traj_states.size()-1].orientation_W_B.y();
        cmd_pose.pose.orientation.z = traj_states[traj_states.size()-1].orientation_W_B.z();
        cmd_pose.pose.orientation.w = traj_states[traj_states.size()-1].orientation_W_B.w();

        pose_pub.publish(cmd_pose);
        vel_pub.publish(cmd_vel);
        
    }
        curr_state++;
        // if(curr_state == traj_states.size()- 45) curr_state = 45;
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
    
    ros::init(argc, argv, "traj_generator");
    ros::AsyncSpinner ich_spinne(1);
    ich_spinne.start();
    ros::NodeHandle node_handle("~");
    poly_traj_plan ptp(std::ref(node_handle));
    int current_traj_state = 0;

    // to-do -> ask user if he likes voxblox or hardcoded trajectories
    bool traj_checker = false;
    do{
        // run the navigation node and get the trajectory    
        int nav_func_checker = ptp.run_navigation_node();

        if (nav_func_checker == 0) ROS_INFO("Navigation succesfully");
        else if (nav_func_checker == -1 ){
            ROS_ERROR("Navigation error");
            return -1;
        }

        // checking the trajectory when the waypoints are produced by voxblox
        // since it's RRT (Random) it couldn't be feasible
        if (ptp.vxblx_active == true){
            if (check_trajectory() == 0){
                    traj_checker = true;
            }
        }
        else if (ptp.vxblx_active == false) traj_checker = true;
    
    } while (traj_checker == false);
      
    
    // sending vel commands to the real drone
    if (traj_checker == true)
    {
        while(ros::ok()){
            
            ptp.send_vel_commands();
            ros::Duration(ptp.sampling_interval).sleep();
            // ros::Duration(0.05).sleep();
        }
    }

    return 0;

}


