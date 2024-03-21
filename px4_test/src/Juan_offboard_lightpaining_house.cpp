#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

//------CALLBACKS-----
//MAVROS STATE
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
//CURRENT POSE
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;

}


//------OTHER FUNCTIONS-----------
//CALC DISTANCE (3D) between 2 points
double calcDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal){
            double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
            double euc = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
            //ROS_INFO_STREAM("Distance to goal: " << euc);
            return euc;
        }





//--------------------
//-------MAIN---------
int main(int argc, char **argv)
{
    //ROS INIT
    ros::init(argc, argv, "offb_node_loop");
    //node handle
    ros::NodeHandle nh;


    //---------SUBSCRIBERS-----------
    //MAVROS STATE SUBS
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //OPTITRACK POSE SUBS
    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/optitrack/pose",10, pose_cb);
    //---------END OFSUBSCRIBERS-----------



    //---------PUBLISHERS-----------
    //MAVROS SETPOINT (LOCAL REFEFERENCE)
    ros::Publisher cmd_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //---------END OF PUBLISHERS-----------



    //---------SERVICE CLIENTS-----------
    //MAVROS FRAME
    ros::ServiceClient set_cmd_frame = nh.serviceClient<mavros_msgs::SetMavFrame>
            ("/mavros/setpoint_position/mav_frame");
    //MAVROS ARMING
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //MAVROS SET MODE
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
     //---------END OF SERVICE CLIENTS-----------



    //For TF transformations
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_(tf_buffer_, nh);


    //the setpoint publishing rate MUST be FASTER THAN 2HZ
    //20HZ
    ros::Rate rate(20.0);
    //1HZ
    ros::Rate wait_rate(1.0);

    // WAIT FOR FCU CONNECTION
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    //-------------------TRAJECTORY VECTOR-----------------

    //GLOBAL FRAME
    //DECLARE ARRAY size 4 for example
    geometry_msgs::Pose trajectory[14];
    
    //IN GLOBAL REFERENCE ODOM
    std::string frame_name = "odom";
    //build the PoseStamped message
    geometry_msgs::PoseStamped pos_globalframe;
    int wp = 0;

    //0) 
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = 1;
    pos_globalframe.pose.position.z = 1;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    
    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;


    //1)  
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = 1;
    pos_globalframe.pose.position.z = 2;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    
    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;

    //2)  
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = 0;
    pos_globalframe.pose.position.z = 3.2;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    
    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;


    //3)  
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = -1;
    pos_globalframe.pose.position.z = 2;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    
    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;

    //4)  
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = -1;
    pos_globalframe.pose.position.z = 1;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    
    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;

    //5)  
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = 1;
    pos_globalframe.pose.position.z = 2;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    
    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;

    //6)  
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = -1;
    pos_globalframe.pose.position.z = 2;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    
    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;

    //7)  
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = 1;
    pos_globalframe.pose.position.z = 1;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    
    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;

    //8)  
    pos_globalframe.pose.position.x = 0;
    pos_globalframe.pose.position.y = -1;
    pos_globalframe.pose.position.z = 1;
    pos_globalframe.pose.orientation.x = 0;
    pos_globalframe.pose.orientation.y = 0;
    pos_globalframe.pose.orientation.z = 0;
    pos_globalframe.pose.orientation.w = 1;
    pos_globalframe.header.frame_id = frame_name;
    


    //store it in trajectory array
    trajectory[wp].position = pos_globalframe.pose.position;
    wp += 1;

    //-------------------END OF TRAJECTORY VECTOR-----------------



    
    //TRANFORM FRAME
    //wait_rate.sleep();
    geometry_msgs::TransformStamped transform_msg;

    //declare t_out TO PUBLISH LATER
    geometry_msgs::PoseStamped cmd_pos;
    
    


    //------------SEND INITIAL 100 POINTS TO CHANGE TO OFFBOARD------------------
    

    for(int i = 100; ros::ok() && i > 0; --i){
        
        //Publish CMD_POS
        cmd_pos_pub.publish(current_pose);
        ros::spinOnce();
        rate.sleep();
    }
    //------------END OF SEND INITIAL 100 POINTS TO CHANGE TO OFFBOARD------------------

    
    
    // --------ARMING AND OFFBOARD --------------
    //REQUEST OFFBOARD FLIGHT MODE
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //REQUEST ARM DRONE
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //STORE TIME NOW as last_request
    ros::Time last_request = ros::Time::now();
    
    //LOOP UNTIL POSITION AND  ARMED
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
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
            offboard = true;
        }else{
           
            if(!current_state.armed){
                ROS_INFO("NOT ARMED, ARM FIRST");
            }
        }
    }while(ros::ok() && !offboard);
    // --------END OF ARMING AND OFFBOARD --------------




    // --------NAVIGATE WAYPOINTS --------------
    //initialize goal counter
    int goal_i = 0;
    //initialize goal
    geometry_msgs::Point goal;
    //first wp
    goal = trajectory[goal_i].position;
    geometry_msgs::PoseStamped cmd_pos_global;
    geometry_msgs::PoseStamped cmd_pos_global_ned;


    //MAIN LOOP OF WP NAVIGATION
    while(ros::ok()){
        

        //CREATE CMD_POS WITH TRAJECTORY (GLOBAL)
        cmd_pos_global.pose = trajectory[goal_i];
        cmd_pos_global.header.frame_id = "odom";
        cmd_pos_global.header.stamp = ros::Time::now();
 

        //TRANSFORM TO LOCAL
        bool tf_recieved = false;       
        while(!tf_recieved){

            try{
                //transform_msg between TARGET_FRAME, SOURCE_FRAME , TIME
                //Target--> SOURCE
                //20240209 it has wrong XY axis, swap them... orientation (in z) is still wrong
                
                transform_msg = tf_buffer_.lookupTransform( "odom", "odom_ned", ros::Time(0));

                //T_IN, T_OUT, transform_msg
                //global->local
                tf2::doTransform(cmd_pos_global, cmd_pos_global_ned, transform_msg);
                
                //cmd_pos frame_id to dummy
                cmd_pos.header.frame_id = "odom_ned";
                tf_recieved = true;

                //for debugging
                //ROS_INFO_STREAM("cmd_pos: "<< cmd_pos);
                //ROS_INFO_STREAM("cmd_pos_global: "<< cmd_pos_global);


            } catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
        }  

        //PUBLISH WP (in local)
        cmd_pos_pub.publish(cmd_pos_global_ned);    

        
        //CALC 3D DISTANCE CURRENT TO GOAL 
        while(calcDistance(current_pose.pose, goal) < 0.2){
           

            //print goal reached and wait 3 sec
            switch(goal_i)
                {
                case 0:     
                ROS_INFO("Goal reached = 0");
                //ros::Duration(3.0).sleep();
                break;

                case 1: 
                ROS_INFO("Goal reached = 1");
                //ros::Duration(3.0).sleep();
                break;

                case 2: 
                ROS_INFO("Goal reached = 2");
                //ros::Duration(3.0).sleep();
                break;

                case 3: 
                ROS_INFO("Goal reached = 3");
                //offb_set_mode.request.custom_mode = "LAND";
                //ros::Duration(3.0).sleep();
                break;

                case 4: 
                ROS_INFO("Goal reached = 4");
                //offb_set_mode.request.custom_mode = "LAND";
                //ros::Duration(3.0).sleep();
                break;


                case 5: 
                ROS_INFO("Goal reached = 5");
                //offb_set_mode.request.custom_mode = "LAND";
                //ros::Duration(3.0).sleep();
                break;
                
                case 6: 
                ROS_INFO("Goal reached = 6");
                //offb_set_mode.request.custom_mode = "LAND";
                //ros::Duration(3.0).sleep();
                break;

                case 7: 
                ROS_INFO("Goal reached = 7");
                //offb_set_mode.request.custom_mode = "LAND";
                //ros::Duration(3.0).sleep();
                break;

                case 8: 
                ROS_INFO("Goal reached = 8");
                //Restart 
                goal_i=-1;
                //offb_set_mode.request.custom_mode = "LAND";
                //ros::Duration(3.0).sleep();
                break;



                default: ROS_INFO("wrong!");
            }

            //MOVE ON TO NEXT GOAL
            goal_i++;
            goal = trajectory[goal_i].position;
        

      
        }

        //ROS SPIN
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
