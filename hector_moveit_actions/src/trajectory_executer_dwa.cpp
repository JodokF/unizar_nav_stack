// calculates dwa3D


#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <hector_moveit_actions/ExecuteDroneTrajectoryAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <hector_uav_msgs/PoseAction.h>
#include <hector_uav_msgs/EnableMotors.h>

#include <cmath>
#define _USE_MATH_DEFINES

#define MAX_SPEED 1.5
#define EPSILON 1e-4
// DWA
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <array>

#define PI 3.14159265

const double T=0.1; //Periodo de control [s]
const double vx_max= 1;
const double vz_max= 0.5;
const double w_max = PI/9; // 30º --> 15º ?? 
const double paso_v= 0.05; //Resolución de discretización del espacio de búsqueda en xz[m/s]
const double paso_w = PI/36; // Resolucion de discretizacion del espacio de busqueda en yaw (5º)
const double aLin = 1.0; // Aceleracion lineal máxima [m/ss]
const double aAng = PI/1.8; // Aceleracion angular maxima [rad/ss] 10º
const double radio_dron = 0.5; //[m]
const double velodyne_max_range = 10; //Rango máximo de sensor velodyne [m]. Es el mismo definido en VLP-16.urdf.xacro
const double res_azim = 30 * PI/180; //Resolución angular [°] en azimuth 30
const double res_elev = 30 * PI/180; //Resolución angular [°] en elevation 30
const int cols = 8, filas_tot = ((2*aLin*T/paso_v) + 1)*((2*aAng*T/paso_w) + 1)*((2*aLin*T/paso_v) + 1);
const int iter_obs = 10; //iter_obs*T = Periodo [s] de actualización de obstáculos = Horizonte temporal en la predicción de posición
int iter_update = 150;

class Dwa3d {
    private:
        typedef actionlib::SimpleActionServer<hector_moveit_actions::ExecuteDroneTrajectoryAction> TrajectoryActionServer;
        typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> OrientationClient;
        typedef hector_moveit_actions::ExecuteDroneTrajectoryResult Result;
        typedef hector_moveit_actions::ExecuteDroneTrajectoryFeedback Feedback;
        ros::NodeHandle nh_;
        ros::Publisher vel_pub, predicted_pose_pub, discarded_poses_pub;
        ros::Subscriber pose_sub;
        TrajectoryActionServer server_;
        OrientationClient orientation_client_;
        std::string action_name;

        geometry_msgs::Twist empty,cmd;
        std::vector<geometry_msgs::Pose> trajectory;
        geometry_msgs::Pose last_pose;
       
        Feedback feedback_;
        Result result_;
        bool success, executing;

        // DWA
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        ros::Subscriber obs_sub;
        ros::Publisher abs_points_publisher;
        sensor_msgs::PointCloud2 abs_cloud, rel_cloud;

        double ALFA, Ky, Kz, BETA, GAMMA;
        double NEAR = 0; // Parametro para momento final de acercamiento al objetivo
        double step;
        std::array<double,6> Vs = { 0, vx_max, -w_max, w_max, -vz_max, vz_max }; // Espacio de velocidades maximas
        bool lecturaObstaculos = true;

    public:
        Dwa3d(std::string name) : action_name(name), tf_listener_(tf_buffer_), 
            server_(nh_,name,boost::bind(&Dwa3d::executeCB,this,_1),false),
            orientation_client_("/action/pose",true){
                orientation_client_.waitForServer();
                empty.linear.x = 0;empty.linear.y = 0; empty.linear.z = 0;
                empty.angular.x = 0;empty.angular.y = 0;empty.angular.z = 0;
                obs_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/merged_clouds",1,&Dwa3d::obsCallback,this); //velodyne_points
                vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_control",10);
                predicted_pose_pub = nh_.advertise<visualization_msgs::Marker>("/predicted_pose",10);
                discarded_poses_pub = nh_.advertise<visualization_msgs::Marker>("/discarded_poses_marker",10);
                pose_sub = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&Dwa3d::poseCallback,this);
                abs_points_publisher =  nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_transformed", 3, true); //true para que aparezca en Rviz
                ros::ServiceClient enable_motors = nh_.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
                hector_uav_msgs::EnableMotors srv;
                srv.request.enable = true;
                nh_.param("/ALFA",ALFA, 0.3);
                nh_.param("/Ky",Ky, 0.5);
                nh_.param("/Kz",Kz, 0.5);
                nh_.param("/BETA",BETA, 0.6);
                nh_.param("/GAMMA",GAMMA, 0.1);
                nh_.param("/goal_step", step, 0.5);
                nh_.param("/iter_update", iter_update, 150);
                if(enable_motors.call(srv)){
                    if(srv.response.success)
                        ROS_INFO("Motors are enabled");
                }
                success = true;
                executing = false;
                server_.start();
            }
        void executeCB(const hector_moveit_actions::ExecuteDroneTrajectoryGoalConstPtr &goal){
            ros::WallTime time_start_, time_end_;
            executing = true;
            trajectory = goal->trajectory;
            int goal_i = 1;
            geometry_msgs::Point subgoal;
            geometry_msgs::Twist cmd_vel;
            std::array<std::array<double,cols>,filas_tot> comp_eval, comp_eval_norm; // Para cada posible velocidad de la ventana
    	    std::array<double,filas_tot> G;	// Puntuaciones
            std::array<double,6> Vd, Vsd;
            geometry_msgs::Pose pose_predicted;
            std::array<double,3> vel_actual={0,0,0};
            ros::Rate loop_rate(1/T);
            int iteraciones = 0, iter2update = 0;
            ros::WallTime start_, end_;
            double headingTime = 0.0, distTime = 0.0, goalDistTime = 0.0;
            subgoal = trajectory[1].position;
            while(goalDistance(last_pose, subgoal) < 1){
                goal_i++;
                subgoal = trajectory[goal_i].position;
            }
            ROS_INFO("DWA Goal: (%f, %f, %f)", subgoal.x, subgoal.y, subgoal.z);
            while(ros::ok()){ // Hasta que se llegue al objetivo o no se pueda continuar
                time_start_ = ros::WallTime::now();
                if ((iteraciones % iter_obs) == 0) lecturaObstaculos = true;
                int filas_eval = 0, filas_x = 0, filas_w = 0, filas_z = 0; //Filas evaluadas de la ventana
                Vd = { vel_actual[0]-aLin*T, vel_actual[0]+aLin*T,
                    vel_actual[1]-aAng*T, vel_actual[1]+aAng*T,
                    vel_actual[2]-aLin*T, vel_actual[2]+aLin*T };  //Espacio ventana dinámica
			    Vsd = { std::max (Vs[0], Vd[0]), std::min (Vs[1], Vd[1]),
                        std::max (Vs[2], Vd[2]), std::min (Vs[3], Vd[3]),
                        std::max (Vs[4], Vd[4]), std::min (Vs[5], Vd[5]) };  //Vsd = intersección Vs con Vd
                // Inicializacion de los vectores
                for(int i=0; i<filas_tot*cols; i++) comp_eval[i/cols][i%cols] = 0;
                for(int i=0; i<filas_tot; i++) G[i]=0;

                visualization_msgs::Marker discarded_poses_debug;
                discarded_poses_debug.type = visualization_msgs::Marker::SPHERE_LIST;
                discarded_poses_debug.action = visualization_msgs::Marker::MODIFY;
                discarded_poses_debug.color.a = 0.1;
                discarded_poses_debug.color.r = 1;
                discarded_poses_debug.color.b = 1;
                discarded_poses_debug.scale.x = 0.1;
                discarded_poses_debug.scale.y = 0.1;
                discarded_poses_debug.scale.z = 0.1;
                discarded_poses_debug.header.frame_id = "world";

                // Recorrer el espacio de búsqueda Vsd               
                for (double vx=Vsd[0]; vx<=Vsd[1]; vx+=paso_v){
                    vx=round(vx/paso_v)*paso_v;// Redondeo de las velocidades
                    for (double w=Vsd[2]; w<=(Vsd[3]-Vsd[3]*vx/(4*std::max(0.01,Vsd[1]))); w+=paso_w){
                        w=round(w/paso_w)*paso_w;
                        for (double vz=Vsd[4]; vz<=Vsd[5]; vz+=paso_v){
                            vz=round(vz/paso_v)*paso_v;
                            if(goal_i < trajectory.size()-1 && vx < 0.2 && vz < 0.0) continue;
                            // Posicion final tras aplicar las velocidades
                            pose_predicted = simubot(vx, w, vz, iter_obs* T);//iter_obs*T
                            // Distancia al obstaculo mas cercano (en la posicion final)
                            double R = vx/w;
                            double diff_w = w - vel_actual[1];
                            double dist = R * diff_w;
                            //start_ = ros::WallTime::now();
                            double min_dist = calcDist(vz, pose_predicted, abs_cloud);
                            // std::cout << "Min Dist: " <<  min_dist << std::endl;
                            //end_ = ros::WallTime::now(); 
                            //distTime += (end_ - start_).toNSec() * 1e-6;
                            double v=sqrt(vx*vx + vz*vz); // Velocidad total lineal dentro de Vr (dentro de Vsd y no llevan a colision)
                            
                            {
                            geometry_msgs::Point point;
                            point.x = pose_predicted.position.x;
                            point.y = pose_predicted.position.y;
                            point.z = pose_predicted.position.z;
                            discarded_poses_debug.points.push_back(point);
                            }


                            if (v <= sqrt(2*aLin*min_dist) || BETA == 0){ // w no deberia en ningun caso llevar a colision!!
                                //start_ = ros::WallTime::now();
                                comp_eval[filas_eval][0] = calcYawHeading(pose_predicted, subgoal);
                                comp_eval[filas_eval][1] = calcZHeading(pose_predicted, subgoal);
                                //end_ = ros::WallTime::now(); 
                                //headingTime += (end_ - start_).toNSec() * 1e-6;
                                comp_eval[filas_eval][2] = min_dist;
                                
                                comp_eval[filas_eval][4] = vx;
                                comp_eval[filas_eval][5] = w;
                                comp_eval[filas_eval][6] = vz;
                                //start_ = ros::WallTime::now();
                                comp_eval[filas_eval][7] = goalDistance(pose_predicted, subgoal);
                                if(comp_eval[filas_eval][0] > 0.9 && comp_eval[filas_eval][1] < 0.5) comp_eval[filas_eval][3] = vx;
                                // Evitar velocidades negativas en z cuando no hay velocidad en x
                                // NO vemos el suelo porque lo tapa el dron -> tiende a chocar con el suelo
                                else if (vx == 0 && vz == 0 && fabs(w) < w_max/2) comp_eval[filas_eval][3] = fabs(w)/4;
                                else comp_eval[filas_eval][3] = v;
                                //end_ = ros::WallTime::now(); 
                                //goalDistTime += (end_ - start_).toNSec() * 1e-6;
                                filas_eval++;
                            } filas_z++;
                        } filas_w++;
                    } filas_x++;
                } 

                //DEBUG
                discarded_poses_debug.header.stamp = ros::Time::now();
                discarded_poses_pub.publish(discarded_poses_debug);

                if ( filas_eval!=0 ) {
		    	  //Cálculo de máximos en la matriz de evaluación
		    	  double yaw_head_max, z_head_max, dist_max, vel_max;
		    	  yaw_head_max=comp_eval[0][0];z_head_max=comp_eval[0][1]; dist_max=comp_eval[0][2]; vel_max=comp_eval[0][3];
		    	  for (int i=0; i< filas_eval; i++)  {
		    		  if ( comp_eval[i][0]>yaw_head_max ) yaw_head_max= comp_eval[i][0];
		    		  if ( comp_eval[i][1]>z_head_max ) z_head_max= comp_eval[i][1];
		    		  if ( comp_eval[i][2]>dist_max ) dist_max= comp_eval[i][2];
		    		  if ( comp_eval[i][3]>vel_max ) vel_max= comp_eval[i][3];
		          }
		    	  double max_G = G[0]; int ind = 0;
		    	  for (int i=0; i< filas_eval; i++) { // Normalizacion y calculo de G (Nos quedamos con el maximo)
		    		  comp_eval_norm[i][0]=comp_eval[i][0]/yaw_head_max;
		    		  comp_eval_norm[i][1]=comp_eval[i][1]/z_head_max;
		    		  comp_eval_norm[i][2]=comp_eval[i][2]/dist_max;
		    		  comp_eval_norm[i][3]=comp_eval[i][3]/vel_max;
		    		  G[i]=  (Ky*ALFA*comp_eval_norm[i][0]  +  Kz*ALFA*(1-comp_eval_norm[i][1])  +  BETA*comp_eval_norm[i][2]  +  GAMMA*comp_eval_norm[i][3])*(1-NEAR)
							 + NEAR*(2-comp_eval_norm[i][7])/2;
		    		  if(G[i]>max_G){
		    			  max_G = G[i];
		    			  ind = i;
		    		  }
		    	  }
		    	  vel_actual[0]=round(comp_eval[ind][4]/paso_v)*paso_v; //vx óptimo
		    	  vel_actual[2]=round(comp_eval[ind][6]/paso_v)*paso_v; //vz óptimo
		    	  vel_actual[1]=round(comp_eval[ind][5]/paso_w)*paso_w; //w óptimo

                time_end_ = ros::WallTime::now();
                double execution_time = (time_end_ - time_start_).toNSec() * 1e-6;
                ROS_INFO_STREAM("Alba DWA computation time: " << execution_time);


                //DEBUG
                pose_predicted = simubot(vel_actual[0], vel_actual[1], vel_actual[2], iter_obs*T);//iter_obs*T
                visualization_msgs::Marker pose_debug;
                pose_debug.pose = pose_predicted;
                //pose_debug.type = visualization_msgs::Marker::MESH_RESOURCE;
                //pose_debug.mesh_resource = "package://hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae";
                pose_debug.type = visualization_msgs::Marker::SPHERE;
                pose_debug.action = visualization_msgs::Marker::MODIFY;
                pose_debug.color.a = 0.6;
                pose_debug.color.g = 1;
                pose_debug.scale.x = 0.4;
                pose_debug.scale.y = 0.4;
                pose_debug.scale.z = 0.4;
                pose_debug.header.frame_id = "world";
                pose_debug.header.stamp = ros::Time::now();
                predicted_pose_pub.publish(pose_debug);


                  /******************************************************** DEBUG INFO *****************************************************/
                    if((iteraciones % 10) == 0){
                        std::cout << "Goal index: " << goal_i << " | Trajectory size: " << trajectory.size() << std::endl;
                        double d_robot_goal = goalDistance(last_pose, subgoal);
                        std::cout << "Goal distance: " << d_robot_goal << std::endl;
                         if(goal_i == trajectory.size()-1) std::cout << "ULTIMO OBJETIVO" << std::endl;
		    		  tf::Quaternion q1(last_pose.orientation.x, last_pose.orientation.y, last_pose.orientation.z, last_pose.orientation.w);
					  tf::Matrix3x3 m1(q1);
					  double roll, pitch,yaw;
					  m1.getRPY(roll, pitch, yaw);
		    		  std::cout << "/ x= " << last_pose.position.x <<" / y = " << last_pose.position.y << " / z = " << last_pose.position.z << std::endl;
                      //std::cout << " / roll: " << rad2deg(roll) <<" / pitch: " << rad2deg(pitch) << " / yaw: " << rad2deg(yaw) << std::endl;
		    		  std::cout << "vel x = " << vel_actual[0] << "// vel z = " << vel_actual[2] << "// w = " << vel_actual[1] << std::endl;
                      std::cout << "yaw_xy = " << comp_eval[ind][0] << " || dist_max = " << comp_eval[ind][2] << " || vel_max = " << comp_eval[ind][3] << std::endl;
                      //std::cout << "G: " << max_G << std::endl;
                      std::cout << "pose predicted: (" << pose_predicted.position.x << " ," << pose_predicted.position.y << " ," << pose_predicted.position.z << ")" << std::endl;
                      //std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
					  
                        //std::cout << "min_x\t|\tmax_x\t|\tmin_w\t|\tmax_w\t|\tmin_z\t|\tmax_z" << std::endl;
                        //std::cout << Vsd[0] << "\t|\t" << Vsd[1] << "|\t" << Vsd[2] << "|\t" << Vsd[3] << "|\t" << Vsd[4] << "|\t" << Vsd[5] << std::endl;
                        //std::cout << "iteracion: " << iteraciones << std::endl;
                        //ROS_INFO_STREAM("Heading time (ms): " << headingTime);
                        //ROS_INFO_STREAM("Distance time (ms): " << distTime);
                        //ROS_INFO_STREAM("Goal Distance time (ms): " << goalDistTime);
                        
                        //std::cout << "/ x= " << last_pose.position.x <<" / y = " << last_pose.position.y << " / z = " << last_pose.position.z << std::endl;
		    	    }
                
                } else { // COLISION
		    	//   ROS_INFO_STREAM("Todas las posibles combinaciones [vx,w,vz] llevan a colision con obstaculo");
		    	//   std::cout<<"Espacio de búsqueda: "<<Vsd[0]<<", "<<Vsd[1]<<", "<<Vsd[2]<<", "<<Vsd[3]<<", "<<Vsd[4]<<", "<<Vsd[5]<<std::endl;
				//   std::cout << "vel x = " << vel_actual[0] << "// vel z = " << vel_actual[2] << "// w = " << vel_actual[1] << std::endl;
		    	  geometry_msgs::Twist empty;
                  vel_pub.publish(empty);
                  result_.result_code = Result::COLLISION_IN_FRONT;
                  server_.setPreempted(result_);
                  return;
		        } 
                cmd_vel.linear.x = vel_actual[0];
		        cmd_vel.angular.z = vel_actual[1];
		        cmd_vel.linear.z = vel_actual[2] + 0.12; //+0.12 para compensación del efecto de la gravedad
		        if (cmd_vel.linear.z > vz_max) cmd_vel.linear.z = vz_max;
                vel_pub.publish(cmd_vel);
                double d_robot_goal = goalDistance(last_pose, subgoal);
                if(goal_i == trajectory.size()-1){ // Ultimo objetivo
                    if (d_robot_goal < 3){ NEAR = 0.7; ALFA=0.5; 	BETA=0.25; 		GAMMA=0.25;		} // Cerca del objetivo final
				    if(d_robot_goal < step){ // Consideramos objetivo
				    	ROS_INFO_STREAM("Objetivo alcanzado");
				    	std::cout<<"...aterrizando..."<<std::endl;
                        geometry_msgs::Twist empty;
                        vel_pub.publish(empty);
				        result_.result_code = Result::SUCCESSFUL;
                        server_.setSucceeded(result_);
                        return;
				    }
                } else if (d_robot_goal < step || goalDistance(last_pose, trajectory[goal_i+1].position) < d_robot_goal) {
                    goal_i++;
                    subgoal = trajectory[goal_i].position;
                    while(goalDistance(last_pose, subgoal) < 1){
                        goal_i++;
                        subgoal = trajectory[goal_i].position;
                    }
				    std::cout << "------------ NEXT subgoal: x=" << subgoal.x << "//y=" << subgoal.y << "//z=" << subgoal.z << " ------------" << std::endl;
                    iter2update = 0;
                } else if (iter2update >= iter_update) {// Else if se queda atascado
                    ROS_INFO_STREAM("Replanning ...");
                    geometry_msgs::Twist empty;
                    empty.linear.z = 0.12; // Mantener altura
                    vel_pub.publish(empty);
                    result_.result_code = Result::COLLISION_IN_FRONT;
                    server_.setPreempted(result_);
                    return;
                }
                iteraciones++;
                iter2update++;
                //ROS_INFO("Iteraciones %d", iteraciones);
                ros::spinOnce();
                loop_rate.sleep();
            }
           
        }

        void idle(){
            while(ros::ok()){
                ros::spinOnce();
            }
        }

        void actionCallback(const hector_uav_msgs::PoseFeedbackConstPtr& feedback,geometry_msgs::Pose& p){
            double euler_distance = pow(p.position.x - feedback->current_pose.pose.position.x,2) + pow(p.position.y - feedback->current_pose.pose.position.y,2)
                                    + pow(p.position.z - feedback->current_pose.pose.position.z,2);
            euler_distance = sqrt(euler_distance);
            if(euler_distance < 0.15)
                orientation_client_.cancelGoal();
        }
        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
        {
            last_pose = msg->pose.pose;
        }
        
        void obsCallback(const sensor_msgs::PointCloud2 cloud){
            if (lecturaObstaculos) {
                lecturaObstaculos = false;

                geometry_msgs::TransformStamped transform;
                try{
                    transform = tf_buffer_.lookupTransform("world", "os0_lidar", //camera_depth_optical_frame
                    ros::Time(0), ros::Duration(1.0));
                } catch (tf2::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }
                rel_cloud = cloud;
                tf2::doTransform (rel_cloud, abs_cloud, transform);
                
                abs_points_publisher.publish(abs_cloud);// Publicación de la nube. Para visualización
            }
        }
        double norm_rad(double rad){ // rad en (-pi,pi]
            while(rad <= -PI) rad += 2*PI; // (-pi, inf)
            while (rad > PI) rad -= 2*PI;  // (-pi, pi]
            return rad;
        }
        double rad2deg(double rad){ return rad*180/PI;}
        
        double calcDist (double vz, geometry_msgs::Pose pose, sensor_msgs::PointCloud2 obstaculos){
            const double radio_dron = 0.4; //[m]
            const double res_azim = 30 * PI/180; //Resolución angular [°] en azimuth
            const double res_elev = 30 * PI/180; //Resolución angular [°] en elevation
            const double velodyne_max_range = 10; //Rango máximo de sensor velodyne [m]. Es el mismo definido en VLP-16.urdf.xacro
            double min_dist = velodyne_max_range, azimuth, r, elevation, dist, dx, dy, dz, roll, pitch, yaw;
            yaw = pose.orientation.z;
            pitch = pose.orientation.x;
            // Para cada punto en la nube, calcular distancia euclidea
            for (sensor_msgs::PointCloud2Iterator<float> it(obstaculos, "x"); it != it.end(); ++it) {
                dx = it[0] - pose.position.x; dy = it[1] - pose.position.y; dz = it[2] - pose.position.z; // Distancia en cada coordenada
                r = sqrt(dx*dx + dy*dy + dz*dz); // modulo de la distancia
                azimuth = atan2(dy, dx); // angulo del punto
                elevation = asin (dz/r); // Restricciones del sensor
              
                // Si el punto entra dentro de las restricciones
                if((fabs(yaw-azimuth)< res_azim &&  fabs(pitch-elevation)< res_elev) || (abs(vz) > 0.4 && dz > 0)){
                    if ((r - radio_dron) < min_dist){ 
                        min_dist = r - radio_dron;// Tener en cuenta las dimensiones del robot
                    }
                }
            }
            return min_dist;
        }
        
        // Alineación entre la dirección de velocidad evaluada y la dirección del objetivo en el plano XY.
        double calcYawHeading (geometry_msgs::Pose pose, geometry_msgs::Point goal){
            double diff_x = goal.x - pose.position.x, diff_y = goal.y - pose.position.y, yaw = -2;
            if(round(diff_x) == 0){
                if(round(diff_y) == 0) yaw = 0; // Estoy en el goal --> puntuacion maxima?
                else if (round(diff_y) < 0) yaw = -PI/2; // DERECHA
                else yaw = PI/2; // IZQUIERDA goal.y > robot.y
            } else if (round(diff_y) == 0){// goal.x != robot.x
                if (round(diff_x) > 0) yaw = 0; // DELANTE
                else yaw = PI; // DETRAS goal.x < robot.x
            } else  yaw = atan2(diff_y, diff_x); // goal.x != robot.x && goal.y != robot.y --> rango (-PI,PI]
            yaw -= pose.orientation.z; // yaw: orientacion del goal respecto a la pos(x,y) del robot --> Restar la orientacion del robot
            return ((PI - fabs(norm_rad(yaw)))/PI);// * ((PI - fabs(norm_rad(yaw)))/PI); // rango [0,1]
        }
        // Alineacion en altura con el objetivo
        double calcZHeading (geometry_msgs::Pose pose, geometry_msgs::Point goal){return fabs(goal.z - pose.position.z);}
        // Modulo del vector distancia entre pose y goal
        double goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal){
            double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
            double euc = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
            return euc;
        }

        //Posición predicha tras ejecutar una trayectoria con velocidad [vx,vy,vz] durante un tiempo [dt]
        geometry_msgs::Pose simubot (double vx, double w, double vz, double dt){
            geometry_msgs::Pose pose_final;
            tf::Quaternion q1(last_pose.orientation.x, last_pose.orientation.y, last_pose.orientation.z, last_pose.orientation.w);
            tf::Matrix3x3 m1(q1);
            double roll, pitch,yaw;
            m1.getRPY(roll, pitch, yaw);
            pose_final.orientation.z = norm_rad(yaw + w * dt);
            pose_final.position.x =  last_pose.position.x + vx*dt*cos(pose_final.orientation.z);
            pose_final.position.y =  last_pose.position.y + vx*dt*sin(pose_final.orientation.z);
            pose_final.position.z =  last_pose.position.z + vz*dt;
            double v = sqrt(vx*vx + vz*vz);
            if (vz==0 && v==0) pose_final.orientation.x = 0;
            else pose_final.orientation.x = pitch;
            pose_final.orientation.y = roll;
            return pose_final;
        }
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_executor");

  Dwa3d controller("/action/trajectory");
  controller.idle();
  return 0;
}