//test_jog_joints.cpp
//wsn, 7/3/18

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>


//the following includes velocity limits--should grab these from URDF/parameter server
#include <mh5020_fk_ik/mh5020_kinematics.h>

using namespace std;
#define VECTOR_DIM 6 // e.g., a 6-dof vector
const double dt_traj = 0.02; // time step for trajectory interpolation
int g_done_count = 0;
int g_done_move = true;
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;
//TEST: deliberately limit joint velocities to very small values
//double g_qdot_max_vec[] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; //put real vel limits here
int ans;

void map_arm_joint_indices(vector<string> joint_names) {
    //vector<string> joint_names = joint_state->name;
    //   vector<string> jnt_names;

    g_arm_joint_indices.clear();
    int index;
    int n_jnts = VECTOR_DIM;
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

    for (int j = 0; j < VECTOR_DIM; j++) {
        j_name = g_ur_jnt_names[j]; //known name, in preferred order
        for (int i = 0; i < n_jnts; i++) {
            if (j_name.compare(joint_names[i]) == 0) {
                index = i;
                //cout<<"found match at index = "<<i<<endl;
                g_arm_joint_indices.push_back(index);
                break;
            }
        }
    }
    cout << "indices of arm joints: " << endl;
    for (int i = 0; i < VECTOR_DIM; i++) {
        cout << g_arm_joint_indices[i] << ", ";
    }
    cout << endl;
}

void set_jnt_names() {
    g_ur_jnt_names.push_back("joint_1_s");
    g_ur_jnt_names.push_back("joint_2_l");
    g_ur_jnt_names.push_back("joint_3_u");
    g_ur_jnt_names.push_back("joint_4_r");
    g_ur_jnt_names.push_back("joint_5_b");
    g_ur_jnt_names.push_back("joint_6_t");
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    //joint_states_ = js_msg; // does joint-name mapping only once
    if (g_arm_joint_indices.size() < 1) {
        int njnts = js_msg.position.size();
        ROS_INFO("finding joint mappings for %d jnts", njnts);
        map_arm_joint_indices(js_msg.name);
    }
        for (int i = 0; i < VECTOR_DIM; i++) {
            g_q_vec_arm_Xd[i] = js_msg.position[g_arm_joint_indices[i]];
        }
        cout << "CB: q_vec_arm: " << g_q_vec_arm_Xd.transpose() << endl;
}

//need to reference realistic joint velocity limits to compute min transition times
double transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0]) / g_qdot_max_vec[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i = 1; i < VECTOR_DIM; i++) {
        ti = fabs(dqvec[i]) / g_qdot_max_vec[i];
        if (ti > t_max) t_max = ti;
    }
    return t_max;
}

//given a path, qvecs, comprised of a sequence of 6DOF poses, construct
// a corresponding trajectory message w/ plausible arrival times
// re-use joint naming, as set by set_jnt_names
//arrival time follows from running joints at max feasible velocities...
// but init vel limits to low values;
// need to upgrade this to accept an arrival time
void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 

    trajectory_point1.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < VECTOR_DIM; i++) {
        new_trajectory.joint_names.push_back(g_ur_jnt_names[i].c_str());
    }

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start, q_end, dqvec, qdot_vec;
    double del_time;
    double net_time = 0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];
    qdot_vec.resize(VECTOR_DIM);
    qdot_vec<<0,0,0,0,0,0;
    cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");
    //trajectory_point1.positions = qvecs[0];

    trajectory_point1.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < VECTOR_DIM; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
        trajectory_point1.velocities.push_back(0.0);
    }
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs


    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end - q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time < dt_traj)
            del_time = dt_traj;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time += del_time;
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i = 0; i < VECTOR_DIM; i++) { //copy over the joint-command values
            trajectory_point1.positions[i] = q_end[i];
            trajectory_point1.velocities[i] = dqvec[i]/del_time;
        }
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }
  //display trajectory:
    for (int iq = 1; iq < qvecs.size(); iq++) {
        cout<<"traj pt: ";
                for (int j=0;j<VECTOR_DIM;j++) {
                    cout<<new_trajectory.points[iq].positions[j]<<", ";
                }
        cout<<endl;
        cout<<"arrival time: "<<new_trajectory.points[iq].time_from_start.toSec()<<endl;
    }
}


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "test_jog_joints"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    //this is the correct topic and message type for Motoman streaming
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1); 
     Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec_arm;
    g_q_vec_arm_Xd.resize(VECTOR_DIM);

    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   
    set_jnt_names(); //fill a vector of joint names in DH order, from base to tip
    //here are initial, hard-coded joint angles for arm pose
    cout << "setting pre-pose: " << endl;
    q_pre_pose.resize(VECTOR_DIM);
    q_pre_pose << 0, 0, 0, 0, 0, 0; //-0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);

    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    while (g_arm_joint_indices.size() < 1) {
        ros::spinOnce();
        ros::Duration(1).sleep();
        ROS_INFO("waiting for joint states...");
    }

    //get current pose of arm:  
    cout << "current pose:" << g_q_vec_arm_Xd.transpose() << endl;
    Eigen::Vector3d O_current, O_desired;
    Eigen::Affine3d A_fwd_kin;
    
    Eigen::Affine3d a_tool;
    a_tool.linear() << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    a_tool.translation() << 0.0,
            0.0,
            0.0;

    MH5020_fwd_solver mh5020_fwd_solver;
    Eigen::VectorXd q_offsets_vecxd;
    q_offsets_vecxd.resize(NJNTS);
    for (int i = 0; i < NJNTS; i++)
        q_offsets_vecxd[i] = DH_q_offsets[i];
    
    //q_pre_pose is initialized to all zeros
    int jnt=5;
    double qval;
    while((jnt>=0)&&(jnt<6)) {
        cout<<"enter jnt num, 0 through 5: ";
        cin>>jnt;
        cout<<"enter jnt angle: ";
        cin>>qval;
        q_pre_pose[jnt] = qval; 
        des_path.clear();
        ros::spinOnce(); //update joint states
        des_path.push_back(g_q_vec_arm_Xd); //start from current pose
        des_path.push_back(q_pre_pose); //and go to new desired pose
        stuff_trajectory(des_path, des_trajectory); //convert path to traj

        cout<<"enter 1 to send the trajectory command to move the arm: CAREFUL!!: ";
        cin>>ans;
        pub.publish(des_trajectory);
        cout<<"enter 1 to capture joint angles: ";
        cin>>ans;        
        ros::spinOnce();
        cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;     
        //compute fwd kin:
        A_fwd_kin = mh5020_fwd_solver.fwd_kin_solve(g_q_vec_arm_Xd);
        ROS_INFO_STREAM("fwd kin toolflange origin: "<<A_fwd_kin.translation().transpose()<<endl);
        ROS_INFO_STREAM("fwd kin orientation: "<<endl);
        ROS_INFO_STREAM(A_fwd_kin.linear()<<endl);
    }

    return 0;
}
    
    

