//test_move_cartesian.cpp
//wsn, 7/10/18
//given cartesian differential motion commands, construct a Cartesian
// trajectory from current pose to specified x, y or z increment (if possible)

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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
//#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Vector3.h>

#include <xform_utils/xform_utils.h>
//the following includes velocity limits--should grab these from URDF/parameter server
#include <mh5020_fk_ik/mh5020_kinematics.h>

using namespace std;
#define VECTOR_DIM 6 // e.g., a 6-dof vector
const double dt_traj = 0.05; // time step for trajectory interpolation
int g_done_count = 0;
int g_done_move = true;
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_jnt_names; //should get these from URDF as well...tough

int ans;
Eigen::VectorXd g_q_vec;
XformUtils xform_utils;

ros::Publisher pub; //global joint-command publisher

void map_arm_joint_indices(vector<string> joint_names) {
    //vector<string> joint_names = joint_state->name;
    //   vector<string> jnt_names;

    g_arm_joint_indices.clear();
    int index;
    int n_jnts = VECTOR_DIM;
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

    for (int j = 0; j < VECTOR_DIM; j++) {
        j_name = g_jnt_names[j]; //known name, in preferred order
        for (int i = 0; i < n_jnts; i++) {
            if (j_name.compare(joint_names[i]) == 0) {
                index = i;
                //cout<<"found match at index = "<<i<<endl;
                g_arm_joint_indices.push_back(index);
                break;
            }
        }
    }
    ROS_INFO_STREAM("indices of arm joints: " << endl);
    for (int i = 0; i < VECTOR_DIM; i++) {
        ROS_INFO_STREAM(g_arm_joint_indices[i] << ", ");
    }
    ROS_INFO_STREAM(endl);
}

void set_jnt_names() {
    g_jnt_names.push_back("joint_1_s");
    g_jnt_names.push_back("joint_2_l");
    g_jnt_names.push_back("joint_3_u");
    g_jnt_names.push_back("joint_4_r");
    g_jnt_names.push_back("joint_5_b");
    g_jnt_names.push_back("joint_6_t");
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
    ROS_INFO_STREAM("CB: q_vec_arm: " << g_q_vec_arm_Xd.transpose() << endl);
}

//need to reference realistic joint velocity limits to compute min transition times
// the following assumes speeds limited to binding constraint of joint velocity limits

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
// need to upgrade this to accept an arrival time
//implicitly assumes first point is same as current joint values, and assigns arrival_time=0 and vel=0 for this point
//set arrival times at equal increments of DT

void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory, double DT) {
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_point1.positions.clear();
    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();

    for (int i = 0; i < VECTOR_DIM; i++) {
        new_trajectory.joint_names.push_back(g_jnt_names[i].c_str());
    }

    new_trajectory.header.stamp = ros::Time::now();
    double sim_time = new_trajectory.header.stamp.toSec();
    ROS_INFO("sim time = %f", sim_time);
    Eigen::VectorXd q_start, q_end, dqvec, qdot_vec;
    double del_time;
    double net_time = 0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];
    qdot_vec.resize(VECTOR_DIM);
    qdot_vec << 0, 0, 0, 0, 0, 0;
    ROS_INFO_STREAM("stuff_traj: start pt = " << q_start.transpose() << endl);
    ROS_INFO("stuffing trajectory");

    trajectory_point1.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < VECTOR_DIM; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
        trajectory_point1.velocities.push_back(0.0); //ditto for velocity vec
    }
    // first point of the trajectory: must be identical to current joint poses; not enforced here
    new_trajectory.points.push_back(trajectory_point1);
    //add the rest of the points from qvecs

    // following loop presumes there are at least two joint-space poses in the qvecs vector of poses
    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end - q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time < DT)
            del_time = DT;

        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time += del_time;
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i = 0; i < VECTOR_DIM; i++) { //copy over the joint-command values
            trajectory_point1.positions[i] = q_end[i];
            trajectory_point1.velocities[i] = dqvec[i] / del_time;
        }

        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }
    //display trajectory:
    for (int iq = 1; iq < qvecs.size(); iq++) {
        ROS_INFO_STREAM("traj pt: ");
        for (int j = 0; j < VECTOR_DIM; j++) {
            ROS_INFO_STREAM(new_trajectory.points[iq].positions[j] << ", ");
        }
        ROS_INFO_STREAM(endl);
        ROS_INFO_STREAM("arrival time: " << new_trajectory.points[iq].time_from_start.toSec() << endl);
    }
}

//given a vector of Eigen vectors, and given a reference, q_ref, find the closest element of q6dof_solns to q_ref,
// and fill in q_closest with this soln.  Return the norm of dist between q_ref and q_closest

double closest_soln(Eigen::VectorXd q_ref, std::vector<Eigen::VectorXd> q6dof_solns, Eigen::VectorXd &q_closest) {
    q_closest.resize(6); // HARD CODED 6DOF
    int nsolns = q6dof_solns.size();
    double q_err_best = 1000000.0;
    double q_err;
    int i_min = 0;

    if (nsolns <= 0) return (q_err_best); //give up if no elements in q6dof_solns;
    q_closest = q6dof_solns[0];
    Eigen::VectorXd q_soln = q_closest;

    for (int i = 0; i < nsolns; i++) {
        q_soln = q6dof_solns[i];
        //ik_solver.fit_joints_to_range(q_soln);
        ROS_INFO_STREAM("q_soln_mh5020: " << q_soln.transpose() << std::endl);
        q_err = (q_ref - q_soln).norm();
        if (q_err < q_err_best) {
            q_closest = q_soln;
            q_err_best = q_err;
        }
    }
    ROS_INFO("closest soln: ");
    ROS_INFO_STREAM(q_closest.transpose() << endl);
    return q_err_best;
}

bool move_to_jspace_point(Eigen::VectorXd q_goal, double move_time) {
    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory  
    des_path.clear();
    ros::spinOnce(); //update reading of current joint angles
    des_path.push_back(g_q_vec_arm_Xd); //start from current pose  
    des_path.push_back(q_goal); //command to go to specified goal pose          

    stuff_trajectory(des_path, des_trajectory, move_time); //convert path to traj

    ROS_WARN("enter 1 to move arm to specified joint-space pose: CAREFUL!");
    int ans = 0;
    cin >> ans;
    if (ans == 1) {
        des_trajectory.header.stamp = ros::Time::now(); //update time stamp to avoid rejection
        pub.publish(des_trajectory);
        ros::spinOnce();
        ros::Duration(move_time).sleep();
        return true;
    } else {
        ROS_WARN("incorrect reply--move aborted");
        return false;
    }
}


// MAIN PROGRAM:

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "test_move_cartesian"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    cout << "enter 1 for simu, 2 for ROS-I: ";
    cin >> ans;
    if (ans == 1) {
        ROS_INFO("will publish to /MH5020/arm_controller/command");
        ros::Publisher pub_simu = nh.advertise<trajectory_msgs::JointTrajectory>("/MH5020/arm_controller/command", 1);
        pub = pub_simu;
    } else if (ans == 2) {
        //this is the correct topic and message type for Motoman streaming
        ros::Publisher pub_rosi = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);
        ROS_INFO("will publish to /joint_path_command");
        pub = pub_rosi;
    }
    Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec_arm;
    g_q_vec_arm_Xd.resize(VECTOR_DIM);
    Eigen::VectorXd q_vec_wsn_model; // hack to negate J2 direction
    q_vec_wsn_model.resize(VECTOR_DIM);

    Eigen::VectorXd q_soln, q_mh5020;
    q_soln.resize(NJNTS);
    q_mh5020.resize(NJNTS);
    Eigen::Vector3d O_current, O_desired;
    Eigen::Vector3d z_tool_des;
    z_tool_des << 0, 0, -1; //paint head pointing down, perpendicular to ground
    Eigen::Vector3d x_tool_des;
    x_tool_des << 1, 0, 0; // arbitrary--tool x-axis pointing away from robot, parallel to ground
    Eigen::Vector3d y_tool_des;
    y_tool_des = z_tool_des.cross(x_tool_des);
    Eigen::Matrix3d R_des; // desired tool orientation in world frame
    R_des.col(0) = x_tool_des;
    R_des.col(1) = y_tool_des;
    R_des.col(2) = z_tool_des;

    Eigen::Affine3d a_tool;
    a_tool.linear() << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    a_tool.translation() << 0.0,
            0.0,
            0.0;

    MH5020_fwd_solver mh5020_fwd_solver;
    MH5020_IK_solver ik_solver;
    std::vector<Eigen::VectorXd> q6dof_solns;
    Eigen::VectorXd q_offsets_vecxd;
    q_offsets_vecxd.resize(NJNTS);
    for (int i = 0; i < NJNTS; i++)
        q_offsets_vecxd[i] = DH_q_offsets[i];

    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   
    set_jnt_names(); //fill a vector of joint names in DH order, from base to tip
    //here are initial, hard-coded joint angles for arm pose
    //cout << "setting pre-pose: " << endl;
    //q_pre_pose.resize(VECTOR_DIM);
    //q_pre_pose << 0, 0, 0, 0, 0, 0; //-0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);

    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    while (g_arm_joint_indices.size() < 1) {
        ros::spinOnce();
        ros::Duration(1).sleep();
        ROS_INFO("waiting for joint states...");
    }

    //ROS_INFO("getting transform, world frame to robot base frame: ...not really yet");
    tf::TransformListener tfListener;
    bool tferr = true;
    ROS_INFO("waiting for tf between rear_wheel_frame and robot's base_link...");
    tf::StampedTransform tfBaseLinkWrtTruck;
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
            tfListener.lookupTransform("rear_wheel_frame", "base_link", ros::Time(0), tfBaseLinkWrtTruck);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");
    Eigen::Affine3d affineBaseLinkWrtTruck = xform_utils.transformStampedTfToEigenAffine3d(tfBaseLinkWrtTruck);
    xform_utils.printAffine(affineBaseLinkWrtTruck);


    //Eigen::Affine3d transformStampedTfToEigenAffine3d(tf::StampedTransform sTf);   



    //get current pose of arm:  
    ROS_INFO_STREAM("current joint values:" << g_q_vec_arm_Xd.transpose() << endl);
    ROS_WARN("ready to warm up trajectory publications...");

    des_path.clear();
    des_path.push_back(g_q_vec_arm_Xd); //start from current pose  
    des_path.push_back(g_q_vec_arm_Xd); //command to go to current pose            

    stuff_trajectory(des_path, des_trajectory, dt_traj); //convert path to traj

    ROS_WARN("enter 1 to send warm-up trajectories...be careful; arm should not move");
    ans = 0;
    cin >> ans;
    if (ans == 1) {
        for (int i = 0; i < 3; i++) {
            des_trajectory.header.stamp = ros::Time::now(); //update time stamp to avoid rejection

            pub.publish(des_trajectory);
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
    } else {
        ROS_WARN("not a valid response; quitting");
        return 1;
    }

    Eigen::Affine3d A_fwd_desired, A_fwd_test_goal;
    Eigen::Affine3d A_flange_wrt_world, A_flange_des_wrt_world;
    Eigen::Affine3d A_flange_wrt_truck, A_flange_des_wrt_truck;
    Eigen::Affine3d A_flange_wrt_base_link, A_flange_des_wrt_base_link;

    A_flange_wrt_base_link = mh5020_fwd_solver.fwd_kin_solve(g_q_vec_arm_Xd); //fwd_kin_solve
    A_flange_wrt_truck = affineBaseLinkWrtTruck*A_flange_wrt_base_link;
    ROS_INFO("computed flange pose w/rt world: ");
    xform_utils.printAffine(A_flange_wrt_truck);
    A_flange_des_wrt_truck = A_flange_wrt_truck;
    A_flange_des_wrt_truck.linear() = R_des; // set desired toolflange pose w/rt world
    ROS_INFO("desired flange pose w/rt world: ");
    xform_utils.printAffine(A_flange_des_wrt_truck);

    //convert desired tool_wrt_truck into tool_wrt_base_link for IK soln:
    A_flange_des_wrt_base_link = affineBaseLinkWrtTruck.inverse() * A_flange_des_wrt_truck;
    ROS_INFO("corresponding des flange pose w/rt robot base_link:");
    xform_utils.printAffine(A_flange_des_wrt_base_link);

    int nsolns = ik_solver.ik_solve(A_flange_des_wrt_base_link, q6dof_solns);
    ROS_INFO_STREAM("found " << nsolns << " solution(s):" << std::endl);
    //q_soln, 
    double best_fit_err = closest_soln(g_q_vec_arm_Xd, q6dof_solns, q_soln);
    ROS_WARN("moving wrist to desired orientation w/rt world: ");
    bool move_success = move_to_jspace_point(q_soln, 3.0);

    double qval;

    while (ros::ok()) {
        cout << "enter 1 to get joint angles: ";
        cin >> ans;
        ros::spinOnce();
        //for (int i = 0; i < NJNTS; i++) {
        //    q_in[i] = g_q_vec[i]; // assign q to actual joint states
        //}
        ROS_INFO("forward kin: ");
        q_vec_wsn_model = g_q_vec_arm_Xd;
        //q_vec_wsn_model[1] = -q_vec_wsn_model[1];
        A_flange_wrt_base_link = mh5020_fwd_solver.fwd_kin_solve(q_vec_wsn_model); //fwd_kin_solve
        A_flange_wrt_truck = affineBaseLinkWrtTruck*A_flange_wrt_base_link;
        ROS_INFO("computed flange pose w/rt rear_wheel_frame: ");
        xform_utils.printAffine(A_flange_wrt_truck);

        O_current = A_flange_wrt_truck.translation();
        std::cout << "tool frame origin from FK: " << O_current.transpose() << std::endl;

        int direction_code;
        double move_size;
        cout << "enter code for desired jog direction " << endl;
        cout << "dx: 0,  dy: 1, dz: 2  :";

        cin >> direction_code;
        if (direction_code < 3 && direction_code> -1) {
            ROS_INFO("entered direction code %d", direction_code);
            cout << "enter desired displacement, in m: ";
            cin >> move_size;
            ROS_INFO("entered desired displacement magnitude = %f", move_size);
            O_desired = O_current;
            O_desired[direction_code] += move_size;
            A_flange_des_wrt_truck.translation() = O_desired;
            A_flange_des_wrt_truck.linear() = R_des; // set desired toolflange pose w/rt world

            ROS_INFO("desired flange pose w/rt world: ");
            xform_utils.printAffine(A_flange_des_wrt_truck);

            //convert desired tool_wrt_truck into tool_wrt_base_link for IK soln:
            A_flange_des_wrt_base_link = affineBaseLinkWrtTruck.inverse() * A_flange_des_wrt_truck;
            ROS_INFO("corresponding des flange pose w/rt robot base_link:");
            xform_utils.printAffine(A_flange_des_wrt_base_link);

            int nsolns = ik_solver.ik_solve(A_flange_des_wrt_base_link, q6dof_solns);
            ROS_INFO_STREAM("found " << nsolns << " solution(s):" << std::endl);
            if (nsolns > 0) {
                double best_fit_err = closest_soln(g_q_vec_arm_Xd, q6dof_solns, q_soln);
                ROS_WARN("preparing to move to new goal pose: ");
                bool move_success = move_to_jspace_point(q_soln, 3.0);
            } else {
                ROS_WARN("move is not reachable");
            }

        } else {
            ROS_WARN("not a valid direction code");
        }
    }

    return 0;
}



