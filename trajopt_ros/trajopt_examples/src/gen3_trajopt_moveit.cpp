#include <tesseract_core/basic_types.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>

#include <urdf_parser/urdf_parser.h>

#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/logging.hpp>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <cmath>

control_msgs::FollowJointTrajectoryGoal trajectory_action;
//trajectory_msgs::JointTrajectory traj_msg;

void trajArrayToJointTrajectory_moveit(std::vector<std::string> joint_names,
                                       tesseract::TrajArray traj_array,
                                       moveit::core::RobotModelPtr robot_model,
                                       bool use_time,
                                       bool interpolate,
                                       ros::Duration time_increment)

{
  // convert to trajectory msg
  // Create the joint trajectory
  trajectory_msgs::JointTrajectory traj_msg;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = "/base_link";
  traj_msg.joint_names = joint_names;

  tesseract::TrajArray pos_mat;
  tesseract::TrajArray time_mat;
  if (use_time)
  {
    // Seperate out the time data in the last column from the joint position data
    pos_mat = traj_array.leftCols(traj_array.cols()-1);
    time_mat = traj_array.rightCols(1);
  }
  else
  {
    pos_mat = traj_array;
  }

  std::cout << "how many waypoints using trajopt: " << traj_array.rows() << std::endl;
  ros::Duration time_from_start(0);
  for (int ind = 0; ind < traj_array.rows(); ind++)
  {
    // Create trajectory point
    trajectory_msgs::JointTrajectoryPoint traj_point;
    auto mat = pos_mat.row(ind);
    // Set the position for this time step
    if (interpolate)
    {
      if (ind == traj_array.rows()-1) continue;
      auto mat_next = pos_mat.row(ind+1);
      auto dist = (time_mat(ind+1, time_mat.cols()-1) / 0.001); // decrease the last number to get valid trajectory
      Eigen::MatrixXd dmat = (mat_next - mat) / dist;

      std::cout << dmat(0,0) << " " << dmat(0,1) << std::endl;
      // if (dmat(0,0) < pow(10, -8) && dmat(0,1) < pow(10, -8) && dmat(0,2) < pow(10, -8) && dmat(0,3) < pow(10, -8) && dmat(0,4) < pow(10, -8) && dmat(0,5) < pow(10, -8) && dmat(0,6) < pow(10, -8))
      // {
      //   goto label;
      // }
      for (int i = 0; i < dist;i++)
      {
        Eigen::MatrixXd temp_mat = mat + i*dmat;
        std::vector<double> vec(temp_mat.data(), temp_mat.data() + temp_mat.rows() * temp_mat.cols());
        traj_point.positions = vec;
        if (use_time)
        {
          time_from_start += ros::Duration(0.001);
        }
        else
        {
          //BUG to fix
          time_from_start += time_increment;
        }
        traj_point.time_from_start = time_from_start;

        traj_msg.points.push_back(traj_point);
      }  
    }
    else {
      std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
      traj_point.positions = vec;
      //traj_point.velocities = std::vector<double>(7, 0.0);
      //traj_point.accelerations = std::vector<double>(7, 0.0);

      // Add the current dt to the time_from_start
      if (use_time)
      {
        time_from_start += ros::Duration(time_mat(ind, time_mat.cols() - 1));
      }
      else
      {
        time_from_start += time_increment;
      }
      traj_point.time_from_start = time_from_start;

      traj_msg.points.push_back(traj_point);
    }

//    if (pos_mat.row(ind+1)[0] == pos_mat.row(ind)[0])
//      break;
  }

//label:
  std::cout << traj_msg.points.size() << std::endl;

  // convert to moveit trajectory
  int num_dof = traj_msg.points[1].positions.size();
  int num_waypoints = traj_msg.points.size();

  robot_trajectory::RobotTrajectory moveit_trajectory(robot_model, "Manipulator");

  //Convert trajopt array trajectory to moveit trajectory
  const robot_model::JointModelGroup* group = moveit_trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED("trajectory_processing", "Need to set the group");
    //return;
  }

  moveit::core::RobotState state(moveit_trajectory.getRobotModel());

  //TODO: Set velocity + acceleration limits
  //Loop through all trajopt waypoints
  for (int i = 0; i < num_waypoints; i++)
  {
    std::vector<double> joint_state;

    for (int j = 0; j < num_dof; j++) {
      joint_state.push_back(traj_msg.points[i].positions[j]);
    }
    state.setVariablePositions(joint_names, joint_state);
    moveit_trajectory.addSuffixWayPoint(state, 0.001);   // it was 0.0, changed to 0.001
  }

  //Smooth trajectory
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  time_parameterization.computeTimeStamps(moveit_trajectory, 0.8, 0.8);  //it was 1.0 and 1.0, changed to 0.8, 0.8

  //Convert smoothed trajectory to ROS trajectory
  moveit_msgs::RobotTrajectory smooth_traj_msg;
  moveit_trajectory.getRobotTrajectoryMsg(smooth_traj_msg);

  // modify the msg
  smooth_traj_msg.joint_trajectory.header.stamp = ros::Time::now();
  smooth_traj_msg.joint_trajectory.header.frame_id = "/base_link";
  smooth_traj_msg.joint_trajectory.joint_names = joint_names;
  for (int i = 0; i < num_waypoints; i++)
  {
    smooth_traj_msg.joint_trajectory.points[i].time_from_start = ros::Duration(0.001*i);
  }


  // hack
  for (int i = 0; i < num_waypoints; i++)
  {
    for (int j = 0; j < 7; j++){
      if (smooth_traj_msg.joint_trajectory.points[i].accelerations[j] < -0.99)
      {
        smooth_traj_msg.joint_trajectory.points[i].accelerations[j] = -0.99;
      }
      if (smooth_traj_msg.joint_trajectory.points[i].accelerations[j] > 0.99)
      {
        smooth_traj_msg.joint_trajectory.points[i].accelerations[j] = 0.99;
      }
    }
  }


  //TODO: Plot smoothed trajectory
  std::cout << "TRAJOPT: Moveit trajectory smoothing is done!\n";


  trajectory_action.trajectory = smooth_traj_msg.joint_trajectory;

//  std::cout << smooth_traj_msg.joint_trajectory.points[9027].accelerations[3] << std::endl;


}

trajectory_msgs::JointTrajectory trajArrayToJointTrajectoryMsg(std::vector<std::string> joint_names,
                                                               tesseract::TrajArray traj_array,
                                                               bool use_time,
                                                               bool interpolate,
                                                               ros::Duration time_increment)
{
  // Create the joint trajectory
  trajectory_msgs::JointTrajectory traj_msg;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = "/base_link";
  traj_msg.joint_names = joint_names;

  tesseract::TrajArray pos_mat;
  tesseract::TrajArray time_mat;
  if (use_time)
  {
    // Seperate out the time data in the last column from the joint position data
    pos_mat = traj_array.leftCols(traj_array.cols()-1);
    time_mat = traj_array.rightCols(1);
  }
  else
  {
    pos_mat = traj_array;
  }

  ros::Duration time_from_start(0);
  for (int ind = 0; ind < traj_array.rows(); ind++)
  {
    // Create trajectory point
    trajectory_msgs::JointTrajectoryPoint traj_point;
    auto mat = pos_mat.row(ind);
    // Set the position for this time step
    if (interpolate)
    {
      if (ind == traj_array.rows()-1) continue;
      auto mat_next = pos_mat.row(ind+1);
      auto dist = (time_mat(ind+1, time_mat.cols()-1) / 0.001);
      Eigen::MatrixXd dmat = (mat_next - mat) / dist;
      for (int i = 0; i < dist;i++)
      {
        Eigen::MatrixXd temp_mat = mat + i*dmat;
        std::vector<double> vec(temp_mat.data(), temp_mat.data() + temp_mat.rows() * temp_mat.cols());
        traj_point.positions = vec;
        if (use_time)
        {
          time_from_start += ros::Duration(0.001);
        }
        else
        {
          //BUG to fix
          time_from_start += time_increment;
        }
        traj_point.time_from_start = time_from_start;

        traj_msg.points.push_back(traj_point);

      }

    }
    else {
      std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
      traj_point.positions = vec;
      //traj_point.velocities = std::vector<double>(7, 0.0);
      //traj_point.accelerations = std::vector<double>(7, 0.0);

      // Add the current dt to the time_from_start
      if (use_time)
      {
        time_from_start += ros::Duration(time_mat(ind, time_mat.cols() - 1));
      }
      else
      {
        time_from_start += time_increment;
      }
      traj_point.time_from_start = time_from_start;

      traj_msg.points.push_back(traj_point);
    }
  }
  return traj_msg;
}

void addCollision(tesseract::tesseract_ros::KDLEnvPtr env, std::string name, Eigen::Vector3d size, Eigen::Vector3d pose)
{
  tesseract::AttachableObjectPtr obj(new tesseract::AttachableObject());
  std::shared_ptr<shapes::Box> box(new shapes::Box());
  Eigen::Isometry3d box_pose = Eigen::Isometry3d::Identity();
  box_pose.translation() += pose;

  box->size[0] = size[0];
  box->size[1] = size[1];
  box->size[2] = size[2];

  obj->name = name;
  obj->visual.shapes.push_back(box);
  obj->visual.shape_poses.push_back(box_pose);
  obj->collision.shapes.push_back(box);
  obj->collision.shape_poses.push_back(box_pose);
  obj->collision.collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  env->addAttachableObject(obj);

  // move box to the correct location
  tesseract::AttachedBodyInfo attached_body;
  Eigen::Isometry3d body_pose = Eigen::Isometry3d::Identity();
  attached_body.object_name = name;
  attached_body.parent_link_name = "link_0";
  attached_body.transform = body_pose;

  env->attachBody(attached_body);
}

int main(int argc, char** argv)
{
  //////////////////////
  /// INITIALIZATION ///
  //////////////////////

  ros::init(argc, argv, "pick_and_place_plan");
  ros::NodeHandle nh, pnh("~");

  int steps_per_phase;
  bool plotting_cb, file_write_cb;
  pnh.param<int>("steps_per_phase", steps_per_phase, 10);
  pnh.param<bool>("plotting", plotting_cb, false);
  pnh.param<bool>("file_write_cb", file_write_cb, false);

  // Set Log Level
  util::gLogLevel = util::LevelInfo;

  /////////////
  /// SETUP ///
  /////////////

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string;

  nh.getParam("robot_description", urdf_xml_string);
  nh.getParam("robot_description_semantic", srdf_xml_string);


  // Initialize the environment
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml_string);
  srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model->initString(*urdf_model, srdf_xml_string);
  tesseract::tesseract_ros::KDLEnvPtr env(new tesseract::tesseract_ros::KDLEnv);
  assert(urdf_model != nullptr);
  assert(env != nullptr);
  bool success = env->init(urdf_model, srdf_model);
  assert(success);

  //Create robot model for using moveit time parameterization
  moveit::core::RobotModelPtr robot_model;
  robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));

  // Set the initial state of the robot
  std::unordered_map<std::string, double> joint_states;
  bool sim_robot = false;
  if (sim_robot) {
    joint_states["joint_1"] = 0.0;      // 0
    joint_states["joint_2"] = -0.35;    // 0.26
    joint_states["joint_3"] = 3.14;     // 3.14
    joint_states["joint_4"] = -2.54;    // -2.27
    joint_states["joint_5"] = 0.0;      // 0
    joint_states["joint_6"] = -0.87;    // 0.96
    joint_states["joint_7"] = 1.57;     // 1.57
  }
  else
  {
    boost::shared_ptr<const sensor_msgs::JointState> joint_pos =
        ros::topic::waitForMessage<sensor_msgs::JointState>("/my_gen3/joint_states", nh);

    joint_states["joint_1"] = joint_pos.get()->position[0];
    joint_states["joint_2"] = joint_pos.get()->position[1];
    joint_states["joint_3"] = joint_pos.get()->position[2];
    joint_states["joint_4"] = joint_pos.get()->position[3];
    joint_states["joint_5"] = joint_pos.get()->position[4];
    joint_states["joint_6"] = joint_pos.get()->position[5];
    joint_states["joint_7"] = joint_pos.get()->position[6];
  }
  env->setState(joint_states);

    // <group_state name="home" group="arm">
    //     <joint name="joint_1" value="0" />
    //     <joint name="joint_2" value="0.26" />
    //     <joint name="joint_3" value="3.14" />
    //     <joint name="joint_4" value="-2.27" />
    //     <joint name="joint_5" value="0" />
    //     <joint name="joint_6" value="0.96" />
    //     <joint name="joint_7" value="1.57" />
    // </group_state>
    // <group_state name="retract" group="arm">
    //     <joint name="joint_1" value="0" />
    //     <joint name="joint_2" value="-0.35" />
    //     <joint name="joint_3" value="3.14" />
    //     <joint name="joint_4" value="-2.54" />
    //     <joint name="joint_5" value="0" />
    //     <joint name="joint_6" value="-0.87" />
    //     <joint name="joint_7" value="1.57" />
    // </group_state>


  for (auto i : env->getJointNames()) {
    ROS_INFO("Joint Names:%s", i.c_str());
  }
 

  addCollision(env, "tube1", Eigen::Vector3d(0.03,0.03,0.04), Eigen::Vector3d(0.42,-0.04,0.085));
  addCollision(env, "tube2", Eigen::Vector3d(0.04,0.03,0.03), Eigen::Vector3d(0.36,-0.04,0.03));
  addCollision(env, "tube3", Eigen::Vector3d(0.03,0.04,0.03), Eigen::Vector3d(0.315,0.02,0.03));


  // Send the initial trajectory for plotting
  tesseract::tesseract_ros::ROSBasicPlotting plotter(env);
  Eigen::RowVectorXd init_pos = env->getCurrentJointValues();
  plotter.plotTrajectory(env->getJointNames(), init_pos.leftCols(env->getJointNames().size()));

  ////////////
  /// PICK ///
  ////////////

  ROS_ERROR("Press enter to continue");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  // Create the planner and the responses that will store the results
  tesseract::tesseract_planning::TrajOptPlanner planner;
  tesseract::tesseract_planning::PlannerResponse planning_response;
  tesseract::tesseract_planning::PlannerResponse planning_response_place;

  // Choose the manipulator and end effector link
  std::string manip = "Manipulator";
  std::string end_effector = "end_effector_link";

  // Define the final pose (on top of the box)
  Eigen::Isometry3d final_pose;
  Eigen::Quaterniond orientation(0.029, 0.706, 0.707, 0.029); // w x y z
  final_pose.linear() = orientation.matrix();
  final_pose.translation() += Eigen::Vector3d(0.425, 0.0, 0.331);  // Offset for the table

  /* retract
  orientation(0.029, 0.706, 0.707, 0.029)
  Eigen::Vector3d(0.125, 0.002, 0.331)
  */
  /* home
  orientation(0.499, 0.500, 0.500, 0.501);
  Eigen::Vector3d(0.456, 0.02, 0.434);
  */

  // Define the approach pose
  Eigen::Isometry3d approach_pose = final_pose;
  approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.0);

  // Create the problem construction info
  trajopt::ProblemConstructionInfo pci(env);

  pci.kin = env->getManipulator(manip);

  pci.basic_info.n_steps = steps_per_phase * 2;
  pci.basic_info.manip = manip;
  pci.basic_info.dt_lower_lim = 0.5;    // 1/most time
  pci.basic_info.dt_upper_lim = 0.5;  // 1/least time
  pci.basic_info.start_fixed = true;
  pci.basic_info.use_time = true;

  pci.init_info.type = trajopt::InitInfo::STATIONARY;
  pci.init_info.dt = 0.5;

  // Add a collision cost
  if (true)
  {
    std::shared_ptr<trajopt::CollisionTermInfo> collision(new trajopt::CollisionTermInfo);
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
    collision->continuous = true;
    collision->first_step = 0;
    collision->last_step = pci.basic_info.n_steps - 1;
    collision->gap = 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 40);
    pci.cost_infos.push_back(collision);
  }

  // Add a velocity cost without time to penalize paths that are longer
  if (true)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->term_type = trajopt::TT_COST;
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cost";
    pci.cost_infos.push_back(jv);
  }

  // Add a velocity cnt with time to insure that robot dynamics are obeyed
  if (true)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);

    // Taken from iiwa documentation (radians/s) and scaled by 0.8
    std::vector<double> vel_lower_lim{ -0.86, -0.86, -0.86, -0.86,
                                       -0.86, -0.86, -0.86 };
    std::vector<double> vel_upper_lim{ 0.86, 0.86, 0.86, 0.86,
                                       0.86, 0.86, 0.86 };

    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->lower_tols = vel_lower_lim;
    jv->upper_tols = vel_upper_lim;
    jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cnt";
    pci.cnt_infos.push_back(jv);
  }

  // Add an acceleration cnt
  if (true)
  {
    std::shared_ptr<trajopt::JointAccTermInfo> ja(new trajopt::JointAccTermInfo);

    // Taken from iiwa documentation (radians/s) and scaled by 0.8
    std::vector<double> acc_lower_lim{ -0.3, -0.3, -0.3, -0.3,
                                       -0.3, -0.3, -0.3 };
    std::vector<double> acc_upper_lim{ 0.3, 0.3, 0.3, 0.3,
                                       0.3, 0.3, 0.3 };

    ja->targets = std::vector<double>(7, 0.0);
    ja->coeffs = std::vector<double>(7, 50.0);
    ja->lower_tols = acc_lower_lim;
    ja->upper_tols = acc_upper_lim;
    ja->term_type = (trajopt::TT_CNT);
    ja->first_step = 0;
    ja->last_step = pci.basic_info.n_steps - 1;
    ja->name = "joint_accelaration_cnt";
    pci.cnt_infos.push_back(ja);
  }


  // Add cartesian pose cnt at the approach point
  if (true)
  {
    Eigen::Quaterniond rotation(approach_pose.linear());
    std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
        std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = steps_per_phase;
    pose_constraint->xyz = approach_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(steps_per_phase);
    pci.cnt_infos.push_back(pose_constraint);
  }

//  if (false)
//  {
//    std::shared_ptr<trajopt::JointPosTermInfo> jp(new trajopt::JointPosTermInfo);

//    jp->targets = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.57};
//    jp->coeffs = std::vector<double>(7, 50.0);
//    jp->term_type = (trajopt::TT_CNT);
//    jp->first_step = 0;
//    jp->last_step = pci.basic_info.n_steps - 1;
//    jp->name = "joint_position_cnt";
//    pci.cnt_infos.push_back(jp);

//  }

  // // Add cartesian pose cnt at the final point
  // if (true)
  // {
  //   Eigen::Quaterniond rotation(final_pose.linear());
  //   std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
  //       std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
  //   pose_constraint->term_type = trajopt::TT_CNT;
  //   pose_constraint->link = end_effector;
  //   pose_constraint->timestep = 2 * steps_per_phase - 1;
  //   pose_constraint->xyz = final_pose.translation();

  //   pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
  //   pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  //   pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  //   pose_constraint->name = "pose_" + std::to_string(2 * steps_per_phase - 1);
  //   pci.cnt_infos.push_back(pose_constraint);
  // }

  // Add a cost on the total time to complete the pick
  if (true)
  {
    std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
    time_cost->name = "time_cost";
    time_cost->coeff = 5.0;
    time_cost->limit = 0.0;
    time_cost->term_type = trajopt::TT_COST;
    pci.cost_infos.push_back(time_cost);
  }

  // Create the pick problem
  trajopt::TrajOptProbPtr pick_prob = ConstructProblem(pci);

  // Set the optimization parameters (Most are being left as defaults)
  tesseract::tesseract_planning::TrajOptPlannerConfig config(pick_prob);
  config.params.max_iter = 500;

  // Create Plot Callback
  if (plotting_cb)
  {
    tesseract::tesseract_ros::ROSBasicPlottingPtr plotter_ptr(new tesseract::tesseract_ros::ROSBasicPlotting(env));
    config.callbacks.push_back(PlotCallback(*pick_prob, plotter_ptr));
  }

  // Create file write callback discarding any of the file's current contents
  std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
  if (file_write_cb)
  {
    std::string path = ros::package::getPath("trajopt_examples") + "/file_output_pick.csv";
    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, pick_prob));
  }

  // Solve problem. Results are stored in the response
  planner.solve(planning_response, config);

  if (file_write_cb)
    stream_ptr->close();

  // Plot the resulting trajectory
  plotter.plotTrajectory(env->getJointNames(), planning_response.trajectory.leftCols(env->getJointNames().size()));
  std::cout << planning_response.trajectory << '\n';

  

  ///////////////
  /// EXECUTE ///
  ///////////////
  // control the real robot ******************************************************************
  ROS_ERROR("Press enter to continue");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

 // Create action client to send trajectories
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execution_client("my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory",false);
  trajectory_msgs::JointTrajectory traj_msg;
  execution_client.waitForServer(ros::Duration(1.0));



//  // Convert TrajArray (Eigen Matrix of joint values) to ROS message
//  ros::Duration t(0.25);
//  traj_msg = trajArrayToJointTrajectoryMsg(planning_response.joint_names, planning_response.trajectory, true, true, t);

//  // Create action message
//  trajectory_action.trajectory = traj_msg;

  // using jt directly
   ros::Duration t(0.25);
   trajArrayToJointTrajectory_moveit(planning_response.joint_names, planning_response.trajectory, robot_model,true, true, t);

   // Send to hardware
   execution_client.sendGoal(trajectory_action);
   execution_client.waitForResult(ros::Duration(10.0));

   if (execution_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   {
     std::cout << "succeeded! \n";
   }
   else
   {
     std::cout << "failed \n";
   }



  ROS_INFO("Done");
  ros::spin();
}
