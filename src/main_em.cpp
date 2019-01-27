/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>

#include "ratslam/experience_map.h"
#include <ratslam_ros/TopologicalAction.h>
#include <nav_msgs/Odometry.h>
#include "graphics/experience_map_scene.h"
#include <ratslam_ros/TopologicalMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>

ros::Publisher pub_em;
ros::Publisher pub_pose;
ros::Publisher pub_em_markers;
ros::Publisher pub_goal_path;
geometry_msgs::PoseStamped pose_output;
ratslam_ros::TopologicalMap em_map;
visualization_msgs::Marker em_marker;

#ifdef HAVE_IRRLICHT
#include "graphics/experience_map_scene.h"
ratslam::ExperienceMapScene *ems;
bool use_graphics;
#endif

using namespace ratslam;

//CHR - BEGIN
int EXP_MAP_NUM;
double x_m = 0, y_m = 0, th_rad = 0;
std::vector<int> action_counter = {}, src_id_counter= {}, dest_id_counter= {};
int prev_em_id = -1;
bool is_nao = false;
//CHR - END


void odo_callback(nav_msgs::OdometryConstPtr odo, ratslam::ExperienceMap *em[]) //CHR
{
  ROS_DEBUG_STREAM(
          "EM:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << odo->twist.twist.linear.x
                             << " r=" << odo->twist.twist.angular.z);

  static ros::Time prev_time(0);
  //CHR - BEGIN
  //Convert position into velocity (using the quaternion implementation)
  double vtrans = 0, vrot = 0;
  static double prev_x = 0, prev_y = 0, prev_angle = 0;
  double curr_angle = 0;
  irr::core::quaternion irr_quaternion = irr::core::quaternion((float) odo->pose.pose.orientation.x, (float) odo->pose.pose.orientation.y,
                                                               (float) odo->pose.pose.orientation.z, (float) odo->pose.pose.orientation.w);
  irr::core::vector3df vector3df;
  //CHR - END

  if (prev_time.toSec() > 0)
  {
    double time_diff = (odo->header.stamp - prev_time).toSec();
    //CHR - BEGIN
    if (is_nao) {
      //euclidean distance between current and previous position
      vtrans = (sqrt(pow(odo->pose.pose.position.x - prev_x,2) + pow(odo->pose.pose.position.y - prev_y, 2))) / time_diff;
      irr_quaternion.toEuler(vector3df);
      curr_angle = vector3df.Z;
      //difference between angles
      vrot = (curr_angle - prev_angle) / time_diff;
    }
    //call "on_odo" for each Experience Map
    for (int i=0; i < EXP_MAP_NUM; i++) {
      //if NAO, use the results from the calcuation above, otherwise: continue normally
      if (is_nao) {
        em[i]->on_odo(vtrans, vrot, time_diff);
      } else {
        em[i]->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff); //[i]
      }
    }
    //CHR - END
  }

  static ros::Time prev_goal_update(0);

  for (int i = 0; i < EXP_MAP_NUM; i++) { //CHR
    if (em[i]->get_current_goal_id() >= 0) //CHR (i)
    {
      // (prev_goal_update.toSec() == 0 || (odo->header.stamp - prev_goal_update).toSec() > 5)
      //em->calculate_path_to_goal(odo->header.stamp.toSec());

      prev_goal_update = odo->header.stamp;

      em[i]->calculate_path_to_goal(odo->header.stamp.toSec()); //CHR (i)

      static nav_msgs::Path path;
      if (em[i]->get_current_goal_id() >= 0) //CHR (i)
      {
        em[i]->get_goal_waypoint(); //CHR (i)

        static geometry_msgs::PoseStamped pose;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = std::to_string(i); //CHR ="1";

        pose.header.seq = 0;
        pose.header.frame_id = std::to_string(i); //CHR = "1";
        path.poses.clear();
        unsigned int trace_exp_id = em[i]->get_goals()[0]; //CHR (i)
        while (trace_exp_id != em[i]->get_goal_path_final_exp()) //CHR (i)
        {
          pose.pose.position.x = em[i]->get_experience(trace_exp_id)->x_m; //CHR (i)
          pose.pose.position.y = em[i]->get_experience(trace_exp_id)->y_m; //CHR (i)
          path.poses.push_back(pose);
          pose.header.seq++;

          trace_exp_id = em[i]->get_experience(trace_exp_id)->goal_to_current; //CHR (i)
        }

        pub_goal_path.publish(path);

        path.header.seq++;

      } else {
        path.header.stamp = ros::Time::now();
        path.header.frame_id = std::to_string(i); //CHR = "1";
        path.poses.clear();
        pub_goal_path.publish(path);

        path.header.seq++;
      }
    }
  } //CHR

  prev_time = odo->header.stamp;
  //CHR - BEGIN
  //save the current position and angle for the next run
  if (is_nao) {
    prev_x = odo->pose.pose.position.x;
    prev_y = odo->pose.pose.position.y;
    prev_angle = curr_angle;
  }
  //CHR - END
}

//CHR - BEGIN
//void action_callback(ratslam_ros::TopologicalActionConstPtr action, ratslam::ExperienceMap *em)
void action_callback(ratslam_ros::TopologicalActionConstPtr action, ratslam::ExperienceMap *em_array[])
//CHR - END
{
  ROS_DEBUG_STREAM("EM:action_callback{" << ros::Time::now() << "} action=" << action->action << " src=" << action->src_id << " dst=" << action->dest_id);

  //CHR - BEGIN
  //get the particular Experience Map
  int curr_em_id = action->em_id;
  ExperienceMap *em = em_array[curr_em_id];

  //if only a single experience map is desired, continue with the original implementation
  if (EXP_MAP_NUM > 1) {
    if (action->first) {
      //reset variables
      x_m = 0;
      y_m = 0;
      th_rad = 0;
      action_counter.clear();
      src_id_counter.clear();
      dest_id_counter.clear();
      prev_em_id = -1;
    }

    //Add "No_Action" messages to the Experience Maps
    for (int i = prev_em_id + 1; i < action->em_id; i++) {
      action_counter.push_back(0);
    }
    if (action->last) {
      for (int i = action->em_id + 1 ; i < EXP_MAP_NUM - 1; i++) {
        action_counter.push_back(0);
      }
    }

    //save the actions and ids for the calculation of the average map
    action_counter.push_back(action->action);
    src_id_counter.push_back(action->src_id);
    dest_id_counter.push_back(action->dest_id);
    prev_em_id = action->em_id;
  }
  //CHR - END

  switch (action->action) {
    case ratslam_ros::TopologicalAction::CREATE_NODE:
      em->on_create_experience(action->dest_id, true); //CHR
      em->on_set_experience(action->dest_id, 0);
      break;

    case ratslam_ros::TopologicalAction::CREATE_EDGE:
      em->on_create_link(em->get_current_id(), action->dest_id, action->relative_rad);
      em->on_set_experience(action->dest_id, action->relative_rad);
      break;

    case ratslam_ros::TopologicalAction::SET_NODE:
      em->on_set_experience(action->dest_id, action->relative_rad);
      break;

  }
  em->iterate();

  //CHR - BEGIN
  //sum x,y and th values up to build an average
  x_m += em->get_experience(em->get_current_id())->x_m;
  y_m += em->get_experience(em->get_current_id())->y_m;
  th_rad += em->get_experience(em->get_current_id())->th_rad;

  //if last message is received, continue with the average Experience Map
  if (action->last){
    int best_action = 0;

    //Average Experience Map (last one)
    curr_em_id = EXP_MAP_NUM - 1;
    ExperienceMap *avg_em = em_array[curr_em_id];

    //if only a single experience map is desired, continue with the original implementation
    if (EXP_MAP_NUM > 1) {

      //calculate the average position
      x_m = ((double) 1/(EXP_MAP_NUM - 1)) * x_m;
      y_m = ((double) 1/(EXP_MAP_NUM - 1)) * y_m;
      th_rad = ((double) 1/(EXP_MAP_NUM - 1)) * th_rad;


      //choose best action
      int count_best_action = 0, tmp_count_action = 0;
      for (int i=0; i<=3; i++) {
        tmp_count_action = (int) std::count(action_counter.begin(), action_counter.end(), i);
        if (tmp_count_action > count_best_action){
          count_best_action = tmp_count_action;
          best_action = i;
          //Draw: choose the action of the relatively "best" (earliest) pose hypothesis
        } else if (tmp_count_action == count_best_action && tmp_count_action > 0){
          if(distance(action_counter.begin(), find(action_counter.begin(), action_counter.end(), i)) <
             distance(action_counter.begin(), find(action_counter.begin(), action_counter.end(), best_action))){
            best_action = i;
          }
        }
      }

      //append missing experience (without an link) to have the same number of experiences in each Experience Map
      for (int i=0;i<EXP_MAP_NUM-1;i++) {
        for (int j = i+1; j < EXP_MAP_NUM-1; j++) {
          if (em_array[j]->get_num_experiences() > em_array[i]->get_num_experiences()) {
            em_array[i]->on_create_experience(0, false);
          } else if (em_array[j]->get_num_experiences() < em_array[i]->get_num_experiences()){
            em_array[j]->on_create_experience(0, false);
          }
        }
      }

      //If necessary do the same in the average experience map
      if (best_action != 1){
        for (int i = 0; i < EXP_MAP_NUM-1; i++) {
          if (em_array[i]->get_num_experiences() > em_array[EXP_MAP_NUM-1]->get_num_experiences()) {
            em_array[EXP_MAP_NUM-1]->on_create_experience(0, false);
          }
        }
      }

      //test, if best action equal to "No action"
      if (best_action > 0) {

        //choose destination id
        unsigned int best_dest_id = 0; int j=0;
        for (int i=0;i<action_counter.size();i++){
          if (action_counter[i] == best_action){
            best_dest_id = (unsigned int) dest_id_counter[j];
            break;
          } else if (action_counter[i] > 0){
            j++;
          }
        }

        unsigned int best_src_id = 0;
        //test, if action equal to "Create Link"
        if(best_action == 2){
          //choose src_id
          j = 0;
          for (int i=0;i<action_counter.size();i++){
            if (action_counter[i] == best_action){
              best_src_id = (unsigned  int) src_id_counter[j];
              break;
            } else if (action_counter[i] > 0){
              j++;
            }
          }
        }

        //execute action on average Experience Map
        switch (best_action) {
          case ratslam_ros::TopologicalAction::CREATE_NODE:
            avg_em->on_create_experience(best_dest_id, true);
            avg_em->on_set_experience(best_dest_id, 0);
            break;

          case ratslam_ros::TopologicalAction::CREATE_EDGE:
            avg_em->on_create_link(avg_em->get_current_id(), best_dest_id, action->relative_rad);
            avg_em->on_set_experience(best_dest_id, action->relative_rad);
            break;

          case ratslam_ros::TopologicalAction::SET_NODE:
            avg_em->on_set_experience(best_dest_id, action->relative_rad);
            break;
        }
        avg_em->iterate();

        //calculate distances between the experiences for ranking
        for (int i = 0; i < EXP_MAP_NUM - 1; i++) {
          em_array[i]->set_exp_distance(em_array[i]->get_current_id(), x_m, y_m);
        }

        //calculate aberration
        double *aberration = new double[EXP_MAP_NUM - 1];
        int max_aberration = 0, min_aberration = 0;
        int n = 10; //consider last 10 experiences
        for (int i = 0; i < EXP_MAP_NUM - 1; i++) {
          aberration[i] = 0;
          int j = 0;
          if ((em_array[i]->get_num_experiences() - n) > 0) {
            j = em_array[i]->get_num_experiences() - n;
          }
          for (j; j < em_array[i]->get_num_experiences(); j++) {
            aberration[i] = aberration[i] + (em_array[i]->get_num_experiences() - j) *
                                            em_array[i]->get_exp_distance(em_array[i]->get_current_id());
          }
          aberration[i] = ((double) 1 / n) * aberration[i];

          //get max and min
          if (aberration[i] > aberration[max_aberration]) {
            max_aberration = i;
          }
          if (aberration[i] < aberration[min_aberration]) {
            min_aberration = i;
          }
        }

        //rank worst Experience Map (if possible) and replace if necessary
        if (min_aberration != max_aberration) {
          em_array[max_aberration]->counter_ranked_worst++;
          if (em_array[max_aberration]->counter_ranked_worst == 4) {
            *em_array[max_aberration] = *em_array[min_aberration];
            em_array[max_aberration]->counter_ranked_worst = 0;
          }
        }
      }
    } else {
      best_action = action->action;
    }

    if (best_action > 0) {
      //CHR - END
      pose_output.header.stamp = ros::Time::now();
      pose_output.header.seq++;
      pose_output.header.frame_id = std::to_string(curr_em_id); //CHR "1"
      pose_output.pose.position.x = avg_em->get_experience(avg_em->get_current_id())->x_m;
      pose_output.pose.position.y = avg_em->get_experience(avg_em->get_current_id())->y_m;
      pose_output.pose.position.z = 0;
      pose_output.pose.orientation.x = 0;
      pose_output.pose.orientation.y = 0;
      pose_output.pose.orientation.z = sin(avg_em->get_experience(avg_em->get_current_id())->th_rad / 2.0);
      pose_output.pose.orientation.w = cos(avg_em->get_experience(avg_em->get_current_id())->th_rad / 2.0);
      pub_pose.publish(pose_output);

      static ros::Time prev_pub_time(0);

      if (action->header.stamp - prev_pub_time > ros::Duration(30.0)) {
        prev_pub_time = action->header.stamp;

        em_map.header.stamp = ros::Time::now();
        em_map.header.seq++;
        em_map.node_count = avg_em->get_num_experiences();
        em_map.node.resize(avg_em->get_num_experiences());
        for (int i = 0; i < avg_em->get_num_experiences(); i++) {
          em_map.node[i].id = avg_em->get_experience(i)->id;
          em_map.node[i].pose.position.x = avg_em->get_experience(i)->x_m;
          em_map.node[i].pose.position.y = avg_em->get_experience(i)->y_m;
          em_map.node[i].pose.orientation.x = 0;
          em_map.node[i].pose.orientation.y = 0;
          em_map.node[i].pose.orientation.z = sin(avg_em->get_experience(i)->th_rad / 2.0);
          em_map.node[i].pose.orientation.w = cos(avg_em->get_experience(i)->th_rad / 2.0);
        }

        em_map.edge_count = avg_em->get_num_links();
        em_map.edge.resize(avg_em->get_num_links());
        for (int i = 0; i < avg_em->get_num_links(); i++) {
          em_map.edge[i].source_id = avg_em->get_link(i)->exp_from_id;
          em_map.edge[i].destination_id = avg_em->get_link(i)->exp_to_id;
          em_map.edge[i].duration = ros::Duration(avg_em->get_link(i)->delta_time_s);
          em_map.edge[i].transform.translation.x = avg_em->get_link(i)->d * cos(avg_em->get_link(i)->heading_rad);
          em_map.edge[i].transform.translation.y = avg_em->get_link(i)->d * sin(avg_em->get_link(i)->heading_rad);
          em_map.edge[i].transform.rotation.x = 0;
          em_map.edge[i].transform.rotation.y = 0;
          em_map.edge[i].transform.rotation.z = sin(avg_em->get_link(i)->facing_rad / 2.0);
          em_map.edge[i].transform.rotation.w = cos(avg_em->get_link(i)->facing_rad / 2.0);
        }
        pub_em.publish(em_map);
      }

      em_marker.header.stamp = ros::Time::now();
      em_marker.header.seq++;
      em_marker.header.frame_id = std::to_string(curr_em_id); //CHR ="1";
      em_marker.type = visualization_msgs::Marker::LINE_LIST;
      em_marker.points.resize(avg_em->get_num_links() * 2);
      em_marker.action = visualization_msgs::Marker::ADD;
      em_marker.scale.x = 0.01;
      //em_marker.scale.y = 1;
      //em_marker.scale.z = 1;
      em_marker.color.a = 1;
      em_marker.ns = "em";
      em_marker.id = 0;
      em_marker.pose.orientation.x = 0;
      em_marker.pose.orientation.y = 0;
      em_marker.pose.orientation.z = 0;
      em_marker.pose.orientation.w = 1;
      for (int i = 0; i < avg_em->get_num_links(); i++) {
        em_marker.points[i * 2].x = avg_em->get_experience(avg_em->get_link(i)->exp_from_id)->x_m;
        em_marker.points[i * 2].y = avg_em->get_experience(avg_em->get_link(i)->exp_from_id)->y_m;
        em_marker.points[i * 2].z = 0;
        em_marker.points[i * 2 + 1].x = avg_em->get_experience(avg_em->get_link(i)->exp_to_id)->x_m;
        em_marker.points[i * 2 + 1].y = avg_em->get_experience(avg_em->get_link(i)->exp_to_id)->y_m;
        em_marker.points[i * 2 + 1].z = 0;
      }

      pub_em_markers.publish(em_marker);

      #ifdef HAVE_IRRLICHT
        if (use_graphics) {
          ems->update_scene();
          ems->draw_all();
        }
      #endif
    } //CHR
  }
}

//CHR - BEGIN
void set_goal_pose_callback(geometry_msgs::PoseStampedConstPtr pose, ratslam::ExperienceMap * em[]) {
  for (int k=1; k < EXP_MAP_NUM; k++) {
    em[k]->add_goal(pose->pose.position.x, pose->pose.position.y);
  }
//CHR - END
}

int main(int argc, char * argv[])
{
  ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  std::string topic_root = "";
  boost::property_tree::ptree settings, general_settings, ratslam_settings;
  read_ini(argv[1], settings);

  get_setting_child(ratslam_settings, settings, "ratslam", true);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");
  get_setting_from_ptree(EXP_MAP_NUM, ratslam_settings, "exp_map_num", 1); //CHR

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMExperienceMap");
  }
  ros::NodeHandle node;

  //CHR - BEGIN
  //if desired, created multiple Experience Maps plus one average map
  if (EXP_MAP_NUM > 1){
    EXP_MAP_NUM++;
  }
  ratslam::ExperienceMap *em[EXP_MAP_NUM];
  for (int i = 0; i < EXP_MAP_NUM; i++) {
    em[i] = new ratslam::ExperienceMap(ratslam_settings);
  }
  //CHR - END

  pub_em = node.advertise<ratslam_ros::TopologicalMap>(topic_root + "/ExperienceMap/Map", 1);
  pub_em_markers = node.advertise<visualization_msgs::Marker>(topic_root + "/ExperienceMap/MapMarker", 1);

  pub_pose = node.advertise<geometry_msgs::PoseStamped>(topic_root + "/ExperienceMap/RobotPose", 1);

  pub_goal_path = node.advertise<nav_msgs::Path>(topic_root + "/ExperienceMap/PathToGoal", 1);

  //CHR - BEGIN
  ros::Subscriber sub_odometry;
  if (!topic_root.compare("nao")) {
    is_nao = true;
    sub_odometry = node.subscribe<nav_msgs::Odometry>("/odom", 0, boost::bind(odo_callback, _1, em),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());
  } else {
    sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, em),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());
  }
  //CHR - END

  ros::Subscriber sub_action = node.subscribe<ratslam_ros::TopologicalAction>(
          topic_root + "/PoseCell/TopologicalAction", 0, boost::bind(action_callback, _1, em),
          ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  ros::Subscriber sub_goal = node.subscribe<geometry_msgs::PoseStamped>(topic_root +"/ExperienceMap/SetGoalPose", 0,
                                                                        boost::bind(set_goal_pose_callback, _1, em),
                                                                        ros::VoidConstPtr(),
                                                                        ros::TransportHints().tcpNoDelay());

#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  if (use_graphics)
  {
    ems = new ratslam::ExperienceMapScene(draw_settings, em[EXP_MAP_NUM - 1]); //CHR draw average Experience Map
  }
#endif

  ros::spin();

  return 0;
}

