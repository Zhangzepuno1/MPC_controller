#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

#include <stdio.h>
#include <string>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <libgen.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <cr_msgs/CarInfo.h>
#include <cr_msgs/ControlCommand.h>
#include <cr_msgs/PathPlanner.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

using namespace std;
#define PI 3.1415926

class crmpc
{
  private:
    nav_msgs::Odometry current_pose;
    cr_msgs::CarInfo info;

    ros::Publisher cmdPub, debugPub;
    std::vector<ros::Subscriber> subs;

    MPC mpc;
    double speed_now;
  private:
      // Fit a polynomial.
    // Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
      assert(xvals.size() == yvals.size());
      assert(order >= 1 && order <= xvals.size() - 1);
      Eigen::MatrixXd A(xvals.size(), order + 1);

      for (int i = 0; i < xvals.size(); ++i) {
        A(i, 0) = 1.0;
      }

      for (int j = 0; j < xvals.size(); ++j) {
        for (int i = 0; i < order; i++) {
          A(j, i + 1) = A(j, i) * xvals(j);
        }
      }

      auto Q = A.householderQr();
      auto result = Q.solve(yvals);
      return result;
    }

    double  normalizeAngle(double angle)
    {
        while(angle > 2 * PI)
        {
            angle -=  PI *  2;
        }
        while (angle < 0)
        {
            angle += PI * 2;
        }
        return angle;
    }
public:
    crmpc()
    {
      ros::NodeHandle nh("~");

      fprintf(stderr, "crcontrol init...!\n");

      this->subs.push_back(nh.subscribe("/current_pose", 5, &crmpc::onPose, this));
      this->subs.push_back(nh.subscribe("/planner_path", 100, &crmpc::update,this));
      this->subs.push_back(nh.subscribe("/base_info", 100, &crmpc::onBaseInfo, this));


      this->cmdPub = nh.advertise<cr_msgs::ControlCommand>("/ctrl_cmd", 10);
      this->debugPub = nh.advertise<visualization_msgs::Marker>("/mpc_debug", 20);
    }
    void onPose(const nav_msgs::Odometry &msg)
    {
        this->current_pose = msg;
    }
    void onBaseInfo(const cr_msgs::CarInfo &msg)
    {
        this->info = msg;
    }
    void path2Marker(std::vector<double> &x, std::vector<double> &y)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns =  "mpc_points";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.id = 10;

        for(int i = 0 ; i< N; i++)
        {
          geometry_msgs::Point p;
          p.x = x[i];
          p.y = y[i];
          p.z = 0.0;

          marker.points.push_back(p);
        }

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1 ;
        marker.color.g = 0 ;
        marker.color.b = 0 ;
        marker.color.a = 1;

        marker.lifetime = ros::Duration();
        this->debugPub.publish(marker);
    }

    /*void publishPath(std::vector<double> &x, std::vector<double> &y)
    {
      visualization_msgs::MarkerArray marker_array;

      for(int i =0; i< N; i++)
        marker_array.markers.push_back(this->path2Marker(x[i], y[i], i));

      this->debugPub.publish(marker_array);
    }*/
    void update(const cr_msgs::PathPlanner &msg)
    {
      //current status
      int length = 10;
      printf("length = %d\n", length);
      vector<double> points_xs; //receive path points
      vector<double> points_ys;

      for(int i =0; i< length; i++)
      {
        points_xs.push_back(msg.x[i]);
        points_ys.push_back(msg.y[i]);
      }

      double px = this->current_pose.pose.pose.position.x;
      double py = this->current_pose.pose.pose.position.y;
      double psi = tf::getYaw(this->current_pose.pose.pose.orientation);
      double v = this->speed_now;//this->current_pose.twist.twist.linear.x;
      double delta = this->current_pose.twist.twist.angular.z;
      double a = 0;
      psi = normalizeAngle(psi);
      //cout<< "current_x " << px << " current_y "<< py << " heading " << psi << endl;
      //**************************************************************
      //* CONVERT WAYPOINTS TO VEHICLE SPACE as VectorXd from GLOBAL SPACE
      //**************************************************************
      const int NUMBER_OF_WAYPOINTS = 10;
      Eigen::VectorXd waypoints_xs(NUMBER_OF_WAYPOINTS);
      Eigen::VectorXd waypoints_ys(NUMBER_OF_WAYPOINTS);

      for(int i = 0; i < NUMBER_OF_WAYPOINTS; ++i)
      {
        const double dx = points_xs.at(i) - px;
        const double dy = points_ys.at(i) - py;

        waypoints_xs[i] = dx * cos(psi) + dy * sin(psi);
        waypoints_ys[i] = dy * cos(psi) - dx * sin(psi);
        //cout<< "d_x " << dx << " d_y "<< dy << endl;
        //cout<< "anchor_x " << points_xs.at(i) << " anchor_y "<< points_ys.at(i) << endl;
        //cout<< "waypoints_x " << waypoints_xs[i] << " waypoints_y "<< waypoints_ys[i] << endl;
      }

      //**************************************************************
      //* FIT POLYNOMAL
      //**************************************************************
      const int ORDER = 3;
      auto K = polyfit(waypoints_xs, waypoints_ys, ORDER);

      //cout<<"k0 "<< K[0] <<" k1 " << K[1] << " k2 " << K[2] << " k3 " << K[3]<<endl;

      //**************************************************************
      //* GET POINTS TO DISPLAY FROM OUR FITTED POLYNOMIAL (ROAD CURVE)
      //**************************************************************
      std::vector<double> next_xs(N);
      std::vector<double> next_ys(N);
      std::vector<double> next_xs_g(N);
      std::vector<double> next_ys_g(N);
      const double D = 0.6;//D is the step 

      for (int i = 0; i < N; ++i)
      {

        const double dx = D * i;
        const double dy = K[3] * dx * dx * dx + K[2] * dx * dx + K[1] * dx + K[0];

        next_xs[i] = dx;
        next_ys[i] = dy;
      }
      path2Marker(next_xs, next_ys);
      //**************************************************************
      //* GENERATE CURRENT ERROR ESTIMATES (cte, epsi)
      //**************************************************************

      // current CTE is fitted polynomial (road curve) evaluated at px = 0.0
      // f = K[3] * px0 * px0 + px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
      const double cte = K[0];

      // current heading error epsi is the tangent to the road curve at px = 0.0
      // epsi = arctan(f') where f' is the derivative of the fitted polynomial
      // f' = 3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]
      const double epsi = -atan(K[1]);

      //**************************************************************
      //* GET THE CURRENT DELAYED STATE
      //**************************************************************

      const double dt = 0;//accounts for delay of actuators(throttle and brake) 
      const double Lf = 2.67;//distance between the front of vehicle to its center of gravity

      // current state must be in vehicle coordinates with the delay factored in
      // kinematic model is at play here
      // note that at current state at vehicle coordinates:
      // px, py, psi = 0.0, 0.0, 0.0
      // note that in vehicle coordinates it is going straight ahead the x-axis
      // which means position in vehicle's y-axis does not change
      // the steering angle is negative the given value as we have
      // as recall that during transformation we rotated all waypoints by -psi
      const double current_px = 0.0 + v * dt;
      const double current_py = 0.0;
      const double current_psi = 0.0 + v * (-delta) / Lf * dt;//still need to prove this equation
      const double current_v = v + a * dt;
      const double current_cte = cte + v * sin(epsi) * dt;
      const double current_epsi = epsi + v * (-delta) / Lf * dt;

      const int NUMBER_OF_STATES = 6;
      Eigen::VectorXd state(NUMBER_OF_STATES);
      state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;

      //**************************************************************
      //* DETERMINE NEXT COURSE OF ACTION AND PREDICTED STATES
      //* USING MODEL PREDICTIVE CONTROL
      //**************************************************************
      mpc.solve(state, K);

      cr_msgs::ControlCommand cmd;
      cmd.stamp = ros::Time::now();
      cmd.linear_velocity = mpc.throttle;
      cmd.steering_angle = -mpc.steer;
      this->speed_now = mpc.throttle;
      cout << setw(20) << cmd.linear_velocity << setw(20) << cmd.steering_angle << endl;
      this->cmdPub.publish(cmd);

      //return 0;
    }

    void run()
    {
        while (1)
        {
            ros::spinOnce();
            usleep(10 * 1000);
        }
    }

};

crmpc *mpc;
void timeout(int sig)
{
    fprintf(stderr, ".");
    //mpc->update();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crmpc");
    mpc = new crmpc();

/*
    struct itimerval tick;
    memset(&tick, 0, sizeof(tick));
    tick.it_value.tv_sec = 0;
    tick.it_value.tv_usec = 1;
    tick.it_interval.tv_sec = 0;
    tick.it_interval.tv_usec = 20 * 1000;

    signal(SIGALRM, timeout);
    if (setitimer(ITIMER_REAL, &tick, NULL) < 0)
        printf("Set timer failed!\n");
        */
    mpc->run();
    return 0;
}
