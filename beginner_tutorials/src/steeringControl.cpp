#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "beginner_tutorials/steeringControl.h"
#include "beginner_tutorials/savePath.h"

#define PATH_RIGHT_TURN		0
#define PATH_LEFT_TURN		1
#define PATH_SIN			2
#define PATH_STRAIGHT		3
#define path_type			PATH_RIGHT_TURN
#define PI 3.14159265

// LOD types
#define LOD_PARKING_LOT		0
#define LOD_STANDARD		1
#define lod_type			LOD_STANDARD
// 4167 for pl, 17802 for standard
#define LOD_LENGTH 4167				// number of points in the line of death

#define MAX_TIRE_ANGLE (35.0*PI/180.0)	// radians
#define APPROX_ARC1_LENGTH 15			// meters
#define APPROX_ARC2_LENGTH 15			// meters
#define APPROX_ARC3_LENGTH 10			// meters
#define APPROX_PATH_LENGTH (APPROX_ARC1_LENGTH+APPROX_ARC2_LENGTH+APPROX_ARC3_LENGTH)
#define PATH_STEP 0.01					// meters
#define PATH_AMPLITUDE 5				// meters
#define PATH_FREQUENCY (PI/40)		// radians
#define MAX_ACCELERATION 10				// meters per seconds squared
#define BRAKING_LIMIT 0.5
#define ACCELERATION_LIMIT 0.20
#define MIN_VELOCITY 0.25
#define MAX_VELOCITY 10
#define OMEGA_GAIN 10

#define DRIVE 127
#define NEUTRAL 128

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

// The next four functions were copied directly from the JAUS Comfort Controller code (Author: Sean Whitsitt), this is the computational load we are alleviating with ROS
double wrapTo2Pi(double angle) {
	while (angle > 2 * PI) {
		angle -= 2 * PI;
	}
	while (angle < 0) {
		angle += 2 * PI;
	}
	return angle;
}

double wrapToPi(double angle) {
	while (angle > PI) {
		angle -= 2 * PI;
	}
	while (angle < -PI) {
		angle += 2 * PI;
	}
	return angle;
}

double fromAngleToAngle(double from, double to) {
	from = wrapTo2Pi(from);
	to = wrapTo2Pi(to);
	return wrapToPi(to - from);
}

void steering_controller(double *omega, double *delta_d, int *path_index, double x, double y, double velocity, double heading, double tire_angle, double *path_x, double *path_y, int flag) {
	double om = *omega;
	int path_i = *path_index;
	double K = 0.5;

	double distance = 0;
	if (path_i < APPROX_PATH_LENGTH / PATH_STEP) {
		distance = sqrt((x - path_x[path_i]) * (x - path_x[path_i]) + (y - path_y[path_i]) * (y - path_y[path_i]));
		double next = sqrt((x - path_x[path_i + 1]) * (x - path_x[path_i + 1]) + (y - path_y[path_i + 1]) * (y - path_y[path_i + 1]));
		for (int ii = path_i; ii < APPROX_PATH_LENGTH / PATH_STEP - 2; ii++) {
			if (distance >= next) {
				distance = next;
				path_i = ii;
			} else {
				break;
			}
			next = sqrt((x - path_x[path_i + 1]) * (x - path_x[path_i + 1]) + (y - path_y[path_i + 1]) * (y - path_y[path_i + 1]));
		}
	} else {
		*omega = 0;
		*delta_d = 0;
		*path_index = APPROX_PATH_LENGTH / PATH_STEP;
		return;
	}

	double tangent = 0;
	if (path_i < APPROX_PATH_LENGTH / PATH_STEP) {
		tangent = atan2(path_y[path_i + 1] - path_y[path_i], path_x[path_i + 1] - path_x[path_i]);
	} else {
		*omega = 0;
		*delta_d = 0;
		*path_index = APPROX_PATH_LENGTH / PATH_STEP;
		return;
	}

	double delta = 0;
	if (flag == 1 || (distance < sqrt((path_x[path_i] - path_x[path_i + 1]) * (path_x[path_i] - path_x[path_i + 1]) + (path_y[path_i] - path_y[path_i + 1]) * (path_y[path_i] - path_y[path_i + 1])))) {
		delta = fromAngleToAngle(heading, tangent);
		*delta_d = MIN(MAX(delta, -MAX_TIRE_ANGLE), MAX_TIRE_ANGLE);
//#ifdef EFFORT_MODE
//		*omega = 0;
//#endif
//#ifdef POSITION_MODE
		*omega = *delta_d;
//#endif
		*path_index = path_i;
		return;
	} else {
		double angleToPath = atan2(path_y[path_i] - y, path_x[path_i] - x);
		double directionToPath = fromAngleToAngle(heading, angleToPath);
		if (directionToPath > 0) {
			delta = fromAngleToAngle(heading, tangent) + atan((K * distance) / velocity);
		} else {
			delta = fromAngleToAngle(heading, tangent) - atan((K * distance) / velocity);
		}
	}
	delta = MIN(MAX(delta, -MAX_TIRE_ANGLE), MAX_TIRE_ANGLE);

//#ifdef EFFORT_MODE
//	om = fromAngleToAngle(tire_angle, delta);
//#endif
//#ifdef POSITION_MODE
	om = delta;
//#endif

	*delta_d = delta;
	*omega = om;
	*path_index = path_i;
}

//All of the rest of the code below was written by Ethan Rabb for this node.

//This class provides the callback functions needed to perform the Comfort Control math, while allowing for access to the path data and booleans specifying whether this data has been defined yet.
class SteeringCaller {
	public:
		bool haveBoth;
		bool haveX;
		double path_x[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
		double path_y[(int)(APPROX_PATH_LENGTH / PATH_STEP)];

		bool savePath(beginner_tutorials::savePath::Request &req, beginner_tutorials::savePath::Response &res) {
			ROS_INFO("I got called!");
			if (!haveBoth) {
				if (!haveX) {
					int xi = 0;
					//beginner_tutorials::Float64MultiArray path = req.path;
					for (std::vector<double>::const_iterator xit = req.path.data.begin(); xit != req.path.data.end(); ++xit) {
						path_x[xi] = *xit;
						++xi;
					}
					//printf("the data is %f\n",req.path.data[0]);
					haveX = true;
					ROS_INFO("X path received.");
					res.check = true;
					return true;
				}
				else {
					int yi = 0;
					for (std::vector<double>::const_iterator yit = req.path.data.begin(); yit != req.path.data.end(); ++yit) {
						path_y[yi] = *yit;
						++yi;
					}
					haveBoth = true;
					res.check = true;
					ROS_INFO("Y path received.");
					return true;
				}
			}
			else {ROS_INFO("Request to receive arrays denied, restart node to refill"); res.check = false; return false;}
		}
		
		bool calcSteering(beginner_tutorials::steeringControl::Request &req, beginner_tutorials::steeringControl::Response &res)
		{
		  if (haveBoth) {
			  //ROS_INFO("request recieved");
			  double * omega_point = &req.omega;
			  double * delta_d_point = &req.delta_d;
			  int * path_index_point = &req.path_index;
			  steering_controller(omega_point, delta_d_point, path_index_point, req.x, req.y, req.velocity, req.heading, req.tire_angle, path_x, path_y, req.flag);
			  res.omega = req.omega;
			  res.delta_d = req.delta_d;
			  res.path_index = req.path_index;
			  //ROS_INFO("sending back response");
			  return true;
		  }
		  else {
			//ROS_INFO("Request to calculate steering denied, paths have not been sent!");
			return false;	
		  }
		}
};

void callbackpath(const std_msgs::Float64MultiArray::ConstPtr& path) {
	ROS_INFO("now it worked");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "steering_control_server");
  ros::NodeHandle servNode;
  SteeringCaller steerCall;
  steerCall.haveX = false; steerCall.haveBoth = false;
  ros::ServiceServer pathServ = servNode.advertiseService("save_path", &SteeringCaller::savePath, &steerCall);
  ros::ServiceServer steerServ = servNode.advertiseService("steering_control", &SteeringCaller::calcSteering, &steerCall);
  ROS_INFO("Ready for control input");
  ros::spin();
  return 0;
}

