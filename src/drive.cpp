#include "main.h"
#include "units.hpp"

//Multithreading protection
pros::Mutex threadProtector;

//Declare new type "vector_t" to effeciently hold vector data
struct vector_t{
  double x;
  double y;
  double a;
  double magnitude;
};

//Generalized template function to get the sign of a number
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


//General drivetrain functions
void drive_voltage(int left, int right) {
  driveFL.move_voltage(left);
  driveML.move_voltage(left);
  driveBL.move_voltage(left);

  driveFR.move_voltage(right);
  driveMR.move_voltage(right);
  driveBR.move_voltage(right);
}

void drive_velocity(int left, int right) {
  driveFL.move_velocity(left);
  driveML.move_velocity(left);
  driveBL.move_velocity(left);

  driveFR.move_velocity(right);
  driveMR.move_velocity(right);
  driveBR.move_velocity(right);
}


//Op control specific drivetrain function
void drive_op(int left, int right) {
  driveFL.move(left);
  driveML.move(left);
  driveBL.move(left);

  driveFR.move(right);
  driveMR.move(right);
  driveBR.move(right);
}


//PID used when grabbing neutral mobile goals during autonomous
//Ensures speed but deceleration while approaching the goal so as to not bump
//it out of its expected position
void speedPID(double setpoint)
{
  double kP = 35.0;
  double kI = 0.0;
  double kD = 5.0;

  float error = 6;
  float prevError;


  setpoint = Units.InToDeg(setpoint);

  pros::lcd::set_text(1, std::to_string(setpoint));

  while(std::abs(error) > 5)
  {
    error = setpoint - tracking.get_value();
    float integral = integral + error;

    if(error == 0)
      integral = 0;

    if(integral > 12000)
		{
			integral = 0;
		}

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    drive_voltage(power, power);

    pros::delay(15);
  }
  drive_voltage(0, 0);
}



class odom{
private:
  //Start of odom declarations
  double distance;              //Distance traveled by the tracking wheel
  double deltaD;                //Distance traveled this update
  double prevD = 0;             //Stores the previous position of the tracking wheel
  double radius;                //Radius of robot's movement arc
  double offset[2] {0, 0};      //Offset of a 2d vector {x, y}
  double heading = 0;           //Heading of the robot
  double deltaH = 0;            //Change in heading since the last update
  double prevH = 0;             //Stores previous heading of the robot
  double chordLength;           //Length of a 2d vector
  double pos[2] {0, 0};         //Position of the robot {x, y}

  //Start of Pure Pursuit declarations
  double spacing = 6;           //Spacing of injected points
  std::vector<std::vector<double>> curvedPath;  //Path for Pure Pursuit to follow
  double weight_smooth = 0.865;                 //
  double weight_data = 1 - weight_smooth;       //Variables for adjusting curvature of path (gradient descent)
  double tolerance = 0.001;                     //
  double maxVelocity = 280;     //Maximum velocity the robot will follow the path with
  double k = 2.5;               //Tuning constant of how fast the robot will move around curves
  double maxAcceleration = 100; //Maximum difference of target velocity between each point
  double maxRateChange = 10;    //Define the maximum amount the rate limiter will change the input per update
  double gearRatio = 0.714285714;//Gear ratio of drivetrain to scale output power
  //Tuning constants
  double kV = 1.0;              //Feedforward velocity constant
  double kA = 0.56;             //Feedforward acceleration constant
  double kP = 2.8;              //Proportional feedback constant

public:
  bool forceQuit = false;       //Kill the odom at any time
  bool forceQuit_t = false;     //Kill Pure Pursuit at any time
  double lookaheadDistance = 25;//Distance the robot looks ahead to follow

  /*
   * @brief Odometer to track the robot's absolute position
   *
   * Because three unpowered tracking wheels is not possible on the current
   * robot, use an algorithm that only uses one (parallel) tracking wheel and
   * the inertial sensor. It is assumed the robot is moving in an arc. Use sin
   * and cosine of the robot's heading to find the x and y components of the
   * chord of that arc. This is the robot's x and y translation. The algorithm
   * updates the robot's position every 20 milliseconds. The change in position
   * and heading is added to a running sum. This is the robot's total movement
   * since startup.
   *
   * @param[out] pos     Robot's absolute position
   */
  void TrackPosition() {
    //Find total distance traveled by the tracking wheel
    distance = Units.DegToIn(tracking.get_value());
    //Calculate the distance traveled since the last update
    deltaD = distance - prevD;
    //Set the previous distance to the current distance
    prevD = distance;
    //Find the change in heading
    heading = gyro.get_rotation();
    deltaH = heading - prevH;
    prevH = heading;

    //Uses a chord of the radius of the curve to determine offset
    if(deltaH < 0 && deltaH != 0)
      radius = (deltaD / deltaH);
    else if(deltaH > 0 && deltaH != 0)
      radius = (deltaD / deltaH);

      //Calculates the chord traveled
      chordLength = 2 * (radius * (deltaH / 2));

      //Calculates x and y components of chordLength
      threadProtector.take();
      offset[0] = sin(Units.DegToRad(heading - (deltaH / 2)))
                  * chordLength;
      offset[1] = cos(Units.DegToRad(heading - (deltaH / 2)))
                  * chordLength;
      threadProtector.give();


    if(isnanf(offset[0]) || isnanf(offset[1])) {
      printf("NAN in odom \n");
    }
    else {
      pos[0] += offset[0];
      pos[1] += offset[1];
      //printf("x: %G \n y: %G \n a: %G", pos[0], pos[1], heading);
      std::string output1 = "X: " + std::to_string(pos[0]);
      std::string output2 = "Y: " + std::to_string(pos[1]);
      std::string output3 = "A: " + std::to_string(heading);
      pros::lcd::set_text(1, output1);
      pros::lcd::set_text(2, output2);
      pros::lcd::set_text(3, output3);
    }
    pros::delay(20);
  }
  //Return position data from odometry
  double posx() {
    return pos[0];
  }
  double posy() {
    return pos[1];
  }
  double posa() {
    return heading;
  }


  //Start of Pure Pursuit functions (see line 272 for a full explanation)
  std::vector<std::vector<double>> InjectPoints(std::vector<std::vector<double>> waypoints) {
    //Inject points
    std::vector<std::vector<double>> new_points;

    for(int i = 0; i < waypoints.size()-1; i++) {
        vector_t vector;

        vector.x = waypoints[i + 1][0] - waypoints[i][0];
        vector.y = waypoints[i + 1][1] - waypoints[i][1];

        vector.magnitude = sqrt(pow(vector.x, 2) + pow(vector.y, 2));

        int num_points_that_fit = ceil(vector.magnitude / spacing);

        vector.x = (vector.x / vector.magnitude) * spacing;
        vector.y = (vector.y / vector.magnitude) * spacing;

        std::vector<std::vector<double>> temp;
        temp.resize(num_points_that_fit, std::vector<double>(2));


        for(int j = 0; j < num_points_that_fit; j++) {
            temp[j][0] = waypoints[i][0] + (vector.x * j);
            temp[j][1] = waypoints[i][1] + (vector.y * j);
        }

        new_points.insert(std::end(new_points), std::begin(temp), std::end(temp));
    }
    new_points.insert(std::end(new_points), waypoints[waypoints.size() - 1]);
    return new_points;
}




std::vector<std::vector<double>> Smoother(std::vector<std::vector<double>> path) {
	//copy array
	std::vector<std::vector<double>> newPath = path;

	double change = tolerance;
	while(change >= tolerance) {
		change = 0.0;
		for(int i=1; i<path.size()-1; i++) {
			for(int j=0; j<path[i].size(); j++) {
				double aux = newPath[i][j];
				newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
				change += std::abs(aux - newPath[i][j]);
			}
    }
	}
	return newPath;
}


void CalculateCurvature(std::vector<std::vector<double>>& path) {
    //Calculate path curvature
    for(int i = 1; i < path.size() - 1; i++) {
        path[i][0] += 0.001;

        double k1 = 0.5 * (pow(path[i][0], 2) + pow(path[i][1], 2) - pow(path[i - 1][0], 2) - pow(path[i - 1][1], 2)) / (path[i][0] - path[i - 1][0]);
        double k2 = (path[i][1] - path[i - 1][1]) / (path[i][0] - path[i - 1][0]);
        double b = 0.5 * (pow(path[i - 1][0], 2) - 2 * path[i - 1][0]  * k1 + pow(path[i - 1][1], 2) - pow(path[i + 1][0], 2) + 2 * path[i + 1][0] * k1 - pow(path[i + 1][1], 2)) / (path[i + 1][0] * k2 - path[i + 1][1] + path[i - 1][1] - path[i - 1][0] * k2);
        double a = k1 - k2 * b;
        double curvature = 1 / (sqrt(pow((path[i][0] - a), 2) + pow((path[i][1] - b), 2)));

        if(std::isnan(curvature))
            curvature = 0;

        path[i][2] = curvature;
    }
    path[0][2] = 0.0;
    path[path.size() - 1][2] = 0.0;
}

void CalculateVelocities(std::vector<std::vector<double>>& path) {
    for(int i = (path.size() - 2); i > 0; i--) {
        double distance = sqrt(pow((path[i][0] - path[i + 1][0]), 2) + pow((path[i][1] - path[i + 1][1]), 2));
        path[i][2] = std::min(std::min(maxVelocity, (k / path[i][2])), sqrt((pow(path[i + 1][2], 2)) + (2 * maxAcceleration * distance)));
    }
}



/*
 * @brief Convert waypoints into finished path
 *
 * Accesses the above functions (starting at line 188) to convert the raw
 * waypoints into a completed path.Four step process:
 *    1. Inject additional points between the waypoints. These must have equal
 *       spacing (6 inches) for the algorithm used in step 2.
 *    2. Use the gradient descent algorithm to smooth the waypoints into a
 *       curved path. Finds the curved position of the current
 *       point based on the positions of the previous and the next point as
 *       the algorithm iterates through all points on the path.
 *    3. Calculate the curvature of the path at each point. Curvature is
 *       defined as the reciprocal of the radius of a circle that passes through
 *       the current point, the next point, and the previous point. This likewise
 *       iterates through each point along the path.
 *    4. Compute the target velocity for each point based on the curvature of
 *       the path at that point and the maximum acceleration (defined in odom class)
 *
 * @param      waypoints       Raw waypoint data to be processed
 * The rest of the parameters are declared in scope of odom class
 * @param[in]  spacing         Spacing of points to be injected
 * @param[in]  weight_smooth, weight_data, tolerance
 *                             Tuning for how curvy the final path is
 * @param[in]  maxVelocity     Constrain robot's movement velocity
 * @param[in]  maxAcceleration Constrain robot's acceleration
 * @param[in]  k               How fast the robot moves around curves
 * @param[out] curvedPath      Completed path used in the rest of the algorithm
 *
 */
void parse_path(std::vector<std::vector<double>> waypoints) {
  std::vector<std::vector<double>> straightPath;

  straightPath = InjectPoints(waypoints);

  curvedPath = Smoother(straightPath);

  CalculateCurvature(curvedPath);

  CalculateVelocities(curvedPath);

  for(int i = 0; i < curvedPath.size(); i++) {
    std::string s = "(" + std::to_string(curvedPath[i][0]) + ", " + std::to_string(curvedPath[i][1]) + ")";
    char outputArray[s.size() + 1];
    strcpy(outputArray, s.c_str());
    printf("%s\n", outputArray);
  }

  printf("Parsed path\n");
}




  /*
   * @brief Pure pursuit path following algorithm
   *
   * Four step process of following the path:
   *    1. Find the closest point along the path to the robot's position
   *       (to access its pre-calculated target velocity)
   *    2. Find where the lookahead circle (with origin on the robot's position
   *       and radius of lookahead distance) intersects the path. This becomes
   *       the lookahead point the robot pursues.
   *    3. Calculate the curvature of the arc between the robot's current Position
   *       and the lookahead point
   *    4. Compute the desired left and right wheel velocity values based on
   *       the curvature and the target velocity of the closest point on the path.
   *       Set the drivetrain motors to this value, left and right respectively
   * All necessary parameters are declared within the scope of odom class
   *
   * @param     posx, posy        Position values, calculated in odometry
   * @param     curvedPath        Path for the algorithm to follow
   * @param     maxRateChange     Constrain acceleration (rate limiter)
   * @param     gearRatio         Account for gear ratio of drivetrain in output power
   * @param     lookaheadDistance Radius of lookahead circle
   * @param     kV, kA, kP        Tuning constants for velocity control
   *
   * TODO: split into multiple functions for ease of reading
   * TODO: fix rate limiter
   */
  void follow_path() {
    //Find closest point on the path (line 353)
    int closestPoint = 1;
    int intersectIndex = 0;
    //Find intersection (lookahead point) (line 367)
    double fractionalIndex = 0;
    vector_t lookaheadPoint;

    //Store previous velocities for derivative term
    timer clock;
    double prevL;
    double prevR;

    //Store position of robot
    vector_t position;

    //Limit rate of change of requested velocity
    rateLimiter limit;

    //Telemtry variables
    int iteratorTelem = 0;
    timer timeSinceStart;
    FILE* targetVelocity_t = fopen("/usd/telem/targetVelocity.txt", "w");
    FILE* measuredVelocity_t = fopen("/usd/telem/measuredVelocity.txt", "w");

    //while loop starts here
    while(closestPoint != curvedPath.size() - 1 && !forceQuit && !forceQuit_t) {
      //Copy position for thread protection
      threadProtector.take();
      position.x = pos[0];
      position.y = pos[1];
      position.a = 0;//Units.DegToRad(gyro.get_rotation());
      threadProtector.give();



      //================================================================================================
      //Find closest point on the path
      double lastClosestDistance = 100;
      for(int i = closestPoint; i < curvedPath.size(); i++) {
        double distance = sqrt(pow(curvedPath[i][0] - position.x, 2) + pow(curvedPath[i][1] - position.y, 2));

        if(distance < lastClosestDistance) {
          lastClosestDistance = distance;
          closestPoint = i;
        }
      }



      //================================================================================================
      //Find intersection (lookahead point)
      for(int i = intersectIndex; i < curvedPath.size() - 1; i++) {
        //E is the starting point of the vector
        //L is the end point of the vector
        //C is the center of the circle (with r lookahead distance)

        //d = L - E
        vector_t d;
        d.x = curvedPath[i + 1][0] - curvedPath[i][0];
        d.y = curvedPath[i + 1][1] - curvedPath[i][1];

        //f = E - C
        vector_t f;
        f.x = curvedPath[i][0] - position.x;
        f.y = curvedPath[i][1] - position.y;

        //a = d.DotProduct(d)
        double a = (d.x * d.x) + (d.y * d.y);
        //b = 2 * f.DotProduct(d)
        double b = 2 * ((f.x * d.x) + (f.y * d.y));
        //c = f.DotProduct(f) - r * r
        //r is the radius of the sphere, the lookahead distance
        double c = ((f.x * f.x) + (f.y * f.y)) - (lookaheadDistance * lookaheadDistance);

        //discriminant = b * b - 4 * a * c
        double discriminant = (b * b) - (4 * a * c);

        if(discriminant < 0) {
          //No intersection
        }
        else {
          discriminant = sqrt(discriminant);

          double t1 = (-b - discriminant) / (2 * a);
          double t2 = (-b + discriminant) / (2 * a);

          //T2 intersection is always further along the path
          if(t2 >= 0 && t2 <= 1) {
            //return t2 intersection
            vector_t intersection;
            intersection.x = curvedPath[i][0] + t2 * d.x;
            intersection.y = curvedPath[i][1] + t2 * d.y;

            //Find the fractional index of the intersection and compare it to the previous lookahead point
            double tempFractionalIndex = i + t2;
            if(tempFractionalIndex > fractionalIndex) {
              //If the fractional index is greater,
              //set the new values for the next loop and the rest of the algorithm
              fractionalIndex = tempFractionalIndex;
              intersectIndex = i;
              lookaheadPoint.x = intersection.x;
              lookaheadPoint.y = intersection.y;

              //Cancel the loop as the value has been found
              break;
            }
          }
          if(t1 >= 0 && t1 <= 1) {
            //return t1 intersection
            vector_t intersection;
            intersection.x = curvedPath[i][0] + t1 * d.x;
            intersection.y = curvedPath[i][1] + t1 * d.y;

            //Find the fractional index of the intersection and compare it to the previous lookahead point
            double tempFractionalIndex = i + t1;
            if(tempFractionalIndex > fractionalIndex) {
              //If the fractional index is greater,
              //set the new values for the next loop and the rest of the algorithm
              fractionalIndex = tempFractionalIndex;
              intersectIndex = i;
              lookaheadPoint.x = intersection.x;
              lookaheadPoint.y = intersection.y;

              //Cancel the loop as the value has been found
              break;
            }
          }
        }
      }



      //================================================================================================
      //Find the curvature of the movement arc
      //Transform the lookahead point to the robot's local coordinates
      vector_t localLookAhead;
      localLookAhead.x = (lookaheadPoint.x - position.x) * cos(position.a) + (lookaheadPoint.y - position.y) * sin(position.a);
      localLookAhead.y = (lookaheadPoint.x - position.x) * sin(position.a) + (lookaheadPoint.y - position.y) * cos(position.a);
      //Find the magnitude of the localLookAhead vector
      //(it should be the lookahead distance but it might change as the robot approaches the end of the path)
      localLookAhead.magnitude = sqrt(pow(localLookAhead.x, 2) + pow(localLookAhead.y, 2));
      //The x-component of the localLookAhead vector is the horizontal distance to the lookahead point
      //Use this to find the curvature of the arc the robot should move
      //double curvature = (2 * localLookAhead.x) / pow(localLookAhead.magnitude, 2);
      double curvature = (2 * localLookAhead.x) / pow(lookaheadDistance, 2);



      //================================================================================================
      //Compute wheel velocities
      //L = target left wheel's speed
      //R = target right wheel's speed
      //T = track width
      double T = 17; //inches

      //Rate limiter call
      //TODO: Fix rate limiter
      //double targetVelocity = limit.constrain(curvedPath[closestPoint][2], maxRateChange);
      double targetVelocity = curvedPath[closestPoint][2];


      //Target left wheel speed
      double L = targetVelocity * (2 + (curvature * T)) / 2;
      double R = targetVelocity * (2 - (curvature * T)) / 2;

      //Control wheel velocities
      double lFF = (kV * L) /*+ (kA * ((L - prevL) / (clock.time() / 1000)))*/; //Add kA term (potentially divide by dT for change in time if it is not constant)
      double rFF = (kV * R) /*+ (kA * ((R - prevR) / (clock.time() / 1000)))*/; //Add kA term
      prevL = L;
      prevR = R;
      clock.reset();

      double lFB = kP * (L - (Units.DPSToRPM(leftEncoder.get_velocity() / 100)));
      double rFB = kP * (R - (Units.DPSToRPM(rightEncoder.get_velocity() / 100)));

      //Set motor velocities
      //Power is set as (feedforward term + feedbackward term) * 60 to scale for move_voltage
      // * gearRatio to scale the velocity for the gear ratio of the drivetrain
      drive_voltage((lFF + lFB) * 60 * gearRatio, (rFF + rFB) * 60 * gearRatio);

      //================================================================================================
      //Telemtry for output data
      std::string output = "Closest point: " + std::to_string(closestPoint) + "Top point: " + std::to_string(curvedPath.size());
      pros::lcd::set_text(4, output);
      std::string lookAheadOutput = "Lookahead: (" + std::to_string(lookaheadPoint.x) + ", " + std::to_string(lookaheadPoint.y) + ")";
      pros::lcd::set_text(5, lookAheadOutput);
      std::string localLookAheadOutput = "local: (" + std::to_string(localLookAhead.x) + ", " + std::to_string(localLookAhead.y) + ")";
      pros::lcd::set_text(6, localLookAheadOutput);
      std::string curvatureOutput = "Curvature: " + std::to_string(curvature);
      pros::lcd::set_text(7, curvatureOutput);
      /*
      std::string velocityOutput = "Velocity: " + std::to_string(L) + ", " + std::to_string(R);
      pros::lcd::set_text(8, velocityOutput);
      */


      //Telemtry for tuning
      if(iteratorTelem % 500 == 0)
      {
        //Left target velocity
        std::string output = "(" + std::to_string(timeSinceStart.time()) + ", " + std::to_string(L) + ")";
        char outputArray[output.size() + 1];
        strcpy(outputArray, output.c_str());
    	  fprintf(targetVelocity_t, "%s\n", outputArray);

        //Left measured velocity
        std::string output2 = "(" + std::to_string(timeSinceStart.time()) + ", " + std::to_string(Units.DPSToRPM(leftEncoder.get_velocity() / 100)) + ")";
        char outputArray2[output2.size() + 1];
        strcpy(outputArray2, output2.c_str());
    	  fprintf(measuredVelocity_t, "%s\n", outputArray2);
      }

      pros::delay(25);
    }
    printf("Quit PP");
    drive_voltage(0, 0);
    //Close telemetry files to save them
    fclose(targetVelocity_t);
    fclose(measuredVelocity_t);
    curvedPath.clear();
  }
};


odom chassis;

//External odometry functions
void calc() {
  while(!chassis.forceQuit) {
    chassis.TrackPosition();
    pros::delay(10);
  }
}

void start_odom() {
  pros::Task odom_task(calc, "Odometry");
}

void quit_odom() {
  chassis.forceQuit = true;
}


//External Pure Pursuit functions
void calc_t() {
  chassis.follow_path();
}

/*
 * @brief Intializes pure pursuit algorithm
 *
 * Accesses member functions of class odom to follow a path constructed based
 * on the waypoints provided (stored as a 2d vector). Also sets lookahead
 * distance in the object chassis of the odom class.
 *
 * @param waypoints     2D vector holds waypoints to form the desired path
 * @param lookaheadDistance     Distance to lookahead point
 *
 * TODO: make a new class to more efficiently store waypoints
 */
void start_pp(std::vector<std::vector<double>> waypoints, int lookaheadDistance) {
  chassis.parse_path(waypoints);
  chassis.lookaheadDistance = lookaheadDistance;
  calc_t();
}

/*
 * @brief Same as above but asynchronous
 *
 * Be careful with tasks. Ensure they are deleted before driver control starts
 */
void start_pp_async(std::vector<std::vector<double>> waypoints, int lookaheadDistance) {
  chassis.parse_path(waypoints);
  chassis.lookaheadDistance = lookaheadDistance;
  pros::Task pp_task(calc_t, "Pure Pursuit");
}

void quit_pp() {
  chassis.forceQuit_t = true;
}
