#include "functions.hpp"
#include "units.hpp"

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

//Mutex to prevent threads from interfering
pros::Mutex threadProtector_t;

//Declare new type "vector_t" to effeciently hold vector data
struct vector_t{
  float x;
  float y;
  float a;
  float magnitude;
};

//Generalized template function to get the sign of a number
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


//Start of odom declarations
float distance;              //Distance traveled by the tracking wheel
float deltaD;                //Distance traveled this update
float prevD = 0;             //Stores the previous position of the tracking wheel
float radius;                //Radius of robot's movement arc
float offset[2] {0, 0};      //Offset of a 2d vector {x, y}
float heading = 0;           //Heading of the robot
float deltaH = 0;            //Change in heading since the last update
float prevH = 0;             //Stores previous heading of the robot
float chordLength;           //Length of a 2d vector
float pos[2] {0, 0};         //Position of the robot {x, y}

bool forceQuit;


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
  while(!forceQuit) {
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

    //Uses the length of arc moved and change in heading to find radius of arc
    radius = (deltaD / deltaH);

    //Calculates the chord traveled
    chordLength = 2 * (radius * (deltaH / 2));

    //Calculates x and y components of chordLength
    offset[0] = sin(Units.DegToRad(heading - (deltaH / 2)))
                * chordLength;
    offset[1] = cos(Units.DegToRad(heading - (deltaH / 2)))
                * chordLength;


    if(isnanf(offset[0]) || isnanf(offset[1])) {
      printf("NAN in odom \n");
    }
    else {
      threadProtector_t.take();
      pos[0] += offset[0];
      pos[1] += offset[1];
      threadProtector_t.give();
      //printf("x: %G \n y: %G \n a: %G", pos[0], pos[1], heading);
      std::string output1 = "X: " + std::to_string(pos[0]);
      std::string output2 = "Y: " + std::to_string(pos[1]);
      std::string output3 = "A: " + std::to_string(heading);
      pros::lcd::set_text(1, output1);
      pros::lcd::set_text(2, output2);
      pros::lcd::set_text(3, output3);
    }
    pros::delay(10);
  }
}

void start_odom() {
  pros::Task odom_task(TrackPosition, "Odometry");
}

class pather {
public:
    std::vector<std::vector<float>> w;
    std::vector<std::vector<float>> curvedPath;

    float spacing = 6;           //Spacing of injected points
    float weight_smooth = 0.865;                 //
    float weight_data = 1 - weight_smooth;       //Variables for adjusting curvature
    float tolerance = 0.001;                        //of path (gradient descent)
    float maxVelocity = 280;     //Maximum velocity the robot will follow the path with
    float k = 3.0;               //Tuning constant of how fast the robot will move
                                   //around curves
    float maxAcceleration = 150; //Maximum difference of target velocity
                                   //between each point
    float maxRateChange = 10;    //Define the maximum amount the rate limiter will
                                   //change the input per update
    float gearRatio = 0.714285714;//Gear ratio of drivetrain to scale output power
    //Tuning constants
    const float kV = 1.0;              //Feedforward velocity constant
    const float kA = 0.0;//0.56;             //Feedforward acceleration constant
    const float kP = 2.8;              //Proportional feedback constant

    float lookaheadDistance = 25;//Distance the robot looks ahead to follow
    bool reversed = false;

    int time = 100000;


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
 * The parameters are declared in class scope
 * @param[in]  spacing         Spacing of points to be injected
 * @param[in]  weight_smooth, weight_data, tolerance
 *                             Tuning for how curvy the final path is
 * @param[in]  maxVelocity     Constrain robot's movement velocity
 * @param[in]  maxAcceleration Constrain robot's acceleration
 * @param[in]  k               How fast the robot moves around curves
 * @param[out] curvedPath      Completed path used in the rest of the algorithm
 *
 */
void parse_path() {
  //Inject points
  std::vector<std::vector<float>> path;
  
  for(int i = 0; i < w.size()-1; i++) {
        vector_t vector;

        vector.x = w[i + 1][0] - w[i][0];
        vector.y = w[i + 1][1] - w[i][1];

        vector.magnitude = sqrt(pow(vector.x, 2) + pow(vector.y, 2));

        int num_points_that_fit = ceil(vector.magnitude / spacing);

        vector.x = (vector.x / vector.magnitude) * spacing;
        vector.y = (vector.y / vector.magnitude) * spacing;

        std::vector<std::vector<float>> temp;
        temp.resize(num_points_that_fit, std::vector<float>(2));


        for(int j = 0; j < num_points_that_fit; j++) {
            temp[j][0] = w[i][0] + (vector.x * j);
            temp[j][1] = w[i][1] + (vector.y * j);
        }

        path.insert(std::end(path), std::begin(temp), std::end(temp));
    }
    path.insert(std::end(path), w[w.size() - 1]);





  //===========================================================================
  //Smooth path
  curvedPath = path;

  float change = tolerance;
	while(change >= tolerance) {
		change = 0.0;
		for(int i=1; i<path.size()-1; i++) {
			for(int j=0; j<path[i].size(); j++) {
				float aux = curvedPath[i][j];
				curvedPath[i][j] += weight_data * (path[i][j] - curvedPath[i][j]) + weight_smooth * (curvedPath[i-1][j] + curvedPath[i+1][j] - (2.0 * curvedPath[i][j]));
				change += std::abs(aux - curvedPath[i][j]);
			}
    }
	}

  //Delete path and w to save memory
  std::vector<std::vector<float>>().swap(path);
  std::vector<std::vector<float>>().swap(w);



  //===========================================================================
  //Calculate curvature
  for(int i = 1; i < curvedPath.size() - 1; i++) {
        curvedPath[i][0] += 0.001;

        float k1 = 0.5 * (pow(curvedPath[i][0], 2) + pow(curvedPath[i][1], 2) - pow(curvedPath[i - 1][0], 2) - pow(curvedPath[i - 1][1], 2)) / (curvedPath[i][0] - curvedPath[i - 1][0]);
        float k2 = (curvedPath[i][1] - curvedPath[i - 1][1]) / (curvedPath[i][0] - curvedPath[i - 1][0]);
        float b = 0.5 * (pow(curvedPath[i - 1][0], 2) - 2 * curvedPath[i - 1][0]  * k1 + pow(curvedPath[i - 1][1], 2) - pow(curvedPath[i + 1][0], 2) + 2 * curvedPath[i + 1][0] * k1 - pow(curvedPath[i + 1][1], 2)) / (curvedPath[i + 1][0] * k2 - curvedPath[i + 1][1] + curvedPath[i - 1][1] - curvedPath[i - 1][0] * k2);
        float a = k1 - k2 * b;
        float curvature = 1 / (sqrt(pow((curvedPath[i][0] - a), 2) + pow((curvedPath[i][1] - b), 2)));

        if(std::isnan(curvature))
            curvature = 0;

        curvedPath[i][2] = curvature;
    }
    curvedPath[0][2] = 0.0;
    curvedPath[curvedPath.size() - 1][2] = 0.0;





  //===========================================================================
  //Calculate target velocities
  for(int i = (curvedPath.size() - 2); i > 0; i--) {
        float distance = sqrt(pow((curvedPath[i][0] - curvedPath[i + 1][0]), 2) + pow((curvedPath[i][1] - curvedPath[i + 1][1]), 2));
        curvedPath[i][2] = std::min(std::min(maxVelocity, (k / curvedPath[i][2])), (float)sqrt((pow(curvedPath[i + 1][2], 2)) + (2 * maxAcceleration * distance)));
    }
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
   * @param     w        Path for the algorithm to follow
   * @param     maxRateChange     Constrain acceleration (rate limiter)
   * @param     gearRatio         Account for gear ratio of drivetrain in output power
   * @param     lookaheadDistance Radius of lookahead circle
   * @param     kV, kA, kP        Tuning constants for velocity control
   *
   * TODO: split into multiple functions for ease of reading
   * TODO: fix rate limiter
   */
  void follow_path() {
    //Find closest point on the path (line 308)
    int closestPoint = 1;
    int intersectIndex = 0;
    //Find intersection (lookahead point) (line 322)
    float fractionalIndex = 0;
    vector_t lookaheadPoint;

    //Store previous velocities for derivative term
    timer clock;
    float prevL;
    float prevR;

    //Store position of robot
    vector_t position;

    //Limit rate of change of requested velocity
    rateLimiter limit;

    //Telemtry variables
    int iteratorTelem = 0;
    timer timeSinceStart;

    //while loop starts here
    while(closestPoint != curvedPath.size() - 1 && timeSinceStart.time() < time) {
      //Copy position for thread protection
      threadProtector_t.take();
      position.x = pos[0];
      position.y = pos[1];
      position.a = Units.DegToRad(gyro.get_rotation());
      threadProtector_t.give();

      //If reverse pathing is desired,
      //make the robot appear reversed so curvature computes correctly
      if(reversed) {
        position.a += PI;
      }



      //=================================================================================
      //Find closest point on the path
      float lastClosestDistance = 100;
      for(int i = closestPoint; i < curvedPath.size(); i++) {
        float distance = sqrt(pow(curvedPath[i][0] - position.x, 2) + pow(curvedPath[i][1] - position.y, 2));

        if(distance < lastClosestDistance) {
          lastClosestDistance = distance;
          closestPoint = i;
        }
      }


      
      //=================================================================================
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
        float a = (d.x * d.x) + (d.y * d.y);
        //b = 2 * f.DotProduct(d)
        float b = 2 * ((f.x * d.x) + (f.y * d.y));
        //c = f.DotProduct(f) - r * r
        //r is the radius of the sphere, the lookahead distance
        float c = ((f.x * f.x) + (f.y * f.y)) - (lookaheadDistance * lookaheadDistance);

        //discriminant = b * b - 4 * a * c
        float discriminant = (b * b) - (4 * a * c);

        if(discriminant < 0) {
          //No intersection
        }
        else {
          discriminant = sqrt(discriminant);

          float t1 = (-b - discriminant) / (2 * a);
          float t2 = (-b + discriminant) / (2 * a);

          //T2 intersection is always further along the path
          if(t2 >= 0 && t2 <= 1) {
            //return t2 intersection
            vector_t intersection;
            intersection.x = curvedPath[i][0] + t2 * d.x;
            intersection.y = curvedPath[i][1] + t2 * d.y;

            //Find the fractional index of the intersection and compare it to the previous lookahead point
            float tempFractionalIndex = i + t2;
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
            float tempFractionalIndex = i + t1;
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



      //=================================================================================
      //Calculate curvature of movement arc
      vector_t headingVector;
      headingVector.x = sin(position.a);
      headingVector.y = cos(position.a);

      vector_t lookaheadVector;
      lookaheadVector.x = lookaheadPoint.x - position.x;
      lookaheadVector.y = lookaheadPoint.y - position.y;
      lookaheadVector.magnitude = sqrt(pow(lookaheadVector.x, 2) + pow(lookaheadVector.y, 2));

      float targetHeading = acos((headingVector.x * lookaheadVector.x
                             + headingVector.y * lookaheadVector.y)
                             / lookaheadVector.magnitude);

      //Find whether the turn is to the left or right (negative or positive)
      int side = sgn(((-1 * headingVector.y) * lookaheadVector.x) + (headingVector.x * lookaheadVector.y)) * -1;

      float x = sin(targetHeading) * lookaheadVector.magnitude;

      float curvature = side * (2 * x) / pow(lookaheadDistance, 2);




      //=================================================================================
      //Compute wheel velocities
      //L = target left wheel's speed
      //R = target right wheel's speed
      //T = track width
      float T = 17; //inches

      //Rate limiter call
      //TODO: Fix rate limiter
      float targetVelocity = limit.constrain(curvedPath[closestPoint][2], maxRateChange);
      //float targetVelocity = curvedPath[closestPoint][2];


      //Target left wheel speed
      float L = targetVelocity * (2 + (curvature * T)) / 2;
      float R = targetVelocity * (2 - (curvature * T)) / 2;

      if(reversed) {
        float tmp = L;

        L = -1 * R;
        R = -1 * tmp;
      }

      //Control wheel velocities
      float lFF = (kV * L) + (kA * ((L - prevL) / (clock.time()))); 
      float rFF = (kV * R) + (kA * ((R - prevR) / (clock.time()))); 
      prevL = L;
      prevR = R;
      clock.reset();

      float lFB = kP * (L - (Units.DPSToRPM(leftEncoder.get_velocity() / 100)));
      float rFB = kP * (R - (Units.DPSToRPM(rightEncoder.get_velocity() / 100)));

      //Set motor velocities
      //Power is set as (feedforward term + feedbackward term) * 60 to scale for move_voltage
      // * gearRatio to scale the velocity for the gear ratio of the drivetrain
      drive_voltage((lFF + lFB) * 60 * gearRatio, (rFF + rFB) * 60 * gearRatio);


      //=================================================================================
      //Telemtry for output data
      std::string output = "Closest point: " + std::to_string(closestPoint) + "Top point: " + std::to_string(curvedPath.size());
      pros::lcd::set_text(4, output);
      std::string lookAheadOutput = "Lookahead: (" + std::to_string(lookaheadPoint.x) + ", " + std::to_string(lookaheadPoint.y) + ")";
      pros::lcd::set_text(5, lookAheadOutput);
      std::string targetHeadingOut = "Target Heading: " + std::to_string(targetHeading);
      pros::lcd::set_text(7, targetHeadingOut);
      std::string curvatureOutput = "Curvature: " + std::to_string(curvature);
      pros::lcd::set_text(7, curvatureOutput);

      pros::delay(25);
    }
    printf("Quit PP \n");
    drive_voltage(0, 0);
  }



    //Parse the path and follow it when an object of this class is instantiated
    pather(std::vector<std::vector<float>> w_t, int lookAhead, bool reversed_t) {
        w = w_t;
        lookaheadDistance = lookAhead;
        reversed = reversed_t;

        parse_path();

        follow_path();
    }

    pather(std::vector<std::vector<float>> w_t, int lookAhead, int maxAccel, bool reversed_t, int time_t) {
        w = w_t;
        lookaheadDistance = lookAhead;
        reversed = reversed_t;
        maxAcceleration = maxAccel;
        time = time_t;

        parse_path();

        
        follow_path();
    }    
};


/*
 * @brief Turn to face point
 *
 * Uses the cross product of two vectors to find the target angle between the
 * robot's current heading and the target heading
 *
 * @param      x, y     Coords of point to face
 */
void turnTo(float x, float y, bool reversed) {
  //Find the target delta heading
  float headingRad = Units.DegToRad(gyro.get_rotation());
  vector_t headingVector;
  headingVector.x = sin(headingRad);
  headingVector.y = cos(headingRad);

  vector_t targetVector;
  targetVector.x = x - pos[0];
  targetVector.y = y - pos[1];
  targetVector.magnitude = sqrt(pow(targetVector.x, 2) + pow(targetVector.y, 2));

  float targetHeading = Units.RadToDeg(acos((headingVector.x * targetVector.x
                          + headingVector.y * targetVector.y)
                          / targetVector.magnitude));

  //Find whether the turn is to the left or right (negative or positive)
  int side = sgn(((-1 * headingVector.y) * targetVector.x) + (headingVector.x * targetVector.y)) * -1;

  targetHeading = gyro.get_rotation() + (targetHeading * side);


  if(reversed) {
    targetHeading += 180;
  }

  turnPID(targetHeading, 0.25, 0.001, 1000);
}

#endif //ODOMETRY_HPP