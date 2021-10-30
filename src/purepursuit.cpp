#include "odometry.h"
#include "functions.h"

std::vector<std::vector<double>> curvedPath;
double lookaheadDistance = 100;


std::vector<std::vector<double>> InjectPoints(std::vector<std::vector<double>> waypoints, double spacing)
{
    std::vector<std::vector<double>> new_points;

    for(int i = 0; i < waypoints.size()-1; i++)
    {
        vector vector;

        vector.x = waypoints[i + 1][0] - waypoints[i][0];
        vector.y = waypoints[i + 1][1] - waypoints[i][1];

        vector.magnitude = sqrt(pow(vector.x, 2) + pow(vector.y, 2));

        int num_points_that_fit = ceil(vector.magnitude / spacing);

        vector.x = (vector.x / vector.magnitude) * spacing;
        vector.y = (vector.y / vector.magnitude) * spacing;

        std::vector<std::vector<double>> temp;
        temp.resize(num_points_that_fit, std::vector<double>(2));


        for(int j = 0; j < num_points_that_fit; j++)
        {
            temp[j][0] = waypoints[i][0] + (vector.x * j);
            temp[j][1] = waypoints[i][1] + (vector.y * j);
        }

        new_points.insert(std::end(new_points), std::begin(temp), std::end(temp));
    }
    new_points.insert(std::end(new_points), waypoints[waypoints.size() - 1]);
    return new_points;
}




std::vector<std::vector<double>> Smoother(std::vector<std::vector<double>> path, double weight_data, double weight_smooth, double tolerance)
{
	//copy array
	std::vector<std::vector<double>> newPath = path;

	double change = tolerance;
	while(change >= tolerance)
	{
		change = 0.0;
		for(int i=1; i<path.size()-1; i++)
			for(int j=0; j<path[i].size(); j++)
			{
				double aux = newPath[i][j];
				newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
				change += std::abs(aux - newPath[i][j]);
			}
	}
	return newPath;
}


void CalculateDistanceAndCurvature(std::vector<std::vector<double>>& path)
{
    //Calculate distances between points
    path[0][2] = 0.0;

    for(int i = 1; i < path.size(); i++)
    {
       path[i][2] = path[i - 1][2] +  sqrt(pow((path[i][0] - path[i - 1][0]), 2) + pow((path[i][1] - path[i - 1][1]), 2));
    }


    //Calculate path curvature
    for(int i = 1; i < path.size() - 1; i++)
    {
        path[i][0] += 0.001;

        double k1 = 0.5 * (pow(path[i][0], 2) + pow(path[i][1], 2) - pow(path[i - 1][0], 2) - pow(path[i - 1][1], 2)) / (path[i][0] - path[i - 1][0]);
        double k2 = (path[i][1] - path[i - 1][1]) / (path[i][0] - path[i - 1][0]);
        double b = 0.5 * (pow(path[i - 1][0], 2) - 2 * path[i - 1][0]  * k1 + pow(path[i - 1][1], 2) - pow(path[i + 1][0], 2) + 2 * path[i + 1][0] * k1 - pow(path[i + 1][1], 2)) / (path[i + 1][0] * k2 - path[i + 1][1] + path[i - 1][1] - path[i - 1][0] * k2);
        double a = k1 - k2 * b;
        double curvature = 1 / (sqrt(pow((path[i][0] - a), 2) + pow((path[i][1] - b), 2)));

        if(std::isnan(curvature))
        {
            curvature = 0;
        }
        path[i][3] = curvature;
    }
    path[0][3] = 0.0;
    path[path.size() - 1][3] = 0.0;
}

void CalculateVelocities(std::vector<std::vector<double>>& path, double maxVelocity, double k, double maxAcceleration)
{
    path[path.size() - 1][3] = 0;
    for(int i = (path.size() - 1); i >= 0; i--)
    {
        if(path[i][3] == 0)
        {
            path[i][3] = 0.0;
            continue;
        }

        double distance = sqrt(pow((path[i][0] - path[i + 1][0]), 2) + pow((path[i][1] - path[i + 1][1]), 2));
        path[i][3] = std::min(std::min(maxVelocity, (k / path[i][3])), sqrt((pow(path[i + 1][3], 2)) + (2 * maxAcceleration * distance)));

        curvedPath[0][3] += 1;

        //std::string velocity = std::to_string(curvedPath[i][0]) + ", " + std::to_string(curvedPath[i][1]);
        //std::string velocity = "velocity" + std::to_string(curvedPath[i][3]);
        //pros::lcd::set_text(4, velocity);
        //pros::delay(500);
    }
}

void Move(void* test)
{
  //Find closest point on the path (add line number here)
  int closestPoint = 1;
  //Find intersection (lookahead point) (add line number here)
  double fractionalIndex = 0;
  vector lookaheadPoint;
  //Rate limiter implimentation
  int lastCall = 0;
  double targetVelocity = 0.0;
  //Tuning constants
  double kV = 1;
  double kA = 0.0;
  double kP = 0.0;
  //Target velocity storage for derivative
  double prevTargetVelocityL = 0.0;
  double prevTargetVelocityR = 0.0;
  int prevVelocityTime;

  int loopStart = pros::millis();

  //Find closest point on the path
  double lastClosestDistance = 100;

  //File open (telemetry)
  int iteratorTelem = 0;

  //while loop starts here
  while(position.y < 120)
  {
    //closestPoint = 0.0;
    for(int i = closestPoint; i < curvedPath.size() - 1; i++)
    {
      double distance = sqrt(pow(curvedPath[i][0] - position.x, 2) + pow(curvedPath[i][1] - position.y, 2));

      if(distance < lastClosestDistance)
      {
        lastClosestDistance = distance;
        closestPoint = i;
      }
    }


    //Find intersection (lookahead point)
    for(int i = 0; i < curvedPath.size() - 1; i++)
    {
      //E is the starting point of the vector
      //L is the end point of the vector
      //C is the center of the circle (with r lookahead distance)

      //d = L - E
      vector d;
      d.x = curvedPath[i + 1][0] - curvedPath[i][0];
      d.y = curvedPath[i + 1][1] - curvedPath[i][1];

      //f = E - C
      vector f;
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

      if(discriminant < 0)
      {
        //No intersection
      }
      else
      {
        discriminant = sqrt(discriminant);

        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        if(t1 >= 0 && t1 <= 1)
        {
          //return t1 intersection
          vector intersection;
          intersection.x = curvedPath[i][0] + t1 * d.x;
          intersection.y = curvedPath[i][1] + t1 * d.y;

          //Find the fractional index of the intersection and compare it to the previous lookahead point
          double tempFractionalIndex = i + t1;
          if(tempFractionalIndex > fractionalIndex)
          {
            //If the fractional index is greater,
            //set the new values for the next loop and the rest of the algorithm
            fractionalIndex = tempFractionalIndex;
            lookaheadPoint.x = intersection.x;
            lookaheadPoint.y = intersection.y;

            //Cancel the loop as the value has been found
            break;
          }
        }
        if(t2 >= 0 && t2 <= 1)
        {
          //return t2 intersection
          vector intersection;
          intersection.x = curvedPath[i][0] + t2 * d.x;
          intersection.y = curvedPath[i][1] + t2 * d.y;

          //Find the fractional index of the intersection and compare it to the previous lookahead point
          double tempFractionalIndex = i + t2;
          if(tempFractionalIndex > fractionalIndex)
          {
            //If the fractional index is greater,
            //set the new values for the next loop and the rest of the algorithm
            fractionalIndex = tempFractionalIndex;
            lookaheadPoint.x = intersection.x;
            lookaheadPoint.y = intersection.y;

            //Cancel the loop as the value has been found
            break;
          }
        }
      }
    }

    //Find the curvature of the movement arc
    double a = -tan(position.a);
    double b = 1;
    double c = tan(position.a) * position.x - position.y;
    double x = std::abs(a * lookaheadPoint.x + b * lookaheadPoint.y + c) / sqrt(pow(a, 2) + pow(b, 2));
    double curvature = (2 * x) / (pow(lookaheadDistance, 2));
    //Find the side of the robot the lookahead point is on
    int side = sgn(sin(position.a) * (lookaheadPoint.x - position.x) - cos(position.a) * (lookaheadPoint.y - position.a));
    curvature *= side;



    //Compute wheel velocities
    //L = target left wheel's speed
    //R = target right wheel's speed
    //T = track width
    double T = 1; //inches

    //Rate limiter implimentation
    double maxRateChange = 5.0;
    pros::lcd::set_text(7, std::to_string(closestPoint));
    targetVelocity = RateLimiter(curvedPath[closestPoint][3], lastCall, maxRateChange, targetVelocity);
    lastCall = pros::millis() - loopStart;

    //targetVelocity = curvedPath[closestPoint][3];
    //targetVelocity *= 100;

    //Target left wheel speed
    double L = targetVelocity * (2 + (curvature * T)) / 2;
    double R = targetVelocity * (2 - (curvature * T)) / 2;

    //Control wheel velocities

    double lFF = (kV * L) /*+ (kA * ((pros::millis() - loopStart - prevVelocityTime) / (prevTargetVelocityL - L)))*/;
    double rFF = (kV * R) /*+ (kA * ((pros::millis() - loopStart - prevVelocityTime) / (prevTargetVelocityR - R)))*/;
    prevVelocityTime = pros::millis();
    prevTargetVelocityL = L;
    prevTargetVelocityR = R;

    double lFB = kP * (targetVelocity - leftEncoder.get_velocity());
    double rFB = kP * (targetVelocity - rightEncoder.get_velocity());

    //Set motor velocities
    driveFL.move_velocity(lFF + lFB);
    driveBL.move_velocity(lFF + lFB);

    driveFR.move_velocity(rFF + rFB);
    driveBR.move_velocity(rFF + rFB);

    //Telemetry
    pros::lcd::set_text(5, std::to_string(leftEncoder.get_velocity()));
    pros::lcd::set_text(6, std::to_string(lFF));

    if(iteratorTelem % 25 == 0)
    {
      //Left target velocity
      std::string output = "(" + std::to_string(iteratorTelem) + ", " + std::to_string(lFF) + ")\n";
      char outputArray[output.size() + 1];
      strcpy(outputArray, output.c_str());
  	  fprintf(targetVelocityL, outputArray);

      //Right target velocity
      std::string output1 = "(" + std::to_string(iteratorTelem) + ", " + std::to_string(rFF) + ")\n";
      char outputArray1[output1.size() + 1];
      strcpy(outputArray1, output1.c_str());
  	  fprintf(targetVelocityR, outputArray1);

      //Left measured velocity
      std::string output2 = "(" + std::to_string(iteratorTelem) + ", " + std::to_string(leftEncoder.get_velocity() * DPStoRPM) + ")\n";
      char outputArray2[output2.size() + 1];
      strcpy(outputArray2, output2.c_str());
  	  fprintf(measuredVelocityL, outputArray2);

      //Right measured velocity
      std::string output3 = "(" + std::to_string(iteratorTelem) + ", " + std::to_string(rightEncoder.get_velocity() * DPStoRPM) + ")\n";
      char outputArray3[output3.size() + 1];
      strcpy(outputArray3, output3.c_str());
  	  fprintf(measuredVelocityR, outputArray3);
    }

    //fprintf(targetVelocityTelem, "test\n");
    iteratorTelem++;

    pros::delay(25);
  }
}


void PurePursuitInit()
{
  //Start odometry task
  purePursuitTask = pros::c::task_create(trackPosition, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");

  std::vector<std::vector<double>> waypoints
  {
    {0.0, 0.0},
    {0.0, 120}
  };


  std::vector<std::vector<double>> straightPath;

  double weight_smooth = 0.865;
  double weight_data = 1 - weight_smooth;
  double tolerance = 0.001;

  straightPath = InjectPoints(waypoints, 6.0);

  curvedPath = Smoother(straightPath, weight_data, weight_smooth, tolerance);

  CalculateDistanceAndCurvature(curvedPath);

  CalculateVelocities(curvedPath, 200, 2.5, 20.0);

  //Start pure pursuit task
  purePursuitTask = pros::c::task_create(Move, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Pure Pursuit");
}
