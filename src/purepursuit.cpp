#include "define.h"

std::vector<std::vector<double>> waypoints
{
   {0.0, 0.0},
   {90.0, 0.0},
   {130.0, -50.0}
};


std::vector<std::vector<double>> straightPath;

double weight_smooth = 0.865;
double weight_data = 1 - weight_smooth;
double tolerance = 0.001;



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
    }
}

void Move(std::vector<std::vector<double>>& path, sPos& position, double r)
{
  double fractionalIndex = 0;
  vector lookaheadPoint;

  for(int i = 0; i < path.size() - 1; i++)
  {
    //Find intersection (lookahead point)

    //E is the starting point of the vector
    //L is the end point of the vector
    //C is the center of the circle (with r lookahead distance)

    //d = l - E
    vector d;
    d.x = path[i + 1][0] - path[i][0];
    d.y = path[i + 1][1] - path[i][1];

    //f = E - C
    vector f;
    f.x = path[i][0] - position.x;
    f.y = path[i][1] - position.y;

    //a = d.DotProduct(d)
    double a = (d.x * d.x) + (d.y * d.y);
    //b = 2 * f.DotProduct(d)
    double b = 2 * ((f.x * d.x) + (f.y * d.y));
    //c = f.DotProduct(f) - r * r
    //r is the radius of the sphere, the lookahead distance
    double c = ((f.x * f.x) + (f.y * f.y)) - (r * r);

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
        intersection.x = path[i][0] + t1 * d.x;
        intersection.y = path[i][1] + t1 * d.y;

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
        intersection.x = path[i][0] + t2 * d.x;
        intersection.y = path[i][1] + t2 * d.y;

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
}

int main()
{
    straightPath = InjectPoints(waypoints, 6.0);

    std::vector<std::vector<double>> curvedPath = Smoother(straightPath, weight_data, weight_smooth, tolerance);

    CalculateDistanceAndCurvature(curvedPath);

    for (int i = 0; i < curvedPath.size(); i++)
    {
        std::cout << "(" << curvedPath[i][0] << ", " << curvedPath[i][1] << "), " << curvedPath[i][2] << ", " << curvedPath[i][3] << "\n";
    }
    return 0;
}
