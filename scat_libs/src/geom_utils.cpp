#include <scat_libs/geom_utils.h>

namespace geom_utils
{
#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

double determinant(double v1[2], double v2[2])
{
  return v1[0] * v2[1] - v2[0] * v1[1];
}

double determinant(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  return p1.x * p2.y - p2.x * p1.y;
}

void findInCenter(double a[2], double b[2], double c[2], double incenter[2])
{
  double AB = base_utils::euclideanDistance(a, b);
  double BC = base_utils::euclideanDistance(b, c);
  double CA = base_utils::euclideanDistance(c, a);

  // Formula to calculate in-center
  incenter[0] = (AB * a[0] + BC * b[0] + CA * c[0]) / (AB + BC + CA);
  incenter[1] = (AB * a[1] + BC * b[1] + CA * c[1]) / (AB + BC + CA);
}

void findCircumCenter(double a[2], double b[2], double c[2], double circumcenter[2])
{
  double ad = a[0] * a[0] + a[1] * a[1];
  double bd = b[0] * b[0] + b[1] * b[1];
  double cd = c[0] * c[0] + c[1] * c[1];
  double D = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
  circumcenter[0] = 1 / D * (ad * (b[1] - c[1]) + bd * (c[1] - a[1]) + cd * (a[1] - b[1]));
  circumcenter[1] = 1 / D * (ad * (c[0] - b[0]) + bd * (a[0] - c[0]) + cd * (b[0] - a[0]));
}

geometry_msgs::Point findInCenter(const std::vector<geometry_msgs::Point>& pts)
{
  geometry_msgs::Point incenter;
  double AB = base_utils::euclideanDistance(pts[0], pts[1]);
  double BC = base_utils::euclideanDistance(pts[1], pts[2]);
  double CA = base_utils::euclideanDistance(pts[2], pts[0]);

  // Formula to calculate in-center
  incenter.x = (AB * pts[0].x + BC * pts[1].x + CA * pts[2].x) / (AB + BC + CA);
  incenter.y = (AB * pts[0].y + BC * pts[1].y + CA * pts[2].y) / (AB + BC + CA);
  return incenter;
}

geometry_msgs::Point findCircumCenter(const std::vector<geometry_msgs::Point>& pts)
{
  geometry_msgs::Point circumcenter;
  double ad = pts[0].x * pts[0].x + pts[0].y * pts[0].y;
  double bd = pts[1].x * pts[1].x + pts[1].y * pts[1].y;
  double cd = pts[2].x * pts[2].x + pts[2].y * pts[2].y;
  double D = 2 * (pts[0].x * (pts[1].y - pts[2].y) + pts[1].x * (pts[2].y - pts[0].y) + pts[2].x * (pts[0].y - pts[1].y));
  circumcenter.x = 1 / D * (ad * (pts[1].y - pts[2].y) + bd * (pts[2].y - pts[0].y) + cd * (pts[0].y - pts[1].y));
  circumcenter.y = 1 / D * (ad * (pts[2].x - pts[1].x) + bd * (pts[0].x - pts[2].x) + cd * (pts[1].x - pts[0].x));
  return circumcenter;
}

geometry_msgs::Point findInCenter(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point& c)
{
  geometry_msgs::Point incenter;
  double AB = base_utils::euclideanDistance(a, b);
  double BC = base_utils::euclideanDistance(b, c);
  double CA = base_utils::euclideanDistance(c, a);

  // Formula to calculate in-center
  incenter.x = (AB * a.x + BC * b.x + CA * c.x) / (AB + BC + CA);
  incenter.y = (AB * a.y + BC * b.y + CA * c.y) / (AB + BC + CA);
  return incenter;
}

geometry_msgs::Point findCircumCenter(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point& c)
{
  geometry_msgs::Point circumcenter;
  double ad = a.x * a.x + a.y * a.y;
  double bd = b.x * b.x + b.y * b.y;
  double cd = c.x * c.x + c.y * c.y;
  double D = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
  circumcenter.x = 1 / D * (ad * (b.y - c.y) + bd * (c.y - a.y) + cd * (a.y - b.y));
  circumcenter.y = 1 / D * (ad * (c.x - b.x) + bd * (a.x - c.x) + cd * (b.x - a.x));
  return circumcenter;
}

bool doIntersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& q1, const geometry_msgs::Point& q2)
{
  return (((q1.x - p1.x) * (p2.y - p1.y) - (q1.y - p1.y) * (p2.x - p1.x)) *
              ((q2.x - p1.x) * (p2.y - p1.y) - (q2.y - p1.y) * (p2.x - p1.x)) <
          0) &&
         (((p1.x - q1.x) * (q2.y - q1.y) - (p1.y - q1.y) * (q2.x - q1.x)) *
              ((p2.x - q1.x) * (q2.y - q1.y) - (p2.y - q1.y) * (q2.x - q1.x)) <
          0);
}

bool doIntersect(double p1[2], double p2[2], double q1[2], double q2[2])
{
  return (((q1[0] - p1[0]) * (p2[1] - p1[1]) - (q1[1] - p1[1]) * (p2[0] - p1[0])) *
              ((q2[0] - p1[0]) * (p2[1] - p1[1]) - (q2[1] - p1[1]) * (p2[0] - p1[0])) <
          0) &&
         (((p1[0] - q1[0]) * (q2[1] - q1[1]) - (p1[1] - q1[1]) * (q2[0] - q1[0])) *
              ((p2[0] - q1[0]) * (q2[1] - q1[1]) - (p2[1] - q1[1]) * (q2[0] - q1[0])) <
          0);
}

bool onSegment(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& q)
{
  // Given three colinear points p1, p1, q, the function checks if
  // point q lies on line segment 'p1 p2'
  if (q.x <= std::max(p1.x, p2.x) && q.x >= std::min(p1.x, p2.x) &&
      q.y <= std::max(p1.y, p2.y) && q.y >= std::min(p1.y, p2.y))
    return true;

  return false;
}

bool onSegment(double p1[2], double p2[2], double q[2])
{
  // Given three colinear points p, q, r, the function checks if
  // point q lies on line segment 'pr'
  if (q[0] <= std::max(p1[0], p2[0]) && q[0] >= std::min(p1[0], p2[0]) &&
      q[1] <= std::max(p1[1], p2[1]) && q[1] >= std::min(p1[1], p2[1]))
    return true;

  return false;
}

int orientation(const geometry_msgs::Point& p, const geometry_msgs::Point& q, const geometry_msgs::Point& r)
{
  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  int val = (q.y - p.y) * (r.x - q.x) -
            (q.x - p.x) * (r.y - q.y);

  if (val == 0)
    return 0; // colinear

  return (val > 0) ? 1 : 2; // clock or counterclock wise
}

int orientation(double p[2], double q[2], double r[2])
{
  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  int val = (q[1] - p[1]) * (r[0] - q[0]) -
            (q[0] - p[0]) * (r[1] - q[1]);

  if (val == 0)
    return 0; // colinear

  return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// returns true when q1 coordinates are on other side of line p1-p2 compared to q2
bool isOnOtherSide(double p1[2], double p2[2], double q1[2], double q2[2])
{
  double v[2] = {p2[0] - p1[0], p2[1] - p1[1]};  // line vector p1,p2
  double va[2] = {p2[0] - q1[0], p2[1] - q1[1]}; // vector p1 > p3
  double vb[2] = {p2[0] - q2[0], p2[1] - q2[1]}; // vector p1 > node

  // returns true when node coordinates are on the other side of the line p1-p2 comq1red to p3
  if (determinant(v, va) * determinant(v, vb) > 1)
    return false;
  else
    return true;
}

// returns true when q1 coordinates are on other side of line p1-p2 compared to q2
bool isOnOtherSide(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& q1, const geometry_msgs::Point& q2)
{
  geometry_msgs::Point v = rosmsg::makePoint(p2.x - p1.x, p2.y - p1.y);  // line vector p1,p2
  geometry_msgs::Point va = rosmsg::makePoint(p2.x - q1.x, p2.y - q1.y); // line vector p1,p2
  geometry_msgs::Point vb = rosmsg::makePoint(p2.x - q2.x, p2.y - q2.y); // line vector p1,p2

  // returns true when node coordinates are on the other side of the line p1-p2 compared to p3
  if (determinant(v, va) * determinant(v, vb) > 1)
    return false;
  else
    return true;
}

bool isInsidePolygon(const std::vector<geometry_msgs::Point>& polygon, const geometry_msgs::Point32& point)
{
  ROS_DEBUG_STREAM("is inside polygon");
  int counter = 0;
  int i, j, N = polygon.size();
  float xinters;
  geometry_msgs::Point p1, p2;

  p1 = polygon[0];
  for (i = 1; i <= N; i++)
  {
    p2 = polygon[i % N];
    if (point.y > std::min(p1.y, p2.y))
    {
      if (point.y <= std::max(p1.y, p2.y))
      {
        if (point.x <= std::max(p1.x, p2.x))
        {
          if (p1.y != p2.y)
          {
            xinters = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
            if (p1.x == p2.x || point.x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }

  if (counter % 2 == 0)
    return false;
  else
    return true;
}

bool isInsidePolygon(const std::vector<geometry_msgs::Point>& polygon, const geometry_msgs::Point& point)
{
  int counter = 0;
  int i, j, N = polygon.size();
  double xinters;
  geometry_msgs::Point p1, p2;

  p1 = polygon[0];
  for (i = 1; i <= N; i++)
  {
    p2 = polygon[i % N];
    if (point.y > MIN(p1.y, p2.y))
    {
      if (point.y <= MAX(p1.y, p2.y))
      {
        if (point.x <= MAX(p1.x, p2.x))
        {
          if (p1.y != p2.y)
          {
            xinters = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
            if (p1.x == p2.x || point.x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }

  if (counter % 2 == 0)
    return false;
  else
    return true;
}

geometry_msgs::Point localToGlobal(const geometry_msgs::Point& point_local, const double& x_local_frame, const double& y_local_frame, const double& theta_local_frame)
{
  geometry_msgs::Point point_global;
  point_global.x = x_local_frame + point_local.x * cos(theta_local_frame) - point_local.y * sin(theta_local_frame);
  point_global.y = y_local_frame + point_local.x * sin(theta_local_frame) + point_local.y * cos(theta_local_frame);
  return point_global;
}

geometry_msgs::Point localToGlobal(const geometry_msgs::Point& point_local, const geometry_msgs::Pose2D& local_frame)
{
  geometry_msgs::Point point_global;
  point_global.x = local_frame.x + point_local.x * cos(local_frame.theta) - point_local.y * sin(local_frame.theta);
  point_global.y = local_frame.y + point_local.x * sin(local_frame.theta) + point_local.y * cos(local_frame.theta);
  return point_global;
}

geometry_msgs::Point32 localToGlobal(const geometry_msgs::Point32& point_local, float x_local_frame, float y_local_frame, float theta_local_frame)
{
  geometry_msgs::Point32 point_global;
  point_global.x = x_local_frame + point_local.x * cos(theta_local_frame) - point_local.y * sin(theta_local_frame);
  point_global.y = y_local_frame + point_local.x * sin(theta_local_frame) + point_local.y * cos(theta_local_frame);
  return point_global;
}

geometry_msgs::Point32 localToGlobal(const geometry_msgs::Point32& point_local, const geometry_msgs::Pose2D& local_frame)
{
  geometry_msgs::Point32 point_global;
  point_global.x = local_frame.x + point_local.x * cos(local_frame.theta) - point_local.y * sin(local_frame.theta);
  point_global.y = local_frame.y + point_local.x * sin(local_frame.theta) + point_local.y * cos(local_frame.theta);
  return point_global;
}

void localToGlobal(double &x, double &y, double &theta, double x_local_frame, double y_local_frame, double theta_local_frame)
{
  double x_global = x_local_frame + x * cos(theta_local_frame) - y * sin(theta_local_frame);
  double y_global = y_local_frame + x * sin(theta_local_frame) + y * cos(theta_local_frame);
  double theta_global = theta + theta_local_frame;
  x = x_global;
  y = y_global;
  theta = theta_global;
}

geometry_msgs::Point globalToLocal(const geometry_msgs::Point& point_global, const geometry_msgs::Pose2D& local_frame)
{
  geometry_msgs::Point point_local;
  point_local.x = (point_global.x - local_frame.x) * cos(local_frame.theta) + (point_global.y - local_frame.y) * sin(local_frame.theta);
  point_local.y = -(point_global.x - local_frame.x) * sin(local_frame.theta) + (point_global.y - local_frame.y) * cos(local_frame.theta);
  return point_local;
}

geometry_msgs::Point globalToLocal(const geometry_msgs::Point& point_global, double x_local_frame, double y_local_frame, double theta_local_frame)
{
  // x_local_frame, y_local_frame, theta_local_frame are agent / local frame coordinates in global frame
  geometry_msgs::Point point_local;
  point_local.x = (point_global.x - x_local_frame) * cos(theta_local_frame) + (point_global.y - y_local_frame) * sin(theta_local_frame);
  point_local.y = -(point_global.x - x_local_frame) * sin(theta_local_frame) + (point_global.y - y_local_frame) * cos(theta_local_frame);
  return point_local;
}

geometry_msgs::Point32 globalToLocal(const geometry_msgs::Point32& point_global, const geometry_msgs::Pose2D& local_frame)
{
  geometry_msgs::Point32 point_local;
  point_local.x = (point_global.x - local_frame.x) * cos(local_frame.theta) + (point_global.y - local_frame.y) * sin(local_frame.theta);
  point_local.y = -(point_global.x - local_frame.x) * sin(local_frame.theta) + (point_global.y - local_frame.y) * cos(local_frame.theta);
  return point_local;
}

geometry_msgs::Point32 globalToLocal(const geometry_msgs::Point32& point_global, float x_local_frame, float y_local_frame, float theta_local_frame)
{
  geometry_msgs::Point32 point_local;
  point_local.x = (point_global.x - x_local_frame) * cos(theta_local_frame) + (point_global.y - y_local_frame) * sin(theta_local_frame);
  point_local.y = -(point_global.x - x_local_frame) * sin(theta_local_frame) + (point_global.y - y_local_frame) * cos(theta_local_frame);
  return point_local;
}

void globalToLocal(double &x, double &y, double &theta, double x_local_frame, double y_local_frame, double theta_local_frame)
{
  // x_local_frame, y_local_frame, theta_local_frame are agent / frame coordinates in global frame
  // x, y, theta are global coordinates, which are being transformed to the local agent frame.
  double x_local = (x - x_local_frame) * cos(theta_local_frame) + (y - y_local_frame) * sin(theta_local_frame);
  double y_local = -(x - x_local_frame) * sin(theta_local_frame) + (y - y_local_frame) * cos(theta_local_frame);
  double theta_local = theta - theta_local_frame;
  x = x_local;
  y = y_local;
  if (theta_local > M_PI)
    theta_local -= 2 * M_PI;
  else if (theta_local < -M_PI)
    theta_local += 2 * M_PI;
  theta = theta_local;
}

double distanceToLine(const geometry_msgs::Point &point,
                      const geometry_msgs::Point &point1,
                      const geometry_msgs::Point &point2)
{
  // Calculates distance to line of infinite length
  // Rewrite point1 & point2 to line equation a*y + b*x + c = 0
  // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  double a = point2.y - point1.y;
  double b = point1.x - point2.x;
  double c = point2.x * point1.y - point2.y * point1.x;
  // using standard formula for distance point 0 to line: a*x0 + b*y0 + c / sqrt(a^2+b^2)
  double distance = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

  return distance;
}

double distanceToLineSegment(const geometry_msgs::Point &point,
                             const geometry_msgs::Point &point1,
                             const geometry_msgs::Point &point2)
{
  // Calculates distance of point to line segment given by point1-point2.
  geometry_msgs::Point point_on_line = findClosestPointOnInterval(point, point1, point2);
  return base_utils::euclideanDistance(point, point_on_line);
}

geometry_msgs::Point findClosestPointOnInterval(const geometry_msgs::Point &point,
                                                const geometry_msgs::Point &point1,
                                                const geometry_msgs::Point &point2)
{
  // This function calculates a point on the line interval, defined by point1 and point2, which is closest to the current position.

  // Line method //
  // Rewrite point1 & point2 to line equation a*y + b*x + c = 0
  // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  // double a = point2.y - point1.y;
  // double b = point1.x - point2.x;
  // double c = point2.x * point1.y - point2.y * point1.x;

  // geometry_msgs::Point point_on_line;
  // point_on_line.x = (b * (b * point.x - a * point.y) - a * c) / sqrt(pow(a, 2) + pow(b, 2));
  // point_on_line.y = (a * (a * point.y - b * point.x) - b * c) / sqrt(pow(a, 2) + pow(b, 2));

  // Vector method //
  double norm = sqrt(pow((point2.x - point1.x), 2) + pow((point2.y - point1.y), 2));
  std::valarray<double> linePoint = {point1.x, point1.y};
  std::valarray<double> lineDir = {(point2.x - point1.x) / norm, (point2.y - point1.y) / norm};
  std::valarray<double> Point = {point.x, point.y};
  std::valarray<double> V, D, result;
  V = Point - linePoint;
  D = V * lineDir;
  result = linePoint + lineDir * D;

  geometry_msgs::Point point_on_line;
  point_on_line.x = result[0];
  point_on_line.y = result[1];

  if (!onInterval(point_on_line, point1, point2))
  {
    point_on_line = findClosestPoint(point, point1, point2);
  }
  return point_on_line;
}

geometry_msgs::Point findClosestPoint(const geometry_msgs::Point &point,
                                      const geometry_msgs::Point &point1,
                                      const geometry_msgs::Point &point2)
{
  // This function calculates which of point1 and point2 is closer to the Point point
  if (base_utils::euclideanDistance(point, point1) <= base_utils::euclideanDistance(point, point2))
    return point1;
  else
    return point2;
}

bool onInterval(const geometry_msgs::Point &point,
                const geometry_msgs::Point &point1,
                const geometry_msgs::Point &point2)
{
  // This function calculates whether point is in between point1 and point2, or outside.
  // The algorithm already assumes that point is on the line drawn by point1 & point2.
  // If point1 and point2 are the same, the function returns true.
  if (
      ((point.x <= point1.x) && (point.x >= point2.x) || (point.x >= point1.x) && (point.x <= point2.x)) && // check whether x is in between x1 & x2
      ((point.y <= point1.y) && (point.y >= point2.y) || (point.y >= point1.y) && (point.y <= point2.y)))   // check whether y is in between y1 & y2
    return true;
  else
    return false;
}

geometry_msgs::Point calcClosestPointOnLine(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Pose& pose1,
                                            const geometry_msgs::Pose& pose2)
{
  // This function calculates a point on the line defined by pose1 and pose2, which is closest to the current pose (robot position).
  geometry_msgs::Pose line; // Denotes the line between pose1 & pose2
  line.position = pose1.position;
  line.orientation = rotateQuat(pose1.orientation, M_PI / 2);

  geometry_msgs::Pose pose_aligned; // Denotes the robot position if oriented perpendicular towards finish line
  pose_aligned.position = pose.position;
  pose_aligned.orientation = pose1.orientation;

  // Here the intersection between the robot pose and the finish line is calculated
  // NOTE: The return point may be on the outside of the finish line. This can later be checked by the onInterval function.
  geometry_msgs::Point intersection;
  intersection = calcIntersection(pose_aligned, line);

  return intersection;
}

geometry_msgs::Point calcIntersection(const geometry_msgs::Pose& pose1,
                                      const geometry_msgs::Pose& pose2)
{
  // Convert to line description:
  tf::Vector3 dir1 = getDirVector(pose1.orientation);
  tf::Vector3 dir2 = getDirVector(pose2.orientation);
  double dx1 = dir1.getX();
  double dy1 = dir1.getY();
  double dx2 = dir2.getX();
  double dy2 = dir2.getY();
  geometry_msgs::Point pos1 = pose1.position;
  geometry_msgs::Point pos2 = pose2.position;

  double s = (dx1 * (pos2.y - pos1.y) - dy1 * (pos2.x - pos1.x)) /
             (dy1 * dx2 - dy2 * dx1);
  double t = (pos2.x - pos1.x) / dx1 + dx2 / dx1 * s;

  double x1 = pos1.x + dx1 * t;
  double y1 = pos1.y + dy1 * t;
  double x2 = pos2.x + dx2 * s;
  double y2 = pos2.y + dy2 * s;

  if (!((x1 == x2) && (y1 == y2)))
  {
    ROS_WARN("Error in calcIntersection. Values don't agree.");
  }

  geometry_msgs::Point intersection;
  intersection.x = x1;
  intersection.y = y1;
  intersection.z = 0;
  return intersection;
}

double angleDisparity(const geometry_msgs::Quaternion& quat1,
                      const geometry_msgs::Quaternion& quat2)
{
  // Calculate difference in angular orientation between two quaternions in 3D space
  tf::Vector3 direction1 = getDirVector(quat1);
  tf::Vector3 direction2 = getDirVector(quat2);
  double angle_disparity = direction1.getX() * direction2.getX() +
                           direction1.getY() * direction2.getY() +
                           direction1.getZ() * direction2.getZ();
  angle_disparity = angle_disparity / (direction1.length() * direction2.length());

  if ((angle_disparity > 1.0) || (angle_disparity < -1.0))
    ROS_WARN("Value of angle_disparity exceeds 1,-1 limits in angleDisparity algorithm.");

  angle_disparity = acos(std::min(std::max(angle_disparity, -1.0), 1.0));
  return angle_disparity;
}

double angleDisparity2(const geometry_msgs::Quaternion& quat1,
                       const geometry_msgs::Quaternion& quat2)
{
  // Calculates directional (signed) difference in angular orientation between two quaternions in 2D.
  // Sign is positive clockwise, as seen from quat1.
  double theta1 = tf::getYaw(quat1);
  double theta2 = tf::getYaw(quat2);
  double angle_disparity = theta2 - theta1;

  if (fabs(angle_disparity) > M_PI)
    angle_disparity -= base_utils::sign(angle_disparity) * 2 * M_PI;

  // Alternative method
  // tf2::Quaternion tfquat1, tfquat2;
  // tf2::convert(quat1, tfquat1);
  // tf2::convert(quat2, tfquat2);
  // double angle_disparity = 2*tfquat2.angle(tfquat1);

  return angle_disparity;
}

tf::Vector3 getDirVector(const geometry_msgs::Quaternion& quat)
{
  tf::Quaternion tfquat;
  tf::quaternionMsgToTF(quat, tfquat);
  tf::Matrix3x3 Bot_RotationMatrix(tfquat);
  tf::Vector3 unitvec(1, 0, 0);
  return Bot_RotationMatrix * unitvec;
}

geometry_msgs::Quaternion rotateQuat(const geometry_msgs::Quaternion& quat, const double& angleZ)
{
  tf2::Quaternion tfquat, tfquat_rot, tfquat_new;
  geometry_msgs::Quaternion quat_new;
  tf2::convert(quat, tfquat);
  tfquat_rot.setRPY(0, 0, angleZ); // Define 90 degrees rotation, to get the line which is perpendicular to goal orientation
  tfquat_new = tfquat_rot * tfquat;
  tfquat_new.normalize();
  tf2::convert(tfquat_new, quat_new);
  return quat_new;
}

geometry_msgs::Point transformPoint(const geometry_msgs::Point& point, double x, double y, double theta) {
  geometry_msgs::Point p;
  p.x = point.x * cos(theta) - point.y * sin(theta) + x;
  p.y = point.x * sin(theta) + point.y * cos(theta) + y;
  return p;
}

geometry_msgs::Point32 transformPoint(const geometry_msgs::Point32& point, double x, double y, double theta) {
  geometry_msgs::Point32 p;
  p.x = point.x * cos(theta) - point.y * sin(theta) + x;
  p.y = point.x * sin(theta) + point.y * cos(theta) + y;
  return p;
}

} // namespace geom_utils
