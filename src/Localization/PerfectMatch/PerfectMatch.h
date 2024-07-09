#ifndef PERFECTMATCH_H
#define PERFECTMATCH_H

#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <chrono>

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Map/Map.h"
#include "config/config.h"
#include "Logger/logger.h"
#include "utils/utils.h"
#include "data_structures/data_structures.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

class PerfectMatch
{
public:
   PerfectMatch(Logger &logger, const Pose startPose = Pose(), const double stepScale = STEP_SCALE);

   Pose match(std::vector<LaserPoint> &data);

   void setPose(const Pose pose) { this->RobotPose = pose; }

   Pose getPose() const { return RobotPose; };

   double getError() const { return this->pmError; }

   void ProcessLaserPoints(std::vector<LaserPoint> &LaserPoints);
   void ProcessLaserPoints(std::vector<LaserPoint> &LaserPoints, const Pose &previousPose, const Pose &currentPose);

   void ProcessBBOutliersFront(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter);
   void ProcessBBOutliersBack(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter);
   void ProcessBBOutliersLeft(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter);
   void ProcessBBOutliersRight(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter);

   void setStep(const double stepScale) { this->stepScale = stepScale; }

   double getStep() const { return this->stepScale; }

   void setSafetyThreshold(const int factor) { this->safety_threshold = factor; }

   int getSafetyThreshold() const { return this->safety_threshold; }

private:
   void RotateAndTranslate(double &rx, double &ry, double px, double py, double tx, double ty, double st, double ct);

   Pose interpolatePose(const Pose &previousPose, const Pose &currentPose, const double alpha);

   int XTopixel(double x);

   int YTopixel(double y);

   void IterLaser(std::vector<LaserPoint> &LaserPoints);

   bool isPointInsideBB(const Vector2d &point, const BoundingBox &box);
   bool isPointInsideBB_real(const Vector2d &point, const BoundingBox &box);

   void calibrate_lidar_points(double &rx, double &ry, double px, double py);

   // void CorrectDistortion(Vector3d &point);

   Map map;
   double meterToPixel;
   Pose RobotPose;
   double stepScale;
   static constexpr double degreeStep = LASER_RANGE / LASER_RAYS;
   double pmError;

   Logger &logger;

   int safety_threshold;

   // ------------------------------------- Front Cam -------------------------------------------------------//
   // Constant members that represent the spatial-relationship between LiDAR Scanner and the Front Camera in the robot
   const Vector3d t_FC_L = Vector3d(0, -6.2e-2, -13.9e-2); // translation of Lidar frame in FrontCam's frame perspective
   const Matrix3d R_FC_L = (Matrix3d() << 0, -1, 0,
                            -sin(ALPHA), 0, -cos(ALPHA),
                            cos(ALPHA), 0, -sin(ALPHA))
                               .finished();

   const Matrix4d T_FC_L = (Matrix4d() << R_FC_L(0, 0), R_FC_L(0, 1), R_FC_L(0, 2), t_FC_L(0),
                            R_FC_L(1, 0), R_FC_L(1, 1), R_FC_L(1, 2), t_FC_L(1),
                            R_FC_L(2, 0), R_FC_L(2, 1), R_FC_L(2, 2), t_FC_L(2),
                            0, 0, 0, 1)
                               .finished(); // Lidar to FrontCam homogeneous transformation matrix
   /*K = [fx, skew, px
           0, fy, py,
           0, 0, 1]*/
   const Matrix3d K_FC;
   const VectorXd distCoeffs;

   // ------------------------------------- Left Cam -------------------------------------------------------//

   /*Camera Matrix:
 [[674.44174186   0.         316.44341026]
 [  0.         674.70844711 236.59682685]
 [  0.           0.           1.        ]]

Distortion Parameters:
 [[ 3.47526339e-01 -2.30942123e+00 -1.72119376e-03 -8.70119278e-04
   4.56941296e+00]]*/
   const Vector3d t_LC_L = Vector3d(0, -5.575e-2, -10.6e-2);
   const Matrix3d R_LC_L = (Matrix3d() << 1, 0, 0,
                            0, 0, -1,
                            0, 1, 0)
                               .finished();
   const Matrix4d T_LC_L = (Matrix4d() << R_LC_L(0, 0), R_LC_L(0, 1), R_LC_L(0, 2), t_LC_L(0),
                            R_LC_L(1, 0), R_LC_L(1, 1), R_LC_L(1, 2), t_LC_L(1),
                            R_LC_L(2, 0), R_LC_L(2, 1), R_LC_L(2, 2), t_LC_L(2),
                            0, 0, 0, 1)
                               .finished(); // Lidar to Left Cam homogeneous transformation matrix

   const Matrix3d K_LC = (Matrix3d() << 674.44174186, 0, 316.44341026,
                          0, 674.70844711, 236.59682685,
                          0, 0, 1)
                             .finished();

   const VectorXd distCoeffs_LC = (VectorXd::Zero(5));
   // ------------------------------------- Right Cam -------------------------------------------------------//
   /*Camera Matrix:
 [[759.42170674   0.         323.09035821]
 [  0.         761.1566875  237.76728549]
 [  0.           0.           1.        ]]

Distortion Parameters:
 [[-2.31090602e-02  1.04684130e+00 -2.26833338e-02  1.19606686e-04
  -4.65080518e+00]]*/

   const Vector3d t_RC_L = Vector3d(0, -5.275e-2, -10.4e-2);
   const Matrix3d R_RC_L = (Matrix3d() << -1, 0, 0,
                            0, 0, -1,
                            0, -1, 0)
                               .finished();
   const Matrix4d T_RC_L = (Matrix4d() << R_RC_L(0, 0), R_RC_L(0, 1), R_RC_L(0, 2), t_RC_L(0),
                            R_RC_L(1, 0), R_RC_L(1, 1), R_RC_L(1, 2), t_RC_L(1),
                            R_RC_L(2, 0), R_RC_L(2, 1), R_RC_L(2, 2), t_RC_L(2),
                            0, 0, 0, 1)
                               .finished();
   // Lidar to Right Cam homogeneous transformation matrix

   const Matrix3d K_RC = (Matrix3d() << 759.42170674, 0, 323.09035821,
                          0, 761.1566875, 237.76728549,
                          0, 0, 1)
                             .finished();
   const VectorXd distCoeffs_RC = (VectorXd::Zero(5));

   // ------------------------------------- Back Cam -------------------------------------------------------//
   /*Camera Matrix:
 [[754.46770451   0.         318.61648504]
 [  0.         753.77395257 219.42914453]
 [  0.           0.           1.        ]]

Distortion Parameters:
 [[-4.39736985e-02  1.49518613e+00  9.86679537e-03  6.35294189e-03
  -7.22896898e+00]]
*/

   const Vector3d t_BC_L = Vector3d(6e-3, -5.675e-2, -14.1e-2);
   const Matrix3d R_BC_L = (Matrix3d() << 0, 1, 0,
                            0, 0, -1,
                            -1, 0, 0)
                               .finished();

   const Matrix4d T_BC_L = (Matrix4d() << R_BC_L(0, 0), R_BC_L(0, 1), R_BC_L(0, 2), t_BC_L(0),
                            R_BC_L(1, 0), R_BC_L(1, 1), R_BC_L(1, 2), t_BC_L(1),
                            R_BC_L(2, 0), R_BC_L(2, 1), R_BC_L(2, 2), t_BC_L(2),
                            0, 0, 0, 1)
                               .finished();
   // Lidar to Back Cam homogeneous transformation matrix

   const Matrix3d K_BC = (Matrix3d() << 754.46770375, 0, 318.61648504,
                          0, 753.77395257, 219.42914453,
                          0, 0, 1)
                             .finished();
   const VectorXd distCoeffs_BC = (VectorXd::Zero(5));
};

#endif // PERFECTMATCH_H