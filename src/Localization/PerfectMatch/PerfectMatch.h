#include <array>
#include <cmath>
#include <stdexcept>
#include "config.h"

class PerfectMatch {
public:
    PerfectMatch();
    Pose match(const std::array<double, 720>& data);

private:
    void RotateAndTranslate(double& rx, double& ry, double px, double py, double tx, double ty, double st, double ct);
    int XTopixel(double x);
    int YTopixel(double y);
    void CalcDistMap(std::array<std::array<int, 640>, 480>& Map);
    int ScanDistMap(std::array<std::array<int, 640>, 480>& Map, int v);
    double d_err(double d);
    void IterLaser(Pose& R, const std::array<LaserPoint, 720>& LaserPoints, int FirstIdx, int LastIdx, double scale);
    void CalcGradMap(std::array<std::array<float, 640>, 480>& GradXMap, std::array<std::array<float, 640>, 480>& GradYMap, const std::array<std::array<int, 640>, 480>& Map);

    int ImgWidth;
    int ImgHeight;
    double PixelSize;
    double PixelScale;
    std::array<std::array<int, 640>, 480> DistMap;
    std::array<std::array<float, 640>, 480> GradXMap;
    std::array<std::array<float, 640>, 480> GradYMap;
    std::array<std::array<float, 640>, 480> GradThetaMap;
    Pose RobotPose;
    double c_err;
};