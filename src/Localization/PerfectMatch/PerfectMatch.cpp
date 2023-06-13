#include "PerfectMatch.h"

PerfectMatch::PerfectMatch() : ImgWidth(640), ImgHeight(480), PixelSize(0.0075), PixelScale(1 / PixelSize) {
    // Initialize the matching
}

Pose PerfectMatch::match(const std::array<double, 720>& data) {
    // Implement the matching algorithm and return the results
    Pose tmp;

    return tmp;
}

void PerfectMatch::RotateAndTranslate(double& rx, double& ry, double px, double py, double tx, double ty, double st, double ct) {
    rx = px * ct - py * st + tx;
    ry = px * st + py * ct + ty;
}

int PerfectMatch::XTopixel(double x) {
    return static_cast<int>(std::round(x * PixelScale) + ImgWidth / 2);
}

int PerfectMatch::YTopixel(double y) {
    return static_cast<int>(std::round(-y * PixelScale) + ImgHeight / 2);
}

void PerfectMatch::CalcDistMap(std::array<std::array<int, 640>, 480>& Map) {
    int misses = 0;
    for (int i = 0; i < 1000; ++i) {
        if (ScanDistMap(Map, i) == 0) {
            ++misses;
            if (misses > 2) break;
        } else {
            misses = 0;
        }
    }
}

int PerfectMatch::ScanDistMap(std::array<std::array<int, 640>, 480>& Map, int v) {
    int result = 0;
    for (int y = 1; y < ImgHeight - 1; ++y) {
        for (int x = 1; x < ImgWidth - 1; ++x) {
            if (Map[y][x] != v) continue;
            ++result;

            Map[y][x + 1] = std::min(Map[y][x + 1], 2 + v);
            Map[y][x - 1] = std::min(Map[y][x - 1], 2 + v);
            Map[y + 1][x] = std::min(Map[y + 1][x], 2 + v);
            Map[y - 1][x] = std::min(Map[y - 1][x], 2 + v);

            Map[y + 1][x + 1] = std::min(Map[y + 1][x + 1], 3 + v);
            Map[y + 1][x - 1] = std::min(Map[y + 1][x - 1], 3 + v);
            Map[y - 1][x + 1] = std::min(Map[y - 1][x + 1], 3 + v);
            Map[y - 1][x - 1] = std::min(Map[y - 1][x - 1], 3 + v);
        }
    }
    return result;
}

double PerfectMatch::d_err(double d) {
    double c2 = c_err * c_err;
    return 1 - c2 / (c2 + d * d);
}

void PerfectMatch::IterLaser(Pose& R, const std::array<LaserPoint, 720>& LaserPoints, int FirstIdx, int LastIdx, double scale) {
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    double st = std::sin(R.getTheta());
    double ct = std::cos(R.getTheta());
    R.setErr(0);
    int n = 0;

    for (int i = FirstIdx; i <= LastIdx; ++i) {
        if (LaserPoints[i].d < 0.1) continue;
        double rx, ry;
        RotateAndTranslate(rx, ry, LaserPoints[i].x, LaserPoints[i].y, R.getX(), R.getY(), st, ct);
        int u = XTopixel(rx);
        int v = YTopixel(ry);
        if (u > 0 && u < ImgWidth - 1 && v > 0 && v < ImgHeight - 1) {
            double gradX = GradXMap[v][u];
            double gradY = GradYMap[v][u];

            dx -= gradX / LaserPoints[i].std;
            dy += gradY / LaserPoints[i].std;
            dtheta -= gradX / LaserPoints[i].std * (-LaserPoints[i].x * st - LaserPoints[i].y * ct)
                      + gradY / LaserPoints[i].std * (LaserPoints[i].x * ct - LaserPoints[i].y * st);
            R.setErr(R.getErr() + DistMap[v][u]);
            ++n;
        }
    }
    R.setX(R.getX() + scale * dx);
    R.setY(R.getY() + scale * dy);
    R.setTheta(R.getTheta() + M_PI * scale * dtheta);
    if (n > 0) R.setErr(R.getErr() / n);
}

void PerfectMatch::CalcGradMap(std::array<std::array<float, 640>, 480>& GradXMap, std::array<std::array<float, 640>, 480>& GradYMap, const std::array<std::array<int, 640>, 480>& Map) {
    for (int y = 1; y < ImgHeight - 1; ++y) {
        for (int x = 1; x < ImgWidth - 1; ++x) {
            GradXMap[y][x] = (d_err(Map[y][x + 1]) - d_err(Map[y][x - 1])) / 2;
            GradYMap[y][x] = (d_err(Map[y + 1][x]) - d_err(Map[y - 1][x])) / 2;
        }
    }
}