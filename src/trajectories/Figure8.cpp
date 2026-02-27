/**
 * @file Figure8.cpp
 * @brief Figure-8 trajectory generation with arc-length parameterization
 * @date 2026-02-27
 */

#include "trajectory_generator/trajectories/Figure8.hpp"
#include <cmath>
#include <vector>

namespace trajectory_generator {

void Figure8::generateTraj(std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
    waypoints.clear();

    // Step 1: Densely sample ONE loop to compute cumulative arc length
    const int N = 10000;
    std::vector<double> t_samples(N + 1);
    std::vector<double> arc_len(N + 1, 0.0);
    std::vector<double> x_samples(N + 1);
    std::vector<double> y_samples(N + 1);

    for (int i = 0; i <= N; ++i) {
        double t = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(N);
        t_samples[i] = t;
        x_samples[i] = amp_x_ * std::sin(t);
        y_samples[i] = amp_y_ * std::sin(t) * std::cos(t);

        if (i > 0) {
            double dx = x_samples[i] - x_samples[i - 1];
            double dy = y_samples[i] - y_samples[i - 1];
            arc_len[i] = arc_len[i - 1] + std::sqrt(dx * dx + dy * dy);
        }
    }

    double loop_length = arc_len[N];
    double step = velocity_ * dt_;
    int points_per_loop = std::max(2, static_cast<int>(std::floor(loop_length / step)));

    // Step 2: Generate waypoints for num_laps_ consecutive loops
    waypoints.reserve(points_per_loop * num_laps_);

    for (int lap = 0; lap < num_laps_; ++lap) {
        int j = 0;  // index into dense samples (reset each lap)
        for (int i = 0; i < points_per_loop; ++i) {
            double target_s = static_cast<double>(i) * step;

            while (j < N && arc_len[j + 1] < target_s) ++j;

            double seg = arc_len[j + 1] - arc_len[j];
            double alpha = (seg > 1e-12) ? (target_s - arc_len[j]) / seg : 0.0;

            double x = x_samples[j] + alpha * (x_samples[j + 1] - x_samples[j]);
            double y = y_samples[j] + alpha * (y_samples[j + 1] - y_samples[j]);

            double t = t_samples[j] + alpha * (t_samples[j + 1] - t_samples[j]);
            double dx_dt = amp_x_ * std::cos(t);
            double dy_dt = amp_y_ * std::cos(2.0 * t);
            double yaw = std::atan2(dy_dt, dx_dt);

            waypoints.push_back(createPoseStamped(x, y, yaw));
        }
    }
}

bool Figure8::trajectoryInsideBounds(double xmin, double xmax,
                                     double ymin, double ymax)
{
    return isPointInsideBounds(xmin, xmax, ymin, ymax, -amp_x_, -amp_y_ / 2.0) &&
           isPointInsideBounds(xmin, xmax, ymin, ymax,  amp_x_,  amp_y_ / 2.0);
}

} // namespace trajectory_generator
