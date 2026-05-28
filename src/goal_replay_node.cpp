/**
 * @file goal_replay_node.cpp
 * @brief Replays a pre-generated dynus_interfaces/Goal trajectory from a CSV file.
 *
 * The CSV is expected to have the header:
 *   t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw,dyaw,power,mode_xy,mode_z
 * one row per time-slice (dt taken from the first two t values). On trigger the
 * node streams each row as a dynus_interfaces/Goal on the "goal" topic, stamping
 * the header with the wall clock, then stops (single-shot).
 *
 * @date 2026-05-28
 */
#include "trajectory_generator/goal_replay_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace trajectory_generator {

namespace {
// Parse a "true"/"false"/"1"/"0" token (case-insensitive) to bool.
bool parseBool(std::string s) {
    for (auto& c : s) c = static_cast<char>(::tolower(c));
    return s == "true" || s == "1";
}

// Append a per-axis quintic (min-jerk) segment to `out`, sampling at `dt` over
// [0, T). Boundary pos/vel/acc are matched at both ends. `tmpl` supplies the
// non-kinematic fields (frame_id, yaw, power, modes) copied onto every sample.
void appendQuinticSegment(std::vector<dynus_interfaces::msg::Goal>& out,
                          const std::array<double, 3>& p0, const std::array<double, 3>& v0,
                          const std::array<double, 3>& a0, const std::array<double, 3>& p1,
                          const std::array<double, 3>& v1, const std::array<double, 3>& a1,
                          double T, double dt, const dynus_interfaces::msg::Goal& tmpl)
{
    struct Quintic { double c0, c1, c2, c3, c4, c5; };
    std::array<Quintic, 3> q;
    for (int i = 0; i < 3; ++i) {
        const double D = p1[i] - p0[i];
        const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
        q[i].c0 = p0[i];
        q[i].c1 = v0[i];
        q[i].c2 = 0.5 * a0[i];
        q[i].c3 = (20.0 * D - (8.0 * v1[i] + 12.0 * v0[i]) * T - (3.0 * a0[i] - a1[i]) * T2) / (2.0 * T3);
        q[i].c4 = (-30.0 * D + (14.0 * v1[i] + 16.0 * v0[i]) * T + (3.0 * a0[i] - 2.0 * a1[i]) * T2) / (2.0 * T4);
        q[i].c5 = (12.0 * D - (6.0 * v1[i] + 6.0 * v0[i]) * T - (a0[i] - a1[i]) * T2) / (2.0 * T5);
    }

    const size_t n = static_cast<size_t>(std::floor(T / dt));
    out.reserve(out.size() + n);
    for (size_t k = 0; k < n; ++k) {
        const double t = k * dt;
        const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        dynus_interfaces::msg::Goal g = tmpl;
        double* pos[3] = {&g.p.x, &g.p.y, &g.p.z};
        double* vel[3] = {&g.v.x, &g.v.y, &g.v.z};
        double* acc[3] = {&g.a.x, &g.a.y, &g.a.z};
        double* jrk[3] = {&g.j.x, &g.j.y, &g.j.z};
        for (int i = 0; i < 3; ++i) {
            const auto& c = q[i];
            *pos[i] = c.c0 + c.c1 * t + c.c2 * t2 + c.c3 * t3 + c.c4 * t4 + c.c5 * t5;
            *vel[i] = c.c1 + 2 * c.c2 * t + 3 * c.c3 * t2 + 4 * c.c4 * t3 + 5 * c.c5 * t4;
            *acc[i] = 2 * c.c2 + 6 * c.c3 * t + 12 * c.c4 * t2 + 20 * c.c5 * t3;
            *jrk[i] = 6 * c.c3 + 24 * c.c4 * t + 60 * c.c5 * t2;
        }
        out.push_back(std::move(g));
    }
}

double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) +
                     (a[1] - b[1]) * (a[1] - b[1]) +
                     (a[2] - b[2]) * (a[2] - b[2]));
}

// Expand a leading "~" to $HOME (the shell does not expand it inside a
// `name:=~/path` launch argument, so the node receives a literal "~").
std::string expandUser(const std::string& path) {
    if (path == "~" || path.rfind("~/", 0) == 0) {
        if (const char* home = std::getenv("HOME")) {
            return std::string(home) + path.substr(1);
        }
    }
    return path;
}
} // namespace

GoalReplayNode::GoalReplayNode(const rclcpp::NodeOptions& options)
    : Node("goal_replay_node", options)
{
    const std::string csv_path   = this->declare_parameter<std::string>("csv_path", "");
    const std::string topic      = this->declare_parameter<std::string>("topic", "goal");
    const std::string pose_topic = this->declare_parameter<std::string>("pose_topic", "pose");
    frame_id_                    = this->declare_parameter<std::string>("frame_id", "world");
    rate_scale_                  = this->declare_parameter<double>("rate_scale", 1.0);
    const bool auto_start        = this->declare_parameter<bool>("auto_start", false);
    const double start_delay     = this->declare_parameter<double>("start_delay", 3.0);
    approach_enabled_            = this->declare_parameter<bool>("approach_enabled", true);
    approach_speed_              = this->declare_parameter<double>("approach_speed", 1.0);
    approach_min_time_           = this->declare_parameter<double>("approach_min_time", 2.0);
    takeoff_z_                   = this->declare_parameter<double>("takeoff_z", 1.0);

    if (csv_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'csv_path' is required.");
        throw std::runtime_error("csv_path not set");
    }
    if (rate_scale_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "rate_scale must be > 0 (got %.3f)", rate_scale_);
        throw std::runtime_error("invalid rate_scale");
    }
    if (approach_speed_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "approach_speed must be > 0 (got %.3f)", approach_speed_);
        throw std::runtime_error("invalid approach_speed");
    }

    loadCsv(csv_path);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable().durability_volatile();
    goal_pub_ = this->create_publisher<dynus_interfaces::msg::Goal>(topic, qos);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, qos,
        std::bind(&GoalReplayNode::poseCallback, this, std::placeholders::_1));

    rclcpp::QoS latched(rclcpp::KeepLast(1));
    latched.reliable().transient_local();
    ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", latched);

    trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "start_replay",
        std::bind(&GoalReplayNode::onTrigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu goals (dt=%.4f s, duration=%.2f s). Publishing to '%s' (frame '%s') at %.2fx.",
                goals_.size(), dt_, goals_.empty() ? 0.0 : goals_.size() * dt_,
                topic.c_str(), frame_id_.c_str(), rate_scale_);
    RCLCPP_INFO(this->get_logger(),
                "Approach planning %s (takeoff_z=%.2f m, speed=%.2f m/s, min_time=%.2f s, pose topic '%s'). CSV start = [%.2f, %.2f, %.2f].",
                approach_enabled_ ? "ENABLED" : "disabled", takeoff_z_, approach_speed_, approach_min_time_,
                pose_topic.c_str(), goals_.front().p.x, goals_.front().p.y, goals_.front().p.z);

    if (auto_start) {
        RCLCPP_INFO(this->get_logger(),
                    "auto_start=true: replay will begin in %.1f s (waiting for state).", start_delay);
        start_delay_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(start_delay)),
            [this]() {
                start_delay_timer_->cancel();
                startReplay();
            });
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for trigger. Call: ros2 service call %s/start_replay std_srvs/srv/Trigger",
                    this->get_namespace());
    }
}

void GoalReplayNode::loadCsv(const std::string& raw_path)
{
    const std::string path = expandUser(raw_path);
    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open CSV: %s", path.c_str());
        throw std::runtime_error("cannot open csv");
    }

    std::string line;
    bool header_skipped = false;
    double first_t = 0.0, second_t = 0.0;
    int t_count = 0;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (!header_skipped) { header_skipped = true; continue; }  // skip header row

        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> cols;
        while (std::getline(ss, cell, ',')) cols.push_back(cell);

        if (cols.size() < 18) {
            RCLCPP_WARN(this->get_logger(), "Skipping malformed row (%zu cols): %s",
                        cols.size(), line.c_str());
            continue;
        }

        const double t = std::stod(cols[0]);
        if (t_count == 0) first_t = t;
        else if (t_count == 1) second_t = t;
        ++t_count;

        dynus_interfaces::msg::Goal g;
        g.header.frame_id = frame_id_;
        g.p.x = std::stod(cols[1]);  g.p.y = std::stod(cols[2]);  g.p.z = std::stod(cols[3]);
        g.v.x = std::stod(cols[4]);  g.v.y = std::stod(cols[5]);  g.v.z = std::stod(cols[6]);
        g.a.x = std::stod(cols[7]);  g.a.y = std::stod(cols[8]);  g.a.z = std::stod(cols[9]);
        g.j.x = std::stod(cols[10]); g.j.y = std::stod(cols[11]); g.j.z = std::stod(cols[12]);
        g.yaw  = std::stod(cols[13]);
        g.dyaw = std::stod(cols[14]);
        g.power = parseBool(cols[15]);
        g.mode_xy = static_cast<uint8_t>(std::stoi(cols[16]));
        g.mode_z  = static_cast<uint8_t>(std::stoi(cols[17]));

        goals_.push_back(std::move(g));
    }

    if (goals_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "CSV contained no data rows: %s", path.c_str());
        throw std::runtime_error("empty csv");
    }
    if (t_count >= 2 && second_t > first_t) {
        dt_ = second_t - first_t;
    } else {
        RCLCPP_WARN(this->get_logger(), "Could not infer dt from CSV; using default %.4f s", dt_);
    }
}

void GoalReplayNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    cur_pos_ = std::array<double, 3>{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}

std::vector<dynus_interfaces::msg::Goal> GoalReplayNode::buildApproach() const
{
    std::vector<dynus_interfaces::msg::Goal> approach;
    if (!approach_enabled_ || !cur_pos_.has_value()) return approach;

    const auto& g0 = goals_.front();
    const std::array<double, 3> p_cur = *cur_pos_;
    const std::array<double, 3> zero  = {0.0, 0.0, 0.0};
    const std::array<double, 3> p_csv = {g0.p.x, g0.p.y, g0.p.z};
    const std::array<double, 3> v_csv = {g0.v.x, g0.v.y, g0.v.z};
    const std::array<double, 3> a_csv = {g0.a.x, g0.a.y, g0.a.z};

    // Template for non-kinematic fields: hold the CSV's (constant) yaw, motors on.
    dynus_interfaces::msg::Goal tmpl;
    tmpl.header.frame_id = frame_id_;
    tmpl.yaw = g0.yaw;
    tmpl.dyaw = 0.0;
    tmpl.power = true;
    tmpl.mode_xy = g0.mode_xy;
    tmpl.mode_z = g0.mode_z;

    // Phase 1 — vertical takeoff in place: climb (or descend) to takeoff_z at the
    // CURRENT x/y, ending at rest. Keeps the vehicle from translating while low.
    const std::array<double, 3> p_hover = {p_cur[0], p_cur[1], takeoff_z_};
    const double climb = dist3(p_cur, p_hover);
    if (climb > 1e-3) {
        const double T1 = std::max(approach_min_time_, climb / approach_speed_);
        appendQuinticSegment(approach, p_cur, zero, zero, p_hover, zero, zero, T1, dt_, tmpl);
    }

    // Phase 2 — transit from the hover point to the CSV start, matching the CSV's
    // initial velocity/acceleration so the handoff is smooth.
    const double transit = dist3(p_hover, p_csv);
    if (transit > 1e-3) {
        const double T2 = std::max(approach_min_time_, transit / approach_speed_);
        appendQuinticSegment(approach, p_hover, zero, zero, p_csv, v_csv, a_csv, T2, dt_, tmpl);
    }

    return approach;
}

void GoalReplayNode::startReplay()
{
    if (running_) {
        RCLCPP_WARN(this->get_logger(), "Replay already running; ignoring start.");
        return;
    }

    std::vector<dynus_interfaces::msg::Goal> approach = buildApproach();
    if (approach_enabled_ && !cur_pos_.has_value()) {
        RCLCPP_WARN(this->get_logger(),
                    "Approach enabled but no drone pose received yet; starting at CSV start point directly.");
    }

    playback_ = std::move(approach);
    playback_.insert(playback_.end(), goals_.begin(), goals_.end());

    idx_ = 0;
    running_ = true;

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(dt_ / rate_scale_));
    timer_ = this->create_wall_timer(period, std::bind(&GoalReplayNode::tick, this));

    // Publish the full planned path (approach + CSV) for RViz (latched).
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = this->now();
    path.poses.reserve(playback_.size());
    for (const auto& g : playback_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = g.p.x;
        ps.pose.position.y = g.p.y;
        ps.pose.position.z = g.p.z;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
    }
    ref_path_pub_->publish(path);

    const size_t n_approach = playback_.size() - goals_.size();
    RCLCPP_INFO(this->get_logger(),
                "Replay started: %zu approach + %zu CSV = %zu goals (approach %.2f s).",
                n_approach, goals_.size(), playback_.size(), n_approach * dt_);
}

void GoalReplayNode::tick()
{
    if (idx_ >= playback_.size()) {
        timer_->cancel();
        running_ = false;
        RCLCPP_INFO(this->get_logger(), "Replay complete (%zu goals published).", playback_.size());
        return;
    }

    dynus_interfaces::msg::Goal g = playback_[idx_];
    g.header.stamp = this->now();
    goal_pub_->publish(g);
    ++idx_;
}

void GoalReplayNode::onTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (running_) {
        res->success = false;
        res->message = "Replay already running.";
        return;
    }
    startReplay();
    res->success = true;
    res->message = "Replay started (" + std::to_string(playback_.size()) + " goals).";
}

} // namespace trajectory_generator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<trajectory_generator::GoalReplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
