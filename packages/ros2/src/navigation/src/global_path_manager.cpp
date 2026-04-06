#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

using json = nlohmann::json;

struct XY {
  double x;
  double y;
};

namespace {
constexpr double kPi = 3.14159265358979323846;

struct DatumFallback {
  double lat;
  double lon;
};

struct FixSnapshot {
  double lat{0.0};
  double lon{0.0};
  double alt{0.0};
  rclcpp::Time stamp;
  bool valid{false};
};

constexpr std::array<DatumFallback, 2> kDatumFallbacks{{
  {28.6017759, -81.2005371},
  {28.5188, -81.6701},
}};

double distXy(const XY &a, const XY &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double degToRad(double deg) { return deg * kPi / 180.0; }

bool isValidLatLon(double lat, double lon) {
  return std::isfinite(lat) && std::isfinite(lon) &&
         lat >= -90.0 && lat <= 90.0 &&
         lon >= -180.0 && lon <= 180.0;
}

double haversineDistanceM(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadiusM = 6371000.0;
  const double p1 = degToRad(lat1);
  const double p2 = degToRad(lat2);
  const double dp = degToRad(lat2 - lat1);
  const double dl = degToRad(lon2 - lon1);
  const double sinDp = std::sin(dp * 0.5);
  const double sinDl = std::sin(dl * 0.5);
  const double a = sinDp * sinDp + std::cos(p1) * std::cos(p2) * sinDl * sinDl;
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
  return kEarthRadiusM * c;
}

double horizontalCovarianceMetricM2(const sensor_msgs::msg::NavSatFix &msg) {
  if (msg.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    return std::numeric_limits<double>::infinity();
  }

  const double xx = msg.position_covariance[0];
  const double yy = msg.position_covariance[4];
  if (!std::isfinite(xx) || !std::isfinite(yy) || xx < 0.0 || yy < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  return std::max(xx, yy);
}

double wrapAngleRad(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

double pointToSegmentDistance(const XY &p, const XY &a, const XY &b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double denom = dx * dx + dy * dy;
  if (denom <= 1e-12) return distXy(p, a);
  double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / denom;
  t = std::clamp(t, 0.0, 1.0);
  const XY proj{a.x + t * dx, a.y + t * dy};
  return distXy(p, proj);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

geometry_msgs::msg::PoseArray makePoseArray(const std::vector<XY> &pts, const std::string &frameId, const rclcpp::Time &stamp) {
  geometry_msgs::msg::PoseArray pa;
  pa.header.frame_id = frameId;
  pa.header.stamp = stamp;
  pa.poses.reserve(pts.size());
  for (const auto &pt : pts) {
    geometry_msgs::msg::Pose p;
    p.position.x = pt.x;
    p.position.y = pt.y;
    p.position.z = 0.0;
    p.orientation.w = 1.0;
    pa.poses.push_back(p);
  }
  return pa;
}

bool readNumber(const json &j, const char *key, double &out) {
  auto it = j.find(key);
  if (it == j.end()) return false;
  if (it->is_number()) {
    out = it->get<double>();
    return std::isfinite(out);
  }
  if (it->is_string()) {
    try {
      out = std::stod(it->get<std::string>());
      return std::isfinite(out);
    } catch (const std::exception &) { return false; }
  }
  return false;
}

std::vector<XY> dedupeConsecutiveKeepEnds(const std::vector<XY> &pts, double minDistM) {
  std::vector<XY> out;
  if (pts.empty()) return out;
  if (pts.size() == 1) {
    out.push_back(pts.front());
    return out;
  }
  out.reserve(pts.size());
  out.push_back(pts.front());
  for (size_t i = 1; i + 1 < pts.size(); i++) {
    if (distXy(pts[i], out.back()) >= minDistM) out.push_back(pts[i]);
  }
  if (distXy(pts.back(), out.back()) >= 1e-9) {
    out.push_back(pts.back());
  } else {
    out.back() = pts.back();
  }
  return out;
}

std::vector<XY> removeKinksGuarded(const std::vector<XY> &pts, double kinkShortM, double uTurnDeg, double collinearDistM, double shortcutRatioMin,
                                   double shortcutMaxDeltaM) {
  if (pts.size() < 3) return pts;

  std::vector<XY> cur = pts;
  const int maxIters = 6;

  for (int iter = 0; iter < maxIters; iter++) {
    bool removedAny = false;
    std::vector<XY> next;
    next.reserve(cur.size());
    next.push_back(cur.front());

    for (size_t i = 1; i + 1 < cur.size(); i++) {
      const XY &a = cur[i - 1];
      const XY &b = cur[i];
      const XY &c = cur[i + 1];

      const double ab = distXy(a, b);
      const double bc = distXy(b, c);
      const double ac = distXy(a, c);

      bool drop = false;

      if (ab < kinkShortM && bc < kinkShortM) {
        const double v1x = a.x - b.x;
        const double v1y = a.y - b.y;
        const double v2x = c.x - b.x;
        const double v2y = c.y - b.y;
        const double n1 = std::sqrt(v1x * v1x + v1y * v1y);
        const double n2 = std::sqrt(v2x * v2x + v2y * v2y);

        if (n1 > 1e-9 && n2 > 1e-9) {
          const double cosang = std::clamp((v1x * v2x + v1y * v2y) / (n1 * n2), -1.0, 1.0);
          const double ang = std::acos(cosang) * 180.0 / kPi;
          if (ang > uTurnDeg) drop = true;
        }
      }

      if (!drop) {
        const double d = pointToSegmentDistance(b, a, c);
        if (d < collinearDistM && (ab < 2.0 || bc < 2.0) && ac > 1e-6) drop = true;
      }

      if (drop) {
        const double oldLen = ab + bc;
        const double newLen = ac;
        const double ratio = (oldLen > 1e-9) ? (newLen / oldLen) : 1.0;
        const double delta = oldLen - newLen;
        if (ratio < shortcutRatioMin && delta > shortcutMaxDeltaM) drop = false;
      }

      if (drop) {
        removedAny = true;
      } else {
        next.push_back(b);
      }
    }

    next.push_back(cur.back());
    cur.swap(next);

    if (!removedAny) break;
    if (cur.size() < 3) break;
  }

  return cur;
}

std::vector<XY> rdpSimplifyRange(const std::vector<XY> &pts, size_t s, size_t e, double epsilonM) {
  if (e <= s || (e - s + 1) < 3) {
    std::vector<XY> out;
    if (s < pts.size()) out.push_back(pts[s]);
    if (e < pts.size() && e != s) out.push_back(pts[e]);
    return out;
  }

  const size_t n = e - s + 1;
  std::vector<bool> keep(n, false);
  keep[0] = true;
  keep[n - 1] = true;

  std::vector<std::pair<size_t, size_t>> stack;
  stack.reserve(n);
  stack.push_back({0, n - 1});

  while (!stack.empty()) {
    auto seg = stack.back();
    stack.pop_back();
    const size_t a = seg.first;
    const size_t b = seg.second;
    if (b <= a + 1) continue;

    double maxD = -1.0;
    size_t maxI = a;

    const XY &pa = pts[s + a];
    const XY &pb = pts[s + b];

    for (size_t i = a + 1; i < b; i++) {
      const double d = pointToSegmentDistance(pts[s + i], pa, pb);
      if (d > maxD) {
        maxD = d;
        maxI = i;
      }
    }

    if (maxD > epsilonM) {
      keep[maxI] = true;
      stack.push_back({a, maxI});
      stack.push_back({maxI, b});
    }
  }

  std::vector<XY> out;
  out.reserve(n);
  for (size_t i = 0; i < n; i++) {
    if (keep[i]) out.push_back(pts[s + i]);
  }
  return out;
}

std::vector<XY> resampleBySpacing(const std::vector<XY> &pts, double spacingM) {
  std::vector<XY> out;
  if (pts.empty()) return out;
  if (pts.size() == 1) {
    out.push_back(pts[0]);
    return out;
  }
  if (spacingM <= 1e-6) return pts;

  const size_t n = pts.size();
  std::vector<double> s(n, 0.0);
  for (size_t i = 1; i < n; i++)
    s[i] = s[i - 1] + distXy(pts[i - 1], pts[i]);

  const double total = s.back();
  if (total <= 1e-9) {
    out.push_back(pts.front());
    return out;
  }

  const int steps = static_cast<int>(std::floor(total / spacingM));
  out.reserve(static_cast<size_t>(steps) + 2);
  out.push_back(pts.front());

  size_t segI = 0;
  for (int k = 1; k <= steps; k++) {
    const double d = spacingM * static_cast<double>(k);
    while (segI + 1 < n && s[segI + 1] < d)
      segI++;
    if (segI + 1 >= n) break;

    const double segLen = s[segI + 1] - s[segI];
    if (segLen <= 1e-12) continue;

    const double t = (d - s[segI]) / segLen;
    const XY p0 = pts[segI];
    const XY p1 = pts[segI + 1];
    const XY p{p0.x + t * (p1.x - p0.x), p0.y + t * (p1.y - p0.y)};
    if (distXy(p, out.back()) >= 1e-4) out.push_back(p);
  }

  if (distXy(pts.back(), out.back()) >= 1e-4) out.push_back(pts.back());
  return out;
}

double pathLengthM(const std::vector<XY> &pts) {
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); i++)
    len += distXy(pts[i - 1], pts[i]);
  return len;
}

double tangentYaw(const std::vector<XY> &pts, size_t i, int look) {
  const size_t n = pts.size();
  if (n < 2) return 0.0;
  const size_t i0 = (i > static_cast<size_t>(look)) ? (i - static_cast<size_t>(look)) : 0;
  const size_t i1 = std::min(n - 1, i + static_cast<size_t>(look));
  const double dx = pts[i1].x - pts[i0].x;
  const double dy = pts[i1].y - pts[i0].y;
  if (std::abs(dx) < 1e-12 && std::abs(dy) < 1e-12) return 0.0;
  return std::atan2(dy, dx);
}

std::vector<double> computeYaws(const std::vector<XY> &pts, int lookaheadPts, int smoothWindowPts) {
  const size_t n = pts.size();
  std::vector<double> yaws(n, 0.0);
  if (n < 2) return yaws;

  const int look = std::max(1, lookaheadPts);
  for (size_t i = 0; i < n; i++)
    yaws[i] = tangentYaw(pts, i, look);

  const int w = std::max(0, smoothWindowPts);
  if (w <= 1) return yaws;

  std::vector<double> sm(n, 0.0);

  for (size_t i = 0; i < n; i++) {
    const int i0 = std::max<int>(0, static_cast<int>(i) - w / 2);
    const int i1 = std::min<int>(static_cast<int>(n) - 1, static_cast<int>(i) + w / 2);
    double sx = 0.0;
    double sy = 0.0;
    for (int k = i0; k <= i1; k++) {
      sx += std::cos(yaws[static_cast<size_t>(k)]);
      sy += std::sin(yaws[static_cast<size_t>(k)]);
    }
    sm[i] = std::atan2(sy, sx);
  }

  return sm;
}

nav_msgs::msg::Path makePathMsgWithYaws(const std::vector<XY> &pts, const std::vector<double> &yaws, const std::string &frameId, const rclcpp::Time &stamp) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frameId;
  path.header.stamp = stamp;
  path.poses.reserve(pts.size());

  for (size_t i = 0; i < pts.size(); i++) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = pts[i].x;
    ps.pose.position.y = pts[i].y;
    ps.pose.position.z = 0.0;

    const double yaw = (i < yaws.size()) ? yaws[i] : 0.0;
    ps.pose.orientation = yawToQuaternion(yaw);

    path.poses.push_back(ps);
  }

  return path;
}

double signedCurvature(const XY &a, const XY &b, const XY &c) {
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double bcx = c.x - b.x;
  const double bcy = c.y - b.y;
  const double acx = c.x - a.x;
  const double acy = c.y - a.y;

  const double ab = std::sqrt(abx * abx + aby * aby);
  const double bc = std::sqrt(bcx * bcx + bcy * bcy);
  const double ac = std::sqrt(acx * acx + acy * acy);
  if (ab < 1e-6 || bc < 1e-6 || ac < 1e-6) return 0.0;

  const double cross = abx * bcy - aby * bcx;
  const double area2 = std::abs(cross);
  const double k = (2.0 * area2) / (ab * bc * ac);
  return (cross >= 0.0) ? k : -k;
}

double speedFromCurvature(double k, double vMin, double vMax, double aLatMax, double kDeadband) {
  const double ak = std::abs(k);
  if (ak < kDeadband) return vMax;
  const double v = std::sqrt(std::max(0.0, aLatMax / ak));
  return std::clamp(v, vMin, vMax);
}

uint64_t gridKey(int gx, int gy) {
  const uint64_t ux = static_cast<uint32_t>(gx);
  const uint64_t uy = static_cast<uint32_t>(gy);
  return (ux << 32) | uy;
}

void markWindow(std::vector<uint8_t> &protect, int idx, int win) {
  const int n = static_cast<int>(protect.size());
  const int a = std::max(0, idx - win);
  const int b = std::min(n - 1, idx + win);
  for (int i = a; i <= b; i++)
    protect[static_cast<size_t>(i)] = 1;
}

std::vector<size_t> computeProtectedIndices(const std::vector<XY> &pts, double protectTurnDeg, double loopGuardDistM, int loopGuardWindowPts,
                                            int loopGuardAdjacencySkipPts) {
  std::vector<size_t> out;
  const size_t n = pts.size();
  if (n == 0) return out;
  if (n == 1) {
    out.push_back(0);
    return out;
  }

  std::vector<uint8_t> protect(n, 0);
  protect[0] = 1;
  protect[n - 1] = 1;

  for (size_t i = 1; i + 1 < n; i++) {
    const double yaw0 = std::atan2(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
    const double yaw1 = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
    const double dy = wrapAngleRad(yaw1 - yaw0);
    const double deg = std::abs(dy) * 180.0 / kPi;
    if (deg >= protectTurnDeg) protect[i] = 1;
  }

  if (loopGuardDistM > 1e-6) {
    const double gs = loopGuardDistM;
    std::unordered_map<uint64_t, std::vector<size_t>> cell;

    for (size_t i = 0; i < n; i++) {
      const int gx = static_cast<int>(std::floor(pts[i].x / gs));
      const int gy = static_cast<int>(std::floor(pts[i].y / gs));

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          const uint64_t key = gridKey(gx + dx, gy + dy);
          auto it = cell.find(key);
          if (it == cell.end()) continue;

          for (size_t j : it->second) {
            if (i <= j) continue;
            const int di = static_cast<int>(i) - static_cast<int>(j);
            if (di <= loopGuardAdjacencySkipPts) continue;
            if (distXy(pts[i], pts[j]) < loopGuardDistM) {
              markWindow(protect, static_cast<int>(i), loopGuardWindowPts);
              markWindow(protect, static_cast<int>(j), loopGuardWindowPts);
            }
          }
        }
      }

      cell[gridKey(gx, gy)].push_back(i);
    }
  }

  out.reserve(n);
  for (size_t i = 0; i < n; i++) {
    if (protect[i]) out.push_back(i);
  }
  return out;
}

std::vector<XY> constrainedRdp(const std::vector<XY> &pts, double epsilonM, const std::vector<size_t> &protectedIdx) {
  if (pts.size() < 3) return pts;
  if (epsilonM <= 1e-6) return pts;

  std::vector<size_t> anchors = protectedIdx;
  if (anchors.empty()) {
    anchors.push_back(0);
    anchors.push_back(pts.size() - 1);
  }
  std::sort(anchors.begin(), anchors.end());
  anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
  if (anchors.front() != 0) anchors.insert(anchors.begin(), 0);
  if (anchors.back() != pts.size() - 1) anchors.push_back(pts.size() - 1);

  std::vector<XY> out;
  out.reserve(pts.size());

  for (size_t k = 0; k + 1 < anchors.size(); k++) {
    const size_t s = anchors[k];
    const size_t e = anchors[k + 1];
    auto seg = rdpSimplifyRange(pts, s, e, epsilonM);
    if (k > 0 && !seg.empty()) seg.erase(seg.begin());
    out.insert(out.end(), seg.begin(), seg.end());
  }

  return out;
}

rclcpp::QoS buildQosFromParams(const std::string &reliability, const std::string &durability, int depth) {
  rclcpp::QoS qos(rclcpp::KeepLast(std::max(1, depth)));

  if (reliability == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  if (durability == "transient_local") {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }

  return qos;
}
} // namespace

struct Config {
  std::string mapFrame;
  std::string inputTopic;
  std::string outputTopicClean;
  std::string outputTopicRaw;
  std::string outputTopicProfile;
  std::string outputTopicMeta;
  std::string outputTopicPointsRaw;
  std::string outputTopicPointsClean;

  std::string inputReliability;
  std::string inputDurability;
  int inputDepth;

  std::string outputReliability;
  std::string outputDurability;
  int outputDepth;

  double datumLat;
  double datumLon;
  double datumAlt;
  bool autoDatumFromFix;
  std::string fixTopic;
  double datumCovarianceThresholdM2;
  double datumWaitTimeoutS;
  double currentFixMaxAgeS;

  int maxInputPoints;

  double minPointDistM;
  double kinkShortM;
  double uTurnDeg;
  double collinearDistM;
  double kinkShortcutRatioMin;
  double kinkShortcutMaxDeltaM;

  bool enableRemoveKinks;
  bool enableConstrainedRdp;
  double rdpEpsilonM;

  double protectTurnDeg;
  double loopGuardDistM;
  int loopGuardWindowPts;
  int loopGuardAdjacencySkipPts;

  double resampleSpacingM;

  int yawLookaheadPts;
  int yawSmoothWindowPts;

  double vMaxMps;
  double vMinMps;
  double aLatMaxMps2;
  double kDeadband;

  double cleanLenRatioWarnMin;
  double cleanLenRatioAbortMin;

  bool publishProfile;
  bool publishMeta;
  bool publishPoints;
};

class GlobalPathManager : public rclcpp::Node {
public:
  GlobalPathManager() : rclcpp::Node("global_path_manager") {
    Config cfg;

    cfg.mapFrame = this->declare_parameter<std::string>("map_frame", "map");
    cfg.inputTopic = this->declare_parameter<std::string>("input_topic", "/ui/global_route_wgs84_json");
    cfg.outputTopicClean = this->declare_parameter<std::string>("output_topic_clean", "/global_path");
    cfg.outputTopicRaw = this->declare_parameter<std::string>("output_topic_raw", "/global_path_raw");
    cfg.outputTopicProfile = this->declare_parameter<std::string>("output_topic_profile", "/global_path_profile");
    cfg.outputTopicMeta = this->declare_parameter<std::string>("output_topic_meta", "/global_path_meta");
    cfg.outputTopicPointsRaw = this->declare_parameter<std::string>("output_topic_points_raw", "/global_path_points_raw");
    cfg.outputTopicPointsClean = this->declare_parameter<std::string>("output_topic_points_clean", "/global_path_points_clean");

    cfg.inputReliability = this->declare_parameter<std::string>("input_reliability", "reliable");
    cfg.inputDurability = this->declare_parameter<std::string>("input_durability", "transient_local");
    cfg.inputDepth = this->declare_parameter<int>("input_depth", 10);

    cfg.outputReliability = this->declare_parameter<std::string>("output_reliability", "reliable");
    cfg.outputDurability = this->declare_parameter<std::string>("output_durability", "transient_local");
    cfg.outputDepth = this->declare_parameter<int>("output_depth", 1);

    cfg.datumLat = this->declare_parameter<double>("datum_lat", 0.0);
    cfg.datumLon = this->declare_parameter<double>("datum_lon", 0.0);
    cfg.datumAlt = this->declare_parameter<double>("datum_alt", 0.0);
    cfg.autoDatumFromFix = this->declare_parameter<bool>("auto_datum_from_fix", false);
    cfg.fixTopic = this->declare_parameter<std::string>("fix_topic", "/fix");
    cfg.datumCovarianceThresholdM2 = this->declare_parameter<double>("datum_covariance_threshold_m2", 25.0);
    cfg.datumWaitTimeoutS = this->declare_parameter<double>("datum_wait_timeout_s", 5.0);
    cfg.currentFixMaxAgeS = this->declare_parameter<double>("current_fix_max_age_s", 3.0);

    cfg.maxInputPoints = this->declare_parameter<int>("max_input_points", 50000);

    cfg.minPointDistM = this->declare_parameter<double>("min_point_dist_m", 0.5);

    cfg.enableRemoveKinks = this->declare_parameter<bool>("enable_remove_kinks", true);
    cfg.kinkShortM = this->declare_parameter<double>("kink_short_m", 1.5);
    cfg.uTurnDeg = this->declare_parameter<double>("u_turn_deg", 160.0);
    cfg.collinearDistM = this->declare_parameter<double>("collinear_dist_m", 0.35);
    cfg.kinkShortcutRatioMin = this->declare_parameter<double>("kink_shortcut_ratio_min", 0.85);
    cfg.kinkShortcutMaxDeltaM = this->declare_parameter<double>("kink_shortcut_max_delta_m", 1.0);

    cfg.enableConstrainedRdp = this->declare_parameter<bool>("enable_constrained_rdp", true);
    cfg.rdpEpsilonM = this->declare_parameter<double>("rdp_epsilon_m", 0.75);

    cfg.protectTurnDeg = this->declare_parameter<double>("protect_turn_deg", 30.0);
    cfg.loopGuardDistM = this->declare_parameter<double>("loop_guard_dist_m", 3.0);
    cfg.loopGuardWindowPts = this->declare_parameter<int>("loop_guard_window_pts", 2);
    cfg.loopGuardAdjacencySkipPts = this->declare_parameter<int>("loop_guard_adjacency_skip_pts", 10);

    cfg.resampleSpacingM = this->declare_parameter<double>("resample_spacing_m", 0.75);

    cfg.yawLookaheadPts = this->declare_parameter<int>("yaw_lookahead_pts", 5);
    cfg.yawSmoothWindowPts = this->declare_parameter<int>("yaw_smooth_window_pts", 7);

    cfg.vMaxMps = this->declare_parameter<double>("v_max_mps", 1.5);
    cfg.vMinMps = this->declare_parameter<double>("v_min_mps", 0.2);
    cfg.aLatMaxMps2 = this->declare_parameter<double>("a_lat_max_mps2", 1.2);
    cfg.kDeadband = this->declare_parameter<double>("k_deadband", 0.02);

    cfg.cleanLenRatioWarnMin = this->declare_parameter<double>("clean_len_ratio_warn_min", 0.65);
    cfg.cleanLenRatioAbortMin = this->declare_parameter<double>("clean_len_ratio_abort_min", 0.6);

    cfg.publishProfile = this->declare_parameter<bool>("publish_profile", true);
    cfg.publishMeta = this->declare_parameter<bool>("publish_meta", true);
    cfg.publishPoints = this->declare_parameter<bool>("publish_points", true);

    {
      std::lock_guard<std::mutex> lk(cfgMutex_);
      cfg_ = cfg;
      if (cfg.autoDatumFromFix) {
        localCartesian_.reset();
      } else {
        localCartesian_ = std::make_shared<GeographicLib::LocalCartesian>(cfg.datumLat, cfg.datumLon, cfg.datumAlt);
      }
    }

    {
      std::lock_guard<std::mutex> lk(datumMutex_);
      datumReady_ = !cfg.autoDatumFromFix;
      datumShutdown_ = false;
    }
    datumSource_ = cfg.autoDatumFromFix ? "pending" : "parameters";
    datumUsedFallback_ = false;
    datumStartTime_ = this->now();

    const auto outQos = buildQosFromParams(cfg.outputReliability, cfg.outputDurability, cfg.outputDepth);
    rawPub_ = this->create_publisher<nav_msgs::msg::Path>(cfg.outputTopicRaw, outQos);
    cleanPub_ = this->create_publisher<nav_msgs::msg::Path>(cfg.outputTopicClean, outQos);

    profilePub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(cfg.outputTopicProfile, outQos);
    metaPub_ = this->create_publisher<std_msgs::msg::String>(cfg.outputTopicMeta, outQos);
    pointsRawPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(cfg.outputTopicPointsRaw, outQos);
    pointsCleanPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(cfg.outputTopicPointsClean, outQos);

    const auto inQos = buildQosFromParams(cfg.inputReliability, cfg.inputDurability, cfg.inputDepth);
    routeSub_ =
        this->create_subscription<std_msgs::msg::String>(cfg.inputTopic, inQos, std::bind(&GlobalPathManager::onRouteJson, this, std::placeholders::_1));
    fixSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      cfg.fixTopic, rclcpp::SensorDataQoS(), std::bind(&GlobalPathManager::onFix, this, std::placeholders::_1));

    paramsCb_ = this->add_on_set_parameters_callback(std::bind(&GlobalPathManager::onSetParameters, this, std::placeholders::_1));

    workerRunning_ = true;
    worker_ = std::thread([this]() { workerLoop(); });
    if (cfg.autoDatumFromFix) {
      datumTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GlobalPathManager::onDatumTimer, this));
    }

    RCLCPP_INFO(this->get_logger(), "global_path_manager ready");
    if (cfg.autoDatumFromFix) {
      RCLCPP_INFO(
        this->get_logger(),
        "waiting up to %.1fs for %s covariance <= %.2f m^2 to resolve datum",
        cfg.datumWaitTimeoutS, cfg.fixTopic.c_str(), cfg.datumCovarianceThresholdM2);
    } else {
      RCLCPP_INFO(this->get_logger(), "datum lat=%.9f lon=%.9f alt=%.3f", cfg.datumLat, cfg.datumLon, cfg.datumAlt);
    }
    RCLCPP_INFO(this->get_logger(), "input=%s raw=%s clean=%s profile=%s meta=%s points_raw=%s points_clean=%s frame=%s", cfg.inputTopic.c_str(),
                cfg.outputTopicRaw.c_str(), cfg.outputTopicClean.c_str(), cfg.outputTopicProfile.c_str(), cfg.outputTopicMeta.c_str(),
                cfg.outputTopicPointsRaw.c_str(), cfg.outputTopicPointsClean.c_str(), cfg.mapFrame.c_str());
  }

  ~GlobalPathManager() override {
    {
      std::lock_guard<std::mutex> lk(datumMutex_);
      datumShutdown_ = true;
    }
    datumCv_.notify_all();
    {
      std::lock_guard<std::mutex> lk(queueMutex_);
      workerRunning_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

private:
  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    Config next;
    {
      std::lock_guard<std::mutex> lk(cfgMutex_);
      next = cfg_;
    }

    bool datumChanged = false;

    for (const auto &p : params) {
      const auto &name = p.get_name();

      if (name == "min_point_dist_m") {
        next.minPointDistM = p.as_double();
      } else if (name == "enable_remove_kinks") {
        next.enableRemoveKinks = p.as_bool();
      } else if (name == "kink_short_m") {
        next.kinkShortM = p.as_double();
      } else if (name == "u_turn_deg") {
        next.uTurnDeg = p.as_double();
      } else if (name == "collinear_dist_m") {
        next.collinearDistM = p.as_double();
      } else if (name == "kink_shortcut_ratio_min") {
        next.kinkShortcutRatioMin = p.as_double();
      } else if (name == "kink_shortcut_max_delta_m") {
        next.kinkShortcutMaxDeltaM = p.as_double();
      } else if (name == "enable_constrained_rdp") {
        next.enableConstrainedRdp = p.as_bool();
      } else if (name == "rdp_epsilon_m") {
        next.rdpEpsilonM = p.as_double();
      } else if (name == "protect_turn_deg") {
        next.protectTurnDeg = p.as_double();
      } else if (name == "loop_guard_dist_m") {
        next.loopGuardDistM = p.as_double();
      } else if (name == "loop_guard_window_pts") {
        next.loopGuardWindowPts = p.as_int();
      } else if (name == "loop_guard_adjacency_skip_pts") {
        next.loopGuardAdjacencySkipPts = p.as_int();
      } else if (name == "resample_spacing_m") {
        next.resampleSpacingM = p.as_double();
      } else if (name == "yaw_lookahead_pts") {
        next.yawLookaheadPts = p.as_int();
      } else if (name == "yaw_smooth_window_pts") {
        next.yawSmoothWindowPts = p.as_int();
      } else if (name == "v_max_mps") {
        next.vMaxMps = p.as_double();
      } else if (name == "v_min_mps") {
        next.vMinMps = p.as_double();
      } else if (name == "a_lat_max_mps2") {
        next.aLatMaxMps2 = p.as_double();
      } else if (name == "k_deadband") {
        next.kDeadband = p.as_double();
      } else if (name == "clean_len_ratio_warn_min") {
        next.cleanLenRatioWarnMin = p.as_double();
      } else if (name == "clean_len_ratio_abort_min") {
        next.cleanLenRatioAbortMin = p.as_double();
      } else if (name == "max_input_points") {
        next.maxInputPoints = p.as_int();
      } else if (name == "publish_profile") {
        next.publishProfile = p.as_bool();
      } else if (name == "publish_meta") {
        next.publishMeta = p.as_bool();
      } else if (name == "publish_points") {
        next.publishPoints = p.as_bool();
      } else if (name == "datum_lat") {
        next.datumLat = p.as_double();
        datumChanged = true;
      } else if (name == "datum_lon") {
        next.datumLon = p.as_double();
        datumChanged = true;
      } else if (name == "datum_alt") {
        next.datumAlt = p.as_double();
        datumChanged = true;
      } else if (name == "datum_covariance_threshold_m2") {
        next.datumCovarianceThresholdM2 = p.as_double();
      } else if (name == "datum_wait_timeout_s") {
        next.datumWaitTimeoutS = p.as_double();
      } else if (name == "current_fix_max_age_s") {
        next.currentFixMaxAgeS = p.as_double();
      } else if (name == "map_frame") {
        next.mapFrame = p.as_string();
      } else if (name == "input_topic" || name == "output_topic_clean" || name == "output_topic_raw" || name == "output_topic_profile" ||
                 name == "output_topic_meta" || name == "output_topic_points_raw" || name == "output_topic_points_clean" || name == "input_reliability" ||
                 name == "input_durability" || name == "input_depth" || name == "output_reliability" || name == "output_durability" || name == "output_depth" ||
                 name == "auto_datum_from_fix" || name == "fix_topic") {
        res.successful = false;
        res.reason = "topic/qos changes require restart";
        return res;
      }
    }

    if (next.minPointDistM < 0.0 || next.kinkShortM < 0.0 || next.collinearDistM < 0.0 || next.rdpEpsilonM < 0.0 || next.resampleSpacingM < 0.0 ||
        next.vMaxMps < 0.0 || next.vMinMps < 0.0 || next.aLatMaxMps2 < 0.0 || next.kDeadband < 0.0 || next.cleanLenRatioWarnMin < 0.0 ||
        next.cleanLenRatioAbortMin < 0.0 || next.maxInputPoints < 0 || next.loopGuardWindowPts < 0 || next.loopGuardAdjacencySkipPts < 0 ||
        next.yawLookaheadPts < 0 || next.yawSmoothWindowPts < 0 || next.datumCovarianceThresholdM2 < 0.0 || next.datumWaitTimeoutS < 0.0 ||
        next.currentFixMaxAgeS < 0.0) {
      res.successful = false;
      res.reason = "invalid negative parameter";
      return res;
    }

    if (next.cleanLenRatioWarnMin > 1.0 || next.cleanLenRatioAbortMin > 1.0) {
      res.successful = false;
      res.reason = "clean_len_ratio_* must be <= 1.0";
      return res;
    }

    if (next.vMinMps > next.vMaxMps) {
      res.successful = false;
      res.reason = "v_min_mps must be <= v_max_mps";
      return res;
    }

    if (next.cleanLenRatioAbortMin > next.cleanLenRatioWarnMin) {
      res.successful = false;
      res.reason = "clean_len_ratio_abort_min must be <= clean_len_ratio_warn_min";
      return res;
    }

    {
      std::lock_guard<std::mutex> lk(cfgMutex_);
      cfg_ = next;
      if (datumChanged) {
        localCartesian_ = std::make_shared<GeographicLib::LocalCartesian>(next.datumLat, next.datumLon, next.datumAlt);
        datumSource_ = "parameters";
        datumUsedFallback_ = false;
      }
    }
    if (datumChanged) {
      {
        std::lock_guard<std::mutex> lk(datumMutex_);
        datumReady_ = true;
      }
      datumCv_.notify_all();
      if (datumTimer_) datumTimer_->cancel();
    }

    return res;
  }

  void publishClearOutputs(const Config &cfg, const std::string &routeId) {
    const auto stamp = this->now();

    nav_msgs::msg::Path emptyPath;
    emptyPath.header.frame_id = cfg.mapFrame;
    emptyPath.header.stamp = stamp;
    rawPub_->publish(emptyPath);
    cleanPub_->publish(emptyPath);

    if (cfg.publishProfile && profilePub_) {
      std_msgs::msg::Float32MultiArray arr;
      profilePub_->publish(arr);
    }

    if (cfg.publishPoints && pointsRawPub_ && pointsCleanPub_) {
      geometry_msgs::msg::PoseArray emptyPoints;
      emptyPoints.header.frame_id = cfg.mapFrame;
      emptyPoints.header.stamp = stamp;
      pointsRawPub_->publish(emptyPoints);
      pointsCleanPub_->publish(emptyPoints);
    }

    if (cfg.publishMeta && metaPub_) {
      json meta;
      meta["route_id"] = routeId;
      meta["cleared"] = true;
      meta["stamp_ns"] = stamp.nanoseconds();

      std_msgs::msg::String msg;
      msg.data = meta.dump();
      metaPub_->publish(msg);
    }
  }

  void onRouteJson(const std_msgs::msg::String::SharedPtr msg) {
    if (!msg) return;
    const std::string &s = msg->data;
    if (s.empty()) return;

    {
      std::lock_guard<std::mutex> lk(queueMutex_);
      latestJson_ = s;
      latestSeq_++;
    }
    cv_.notify_one();
  }

  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!msg) return;
    if (msg->status.status < 0) return;
    if (!isValidLatLon(msg->latitude, msg->longitude)) return;

    double covarianceThresholdM2 = 0.0;
    bool autoDatumFromFix = false;
    {
      std::lock_guard<std::mutex> lk(cfgMutex_);
      covarianceThresholdM2 = cfg_.datumCovarianceThresholdM2;
      autoDatumFromFix = cfg_.autoDatumFromFix;
    }

    const double horizontalCovarianceM2 = horizontalCovarianceMetricM2(*msg);
    const bool covarianceOk = std::isfinite(horizontalCovarianceM2) && horizontalCovarianceM2 <= covarianceThresholdM2;

    FixSnapshot fix;
    fix.lat = msg->latitude;
    fix.lon = msg->longitude;
    fix.alt = std::isfinite(msg->altitude) ? msg->altitude : 0.0;
    fix.stamp = this->now();
    fix.valid = true;

    {
      std::lock_guard<std::mutex> lk(fixMutex_);
      latestFix_ = fix;
    }

    if (autoDatumFromFix && covarianceOk) {
      resolveDatum(fix.lat, fix.lon, fix.alt, "fix", false);
    }
  }

  bool waitForDatumReady() {
    std::unique_lock<std::mutex> lk(datumMutex_);
    datumCv_.wait(lk, [this]() { return datumReady_ || datumShutdown_; });
    return datumReady_;
  }

  bool getRecentFix(FixSnapshot &out, double maxAgeS) {
    {
      std::lock_guard<std::mutex> lk(fixMutex_);
      out = latestFix_;
    }

    if (!out.valid) return false;
    if (maxAgeS <= 0.0) return true;
    return (this->now() - out.stamp).seconds() <= maxAgeS;
  }

  DatumFallback selectFallbackDatum(const FixSnapshot &fix) const {
    if (!fix.valid) return kDatumFallbacks.front();

    const DatumFallback *best = &kDatumFallbacks.front();
    double bestDist = std::numeric_limits<double>::infinity();
    for (const auto &candidate : kDatumFallbacks) {
      const double d = haversineDistanceM(fix.lat, fix.lon, candidate.lat, candidate.lon);
      if (d < bestDist) {
        bestDist = d;
        best = &candidate;
      }
    }
    return *best;
  }

  void resolveDatum(double lat, double lon, double alt, const std::string &source, bool usedFallback) {
    {
      std::lock_guard<std::mutex> lk(cfgMutex_);
      if (localCartesian_) return;
      cfg_.datumLat = lat;
      cfg_.datumLon = lon;
      cfg_.datumAlt = alt;
      localCartesian_ = std::make_shared<GeographicLib::LocalCartesian>(lat, lon, alt);
      datumSource_ = source;
      datumUsedFallback_ = usedFallback;
    }

    {
      std::lock_guard<std::mutex> lk(datumMutex_);
      if (datumReady_) return;
      datumReady_ = true;
    }

    if (datumTimer_) datumTimer_->cancel();
    datumCv_.notify_all();

    if (usedFallback) {
      RCLCPP_WARN(
        this->get_logger(),
        "datum fallback selected from %s: lat=%.9f lon=%.9f alt=%.3f",
        source.c_str(), lat, lon, alt);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "datum resolved from %s: lat=%.9f lon=%.9f alt=%.3f",
        source.c_str(), lat, lon, alt);
    }
  }

  void onDatumTimer() {
    Config cfg;
    {
      std::lock_guard<std::mutex> lk(cfgMutex_);
      cfg = cfg_;
    }

    if (!cfg.autoDatumFromFix) {
      if (datumTimer_) datumTimer_->cancel();
      return;
    }

    {
      std::lock_guard<std::mutex> lk(datumMutex_);
      if (datumReady_ || datumShutdown_) {
        if (datumTimer_) datumTimer_->cancel();
        return;
      }
    }

    const double elapsedS = (this->now() - datumStartTime_).seconds();
    if (elapsedS < cfg.datumWaitTimeoutS) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "waiting for %s covariance <= %.2f m^2 before setting datum",
        cfg.fixTopic.c_str(), cfg.datumCovarianceThresholdM2);
      return;
    }

    FixSnapshot fix;
    {
      std::lock_guard<std::mutex> lk(fixMutex_);
      fix = latestFix_;
    }

    const auto fallback = selectFallbackDatum(fix);
    resolveDatum(
      fallback.lat, fallback.lon, 0.0,
      fix.valid ? "nearest_fallback_to_fix" : "default_fallback",
      true);
  }

  void workerLoop() {
    uint64_t lastSeq = 0;

    for (;;) {
      std::string payload;

      {
        std::unique_lock<std::mutex> lk(queueMutex_);
        cv_.wait(lk, [&]() { return !workerRunning_ || latestSeq_ != lastSeq; });
        if (!workerRunning_) return;
        payload = latestJson_;
        lastSeq = latestSeq_;
      }

      processPayload(payload);
    }
  }

  void processPayload(const std::string &s) {
    const auto t0 = std::chrono::steady_clock::now();
    if (!waitForDatumReady()) return;

    Config cfg;
    std::shared_ptr<GeographicLib::LocalCartesian> lc;
    {
      std::lock_guard<std::mutex> lk(cfgMutex_);
      cfg = cfg_;
      lc = localCartesian_;
    }
    if (!lc) {
      RCLCPP_ERROR(this->get_logger(), "datum unavailable, dropping route payload");
      return;
    }

    json j;
    try {
      j = json::parse(s);
    } catch (const std::exception &e) {
      const std::string preview = s.substr(0, std::min<size_t>(120, s.size()));
      RCLCPP_ERROR(this->get_logger(), "failed to parse route json: %s, preview: %s", e.what(), preview.c_str());
      return;
    }

    json waypoints;
    std::string routeId;
    if (j.is_object() && j.contains("waypoints") && j["waypoints"].is_array()) {
      waypoints = j["waypoints"];
      if (j.contains("route_id") && j["route_id"].is_string()) routeId = j["route_id"].get<std::string>();
    } else if (j.is_array()) {
      waypoints = j;
    } else {
      RCLCPP_ERROR(this->get_logger(), "json must be an array of points or {waypoints:[...]}");
      return;
    }

    if (waypoints.empty()) {
      if (haveLastRouteState_ && routeId == lastRouteId_ && lastRouteSignature_.empty()) return;

      publishClearOutputs(cfg, routeId);
      lastRouteId_ = routeId;
      lastRouteSignature_.clear();
      haveLastRouteState_ = true;
      return;
    }

    const std::string tag = routeId.empty() ? "" : ("route_id=" + routeId + " ");

    if (waypoints.size() == 1) {
      FixSnapshot fix;
      if (!getRecentFix(fix, cfg.currentFixMaxAgeS)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "%ssingle-waypoint route requires recent %s within %.1fs",
          tag.c_str(), cfg.fixTopic.c_str(), cfg.currentFixMaxAgeS);
        return;
      }

      json currentWp;
      currentWp["latitude"] = fix.lat;
      currentWp["longitude"] = fix.lon;
      currentWp["altitude"] = fix.alt;
      json prepended = json::array();
      prepended.push_back(currentWp);
      prepended.push_back(waypoints.front());
      waypoints = std::move(prepended);
      RCLCPP_INFO(this->get_logger(), "%sprepended current %s to single-waypoint route", tag.c_str(), cfg.fixTopic.c_str());
    }

    if (cfg.maxInputPoints > 0 && static_cast<int>(waypoints.size()) > cfg.maxInputPoints) {
      RCLCPP_ERROR(this->get_logger(), "input has %zu points, exceeds max_input_points=%d", waypoints.size(), cfg.maxInputPoints);
      return;
    }

    std::vector<XY> rawPts;
    rawPts.reserve(waypoints.size());
    std::string normalizedRouteSignature;
    normalizedRouteSignature.reserve(waypoints.size() * 48);

    size_t skipped = 0;
    size_t invalid = 0;

    for (const auto &wp : waypoints) {
      if (!wp.is_object()) {
        skipped++;
        continue;
      }

      double lat = 0.0;
      double lon = 0.0;
      double alt = 0.0;

      bool ok = readNumber(wp, "latitude", lat) && readNumber(wp, "longitude", lon);
      if (!ok) {
        double lonTmp = 0.0;
        ok = readNumber(wp, "lat", lat) && (readNumber(wp, "lon", lonTmp));
        if (ok) lon = lonTmp;
      }

      if (!ok) {
        skipped++;
        continue;
      }

      if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        invalid++;
        continue;
      }

      double altTmp = 0.0;
      if (readNumber(wp, "altitude", altTmp) || readNumber(wp, "alt", altTmp)) alt = altTmp;

      normalizedRouteSignature += std::to_string(std::llround(lat * 10000000.0));
      normalizedRouteSignature.push_back(',');
      normalizedRouteSignature += std::to_string(std::llround(lon * 10000000.0));
      normalizedRouteSignature.push_back(',');
      normalizedRouteSignature += std::to_string(std::llround(alt * 100.0));
      normalizedRouteSignature.push_back(';');

      double x = 0.0, y = 0.0, z = 0.0;
      lc->Forward(lat, lon, alt, x, y, z);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        invalid++;
        continue;
      }

      const XY xy{x, y};
      rawPts.push_back(xy);
    }

    if (rawPts.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "need at least 2 valid waypoints");
      return;
    }

    if (skipped > 0 || invalid > 0) RCLCPP_WARN(this->get_logger(), "%sskipped=%zu invalid=%zu", tag.c_str(), skipped, invalid);

    if (haveLastRouteState_ && routeId == lastRouteId_ && normalizedRouteSignature == lastRouteSignature_) return;

    const auto stamp = this->now();
    const int yawLook = std::max(1, cfg.yawLookaheadPts);
    const int yawSmooth = std::max(0, cfg.yawSmoothWindowPts);

    {
      const auto rawYaws = computeYaws(rawPts, yawLook, 0);
      const auto rawMsg = makePathMsgWithYaws(rawPts, rawYaws, cfg.mapFrame, stamp);
      rawPub_->publish(rawMsg);
    }

    std::vector<XY> pts = rawPts;

    pts = dedupeConsecutiveKeepEnds(pts, cfg.minPointDistM);

    if (cfg.enableRemoveKinks)
      pts = removeKinksGuarded(pts, cfg.kinkShortM, cfg.uTurnDeg, cfg.collinearDistM, cfg.kinkShortcutRatioMin, cfg.kinkShortcutMaxDeltaM);

    if (cfg.enableConstrainedRdp && cfg.rdpEpsilonM > 1e-6 && pts.size() >= 3) {
      const auto protectedIdx = computeProtectedIndices(pts, cfg.protectTurnDeg, cfg.loopGuardDistM, cfg.loopGuardWindowPts, cfg.loopGuardAdjacencySkipPts);
      pts = constrainedRdp(pts, cfg.rdpEpsilonM, protectedIdx);
    }

    if (cfg.resampleSpacingM > 1e-6 && pts.size() >= 2) pts = resampleBySpacing(pts, cfg.resampleSpacingM);

    if (pts.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "route collapsed after cleaning");
      return;
    }

    const double rawLen = pathLengthM(rawPts);
    const double cleanLen = pathLengthM(pts);
    const double ratio = (rawLen > 1e-6) ? (cleanLen / rawLen) : 1.0;
    const bool cleanOk = (cfg.cleanLenRatioAbortMin <= 1e-6) || (ratio >= cfg.cleanLenRatioAbortMin);
    const std::vector<XY> &pubPts = cleanOk ? pts : rawPts;

    if (!cleanOk)
      RCLCPP_ERROR(this->get_logger(),
                   "%sclean_len/raw_len ratio %.3f below abort %.3f, "
                   "publishing raw as clean",
                   tag.c_str(), ratio, cfg.cleanLenRatioAbortMin);

    const auto yaws = computeYaws(pubPts, yawLook, yawSmooth);
    const auto cleanMsg = makePathMsgWithYaws(pubPts, yaws, cfg.mapFrame, stamp);
    cleanPub_->publish(cleanMsg);

    const size_t pubSize = pubPts.size();
    const bool wantProfile = cfg.publishProfile && profilePub_;
    const bool wantMeta = cfg.publishMeta && metaPub_;
    const bool wantPoints = cfg.publishPoints && pointsRawPub_ && pointsCleanPub_;

    std::vector<double> sM;
    if (wantProfile) {
      sM.assign(pubSize, 0.0);
      for (size_t i = 1; i < pubSize; i++)
        sM[i] = sM[i - 1] + distXy(pubPts[i - 1], pubPts[i]);
    }

    if (wantProfile) {
      std_msgs::msg::Float32MultiArray arr;
      arr.layout.dim.resize(2);
      arr.layout.dim[0].label = "points";
      arr.layout.dim[0].size = pubSize;
      arr.layout.dim[0].stride = pubSize * 3;
      arr.layout.dim[1].label = "s_k_v";
      arr.layout.dim[1].size = 3;
      arr.layout.dim[1].stride = 3;
      arr.layout.data_offset = 0;

      arr.data.reserve(pubSize * 3);
      for (size_t i = 0; i < pubSize; i++) {
        double k = 0.0;
        if (i > 0 && i + 1 < pubSize) k = signedCurvature(pubPts[i - 1], pubPts[i], pubPts[i + 1]);
        const double v = speedFromCurvature(k, cfg.vMinMps, cfg.vMaxMps, cfg.aLatMaxMps2, cfg.kDeadband);

        arr.data.push_back(static_cast<float>(sM[i]));
        arr.data.push_back(static_cast<float>(k));
        arr.data.push_back(static_cast<float>(v));
      }

      profilePub_->publish(arr);
    }

    if (wantPoints) {
      pointsRawPub_->publish(makePoseArray(rawPts, cfg.mapFrame, stamp));
      pointsCleanPub_->publish(makePoseArray(pubPts, cfg.mapFrame, stamp));
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();

    if (ratio < cfg.cleanLenRatioWarnMin) {
      RCLCPP_WARN(this->get_logger(),
                  "%sclean_len/raw_len ratio low: %.3f raw_pts=%zu "
                  "clean_pts=%zu pub_pts=%zu ms=%.2f",
                  tag.c_str(), ratio, rawPts.size(), pts.size(), pubPts.size(), ms);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "%sraw_pts=%zu raw_len_m=%.2f clean_pts=%zu clean_len_m=%.2f "
                  "pub_pts=%zu ms=%.2f",
                  tag.c_str(), rawPts.size(), rawLen, pts.size(), cleanLen, pubPts.size(), ms);
    }

    if (wantMeta) {
      json meta;
      meta["route_id"] = routeId;
      meta["raw_pts"] = rawPts.size();
      meta["clean_pts"] = pts.size();
      meta["published_pts"] = pubPts.size();
      meta["raw_len_m"] = rawLen;
      meta["clean_len_m"] = cleanLen;
      meta["published_len_m"] = cleanOk ? cleanLen : rawLen;
      meta["clean_over_raw"] = ratio;
      meta["processing_ms"] = ms;
      meta["stamp_ns"] = stamp.nanoseconds();
      meta["datum_lat"] = cfg.datumLat;
      meta["datum_lon"] = cfg.datumLon;
      meta["datum_alt"] = cfg.datumAlt;
      meta["datum_source"] = datumSource_;
      meta["datum_used_fallback"] = datumUsedFallback_;
      meta["skipped_waypoints"] = skipped;
      meta["invalid_waypoints"] = invalid;
      meta["clean_aborted"] = !cleanOk;
      meta["published_is_raw"] = !cleanOk;

      std_msgs::msg::String m;
      m.data = meta.dump();
      metaPub_->publish(m);
    }

    lastRouteId_ = routeId;
    lastRouteSignature_ = normalizedRouteSignature;
    haveLastRouteState_ = true;
  }

  std::mutex cfgMutex_;
  Config cfg_;
  std::shared_ptr<GeographicLib::LocalCartesian> localCartesian_;
  std::string datumSource_{"parameters"};
  bool datumUsedFallback_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramsCb_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rawPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr cleanPub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr profilePub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metaPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pointsRawPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pointsCleanPub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr routeSub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fixSub_;
  rclcpp::TimerBase::SharedPtr datumTimer_;

  std::mutex queueMutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool workerRunning_{false};
  std::string latestJson_;
  uint64_t latestSeq_{0};
  bool haveLastRouteState_{false};
  std::string lastRouteId_;
  std::string lastRouteSignature_;

  std::mutex datumMutex_;
  std::condition_variable datumCv_;
  bool datumReady_{true};
  bool datumShutdown_{false};
  rclcpp::Time datumStartTime_;

  std::mutex fixMutex_;
  FixSnapshot latestFix_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPathManager>());
  rclcpp::shutdown();
  return 0;
}
