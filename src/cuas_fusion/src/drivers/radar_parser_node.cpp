// @file radar_parser_node.cpp
// @brief IWR6843ISK frame parser with DBSCAN clustering and ROS publishing.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <string>
#include <thread>
#include <utility>
#include <cstdio>

namespace cuas {

static constexpr float32_t CLUTTER_VEL_THRESH  = 0.1F;
static constexpr float32_t MAX_RANGE_M         = 15.0F;
static constexpr float32_t DBSCAN_EPS          = 0.7F;
static constexpr int32_t   DBSCAN_MIN_PTS      = 2;

static constexpr std::array<uint8_t, 8> MAGIC_WORD = {
    0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07
};

static constexpr std::size_t HEADER_SIZE              = 40U;
static constexpr uint32_t    TLV_TYPE_DETECTED_POINTS = 1U;
static constexpr uint32_t    MAX_PACKET_BYTES         = 65536U;

#pragma pack(push, 1)

// SDK 3.4+ moved numTLVs before subFrameNumber
struct FrameHeader {
    uint8_t  magic[8];
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
};
static_assert(sizeof(FrameHeader) == HEADER_SIZE, "FrameHeader must be exactly 40 bytes");

struct TlvHeader {
    uint32_t type;
    uint32_t length;  // payload length excludes this header
};
static_assert(sizeof(TlvHeader) == 8U, "TlvHeader must be exactly 8 bytes");

struct DetectedPoint {
    float32_t x;
    float32_t y;
    float32_t z;
    float32_t doppler;  // positive = approaching
};
static_assert(sizeof(DetectedPoint) == 16U, "DetectedPoint must be exactly 16 bytes");

#pragma pack(pop)

static FixedVector<DetectedPoint, TRACK_MAX_TRACKS> filterPoints(
    const FixedVector<DetectedPoint, TRACK_MAX_TRACKS>& raw)
{
    FixedVector<DetectedPoint, TRACK_MAX_TRACKS> out;
    for (uint32_t i = 0U; i < raw.size(); ++i) {
        const DetectedPoint& p = raw[i];
        const float32_t range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (range > MAX_RANGE_M) {
            continue;
        }
        if (std::abs(p.doppler) < CLUTTER_VEL_THRESH) {
            continue;
        }
        if (!out.push_back(p)) {
            break;
        }
    }
    return out;
}

static float32_t pointDist(const DetectedPoint& a, const DetectedPoint& b)
{
    const float32_t dx = a.x - b.x;
    const float32_t dy = a.y - b.y;
    const float32_t dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static FixedVector<DetectedPoint, TRACK_MAX_TRACKS> dbscanCluster(
    const FixedVector<DetectedPoint, TRACK_MAX_TRACKS>& pts)
{
    const int32_t n = static_cast<int32_t>(pts.size());
    if (n == 0) {
        return {};
    }

    FixedVector<int32_t, TRACK_MAX_TRACKS> label;
    for (int32_t i = 0; i < n; ++i) {
        (void)label.push_back(-1);
    }
    int32_t cluster_id = 0;

    for (int32_t i = 0; i < n; ++i) {
        if (label[static_cast<uint32_t>(i)] != -1) {
            continue;
        }

        FixedVector<int32_t, TRACK_MAX_TRACKS> neighbors;
        for (int32_t j = 0; j < n; ++j) {
            if (pointDist(pts[static_cast<uint32_t>(i)],
                          pts[static_cast<uint32_t>(j)]) <= DBSCAN_EPS) {
                (void)neighbors.push_back(j);
            }
        }

        if (static_cast<int32_t>(neighbors.size()) < DBSCAN_MIN_PTS) {
            label[static_cast<uint32_t>(i)] = 0;
            continue;
        }

        ++cluster_id;
        label[static_cast<uint32_t>(i)] = cluster_id;

        for (uint32_t qi = 0U; qi < neighbors.size(); ++qi) {
            const int32_t q = neighbors[qi];
            if (label[static_cast<uint32_t>(q)] == 0) {
                label[static_cast<uint32_t>(q)] = cluster_id;
            }
            if (label[static_cast<uint32_t>(q)] != -1) {
                continue;
            }
            label[static_cast<uint32_t>(q)] = cluster_id;

            for (int32_t j = 0; j < n; ++j) {
                if (pointDist(pts[static_cast<uint32_t>(q)],
                              pts[static_cast<uint32_t>(j)]) <= DBSCAN_EPS) {
                    bool already = false;
                    for (uint32_t k = 0U; k < neighbors.size(); ++k) {
                        if (neighbors[k] == j) {
                            already = true;
                            break;
                        }
                    }
                    if (!already) {
                        (void)neighbors.push_back(j);
                    }
                }
            }
        }
    }

    FixedVector<DetectedPoint, TRACK_MAX_TRACKS> centroids;
    for (int32_t c = 1; c <= cluster_id; ++c) {
        float32_t sx           = 0.0F;
        float32_t sy           = 0.0F;
        float32_t sz           = 0.0F;
        float32_t sum_weight   = 0.0F;
        float32_t max_abs_dop  = -1.0F;
        float32_t dominant_dop = 0.0F;
        int32_t   cnt          = 0;
        for (int32_t i = 0; i < n; ++i) {
            if (label[static_cast<uint32_t>(i)] == c) {
                const DetectedPoint& p = pts[static_cast<uint32_t>(i)];
                const float32_t abs_dop = std::abs(p.doppler);
                const float32_t w       = abs_dop + kDopplerWeightFloor;
                sx         += w * p.x;
                sy         += w * p.y;
                sz         += w * p.z;
                sum_weight += w;
                if (abs_dop > max_abs_dop) {
                    max_abs_dop  = abs_dop;
                    dominant_dop = p.doppler;
                }
                ++cnt;
            }
        }
        if (cnt > 0 && sum_weight > 0.0F) {
            (void)centroids.push_back({sx / sum_weight,
                                       sy / sum_weight,
                                       sz / sum_weight,
                                       dominant_dop});
        }
    }

    if (centroids.empty()) {
        for (int32_t i = 0; i < n; ++i) {
            if (label[static_cast<uint32_t>(i)] == 0) {
                (void)centroids.push_back(pts[static_cast<uint32_t>(i)]);
            }
        }
    }

    return centroids;
}

class RadarParserNode : public rclcpp::Node
{
public:
    RadarParserNode()
    : Node("radar_parser_node"), fd_(-1), running_(false)
    {
        declare_parameter("data_port", std::string("/dev/radar_data"));
        declare_parameter("config_port", std::string("/dev/radar_config"));

        data_port_   = get_parameter("data_port").as_string();
        config_port_ = get_parameter("config_port").as_string();

        if (access(data_port_.c_str(), F_OK) != 0) {
            RCLCPP_WARN(get_logger(),
                "Port %s not found, attempting auto-detection...",
                data_port_.c_str());
            auto ports = detectRadarPorts();
            if (!ports.first.empty()) {
                data_port_   = ports.first;
                config_port_ = ports.second;
                RCLCPP_INFO(get_logger(),
                    "Auto-detected: data=%s  config=%s",
                    data_port_.c_str(), config_port_.c_str());
            }
        }

        port_ = data_port_;
        pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/radar/detections", 10);

        if (!open_port()) {
            RCLCPP_FATAL(get_logger(),
                "Failed to open radar serial port %s — shutting down",
                port_.c_str());
            return;
        }

        running_ = true;
        parse_thread_ = std::thread(&RadarParserNode::parse_loop, this);

        RCLCPP_INFO(get_logger(), "Radar parser started — data_port: %s  config_port: %s",
                     data_port_.c_str(), config_port_.c_str());
    }

    ~RadarParserNode() override
    {
        running_ = false;
        if (parse_thread_.joinable()) {
            parse_thread_.join();
        }
        if (fd_ >= 0) {
            (void)close(fd_);
            fd_ = -1;
        }
    }

private:
    std::pair<std::string, std::string> detectRadarPorts()
    {
        FixedVector<std::string, 10U> radar_ports;
        for (int32_t i = 0; i <= 9; ++i) {
            const std::string port = "/dev/ttyUSB" + std::to_string(i);
            if (access(port.c_str(), F_OK) != 0) {
                continue;
            }
            const std::string base = "/sys/class/tty/ttyUSB"
                               + std::to_string(i) + "/device/../";
            std::ifstream vid_file(base + "idVendor");
            std::ifstream pid_file(base + "idProduct");
            std::string vid;
            std::string pid;
            // CP210x on the IWR6843ISK reports VID=10c4 PID=ea70
            if ((vid_file >> vid) && (pid_file >> pid)
                && vid == "10c4" && pid == "ea70") {
                (void)radar_ports.push_back(port);
            }
        }
        if (radar_ports.size() < 2U) {
            RCLCPP_ERROR(get_logger(),
                "Radar auto-detect failed: found %u CP210x ports, need 2",
                radar_ports.size());
            return {"", ""};
        }
        std::sort(radar_ports.begin(), radar_ports.end());
        // Lower ttyUSB index is the data port, higher is the config port
        return {radar_ports[0], radar_ports[1]};
    }

    bool open_port()
    {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(),
                "Cannot open serial port %s: %s",
                port_.c_str(), strerror(errno));
            return false;
        }

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(get_logger(),
                "tcgetattr failed: %s", strerror(errno));
            return false;
        }

        cfmakeraw(&tty);

        if (cfsetispeed(&tty, B921600) != 0 || cfsetospeed(&tty, B921600) != 0) {
            RCLCPP_ERROR(get_logger(),
                "cfsetspeed B921600 failed — check kernel support");
            return false;
        }

        tty.c_cflag |=  (CLOCAL | CREAD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(get_logger(),
                "tcsetattr failed: %s", strerror(errno));
            return false;
        }

        (void)tcflush(fd_, TCIFLUSH);
        return true;
    }

    bool read_exact(uint8_t * buf, std::size_t n)
    {
        std::size_t total = 0U;
        while (total < n) {
            if (!running_) {
                return false;
            }
            const ssize_t r = ::read(fd_, buf + total, n - total);
            if (r < 0) {
                if (errno == EINTR) {
                    continue;
                }
                RCLCPP_ERROR(get_logger(), "Serial read error: %s", strerror(errno));
                return false;
            }
            total += static_cast<std::size_t>(r);
        }
        return true;
    }

    bool sync_to_magic()
    {
        std::size_t matched = 0U;
        while (running_) {
            uint8_t byte = 0U;
            if (!read_exact(&byte, 1U)) {
                return false;
            }

            if (byte == MAGIC_WORD[matched]) {
                ++matched;
                if (matched == MAGIC_WORD.size()) {
                    return true;
                }
            } else {
                // Mismatched byte might itself start a new magic sequence
                if (byte == MAGIC_WORD[0]) {
                    matched = 1U;
                } else {
                    matched = 0U;
                }
            }
        }
        return false;
    }

    static builtin_interfaces::msg::Time monotonic_stamp()
    {
        struct timespec ts{};
        (void)clock_gettime(CLOCK_MONOTONIC, &ts);
        builtin_interfaces::msg::Time t;
        t.sec     = static_cast<int32_t>(ts.tv_sec);
        t.nanosec = static_cast<uint32_t>(ts.tv_nsec);
        return t;
    }

    void parse_loop()
    {
        RCLCPP_INFO(get_logger(), "Parse thread running");

        while (running_) {
            if (!sync_to_magic()) {
                break;
            }

            uint8_t hdr_buf[HEADER_SIZE];
            std::memcpy(hdr_buf, MAGIC_WORD.data(), MAGIC_WORD.size());

            if (!read_exact(hdr_buf + MAGIC_WORD.size(),
                            HEADER_SIZE - MAGIC_WORD.size())) {
                break;
            }

            // memcpy instead of reinterpret_cast: no FrameHeader object lives in
            // hdr_buf, so a type-punned reference is strict-aliasing UB
            // (MISRA 8.2.5). Compiles to the same loads at -O2.
            FrameHeader hdr;
            std::memcpy(&hdr, hdr_buf, sizeof(hdr));

            if (hdr.totalPacketLen < HEADER_SIZE ||
                hdr.totalPacketLen > MAX_PACKET_BYTES)
            {
                RCLCPP_WARN(get_logger(),
                    "Frame %u: implausible totalPacketLen=%u — resyncing",
                    hdr.frameNumber, hdr.totalPacketLen);
                continue;
            }

            const uint32_t payload_len = hdr.totalPacketLen - HEADER_SIZE;

            // WHY: payload buffer size is unknown at compile time and bounded by
            // MAX_PACKET_BYTES; stack array avoids heap (DEV-005).
            std::array<uint8_t, MAX_PACKET_BYTES> payload{};
            if (payload_len > MAX_PACKET_BYTES) {
                break;
            }
            if (!read_exact(payload.data(), payload_len)) {
                break;
            }

            const auto stamp = monotonic_stamp();

            FixedVector<DetectedPoint, TRACK_MAX_TRACKS> points;
            std::size_t offset = 0U;

            for (uint32_t tlv_idx = 0U;
                 tlv_idx < hdr.numTLVs && offset + sizeof(TlvHeader) <= payload_len;
                 ++tlv_idx)
            {
                TlvHeader tlv;
                std::memcpy(&tlv, payload.data() + offset, sizeof(tlv));
                offset += sizeof(TlvHeader);

                if (tlv.type == TLV_TYPE_DETECTED_POINTS) {
                    const std::size_t num_pts = tlv.length / sizeof(DetectedPoint);
                    for (std::size_t i = 0U;
                         i < num_pts && offset + sizeof(DetectedPoint) <= payload_len;
                         ++i)
                    {
                        DetectedPoint pt;
                        std::memcpy(&pt, payload.data() + offset, sizeof(pt));
                        (void)points.push_back(pt);
                        offset += sizeof(DetectedPoint);
                    }
                } else {
                    if (offset + tlv.length > payload_len) {
                        break;
                    }
                    offset += tlv.length;
                }
            }

            const auto filtered = filterPoints(points);
            const auto clusters = dbscanCluster(filtered);

            RCLCPP_DEBUG(get_logger(),
                "[Frame %5u] raw=%u filtered=%u clusters=%u",
                hdr.frameNumber, points.size(), filtered.size(), clusters.size());

            if (!clusters.empty()) {
                publish_cloud(clusters, stamp);
            }
        }

        RCLCPP_INFO(get_logger(), "Parse thread exited");
    }

    void publish_cloud(const FixedVector<DetectedPoint, TRACK_MAX_TRACKS> & points,
                       const builtin_interfaces::msg::Time & stamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "radar_frame";
        msg.height          = 1U;
        msg.width           = static_cast<uint32_t>(points.size());
        msg.is_dense        = false;
        msg.is_bigendian    = false;

        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2Fields(
            4,
            "x",        1, sensor_msgs::msg::PointField::FLOAT32,
            "y",        1, sensor_msgs::msg::PointField::FLOAT32,
            "z",        1, sensor_msgs::msg::PointField::FLOAT32,
            "velocity", 1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_v(msg, "velocity");

        for (std::size_t i = 0U; i < points.size(); ++i) {
            *it_x = points[i].x;       ++it_x;
            *it_y = points[i].y;       ++it_y;
            *it_z = points[i].z;       ++it_z;
            *it_v = points[i].doppler; ++it_v;
        }

        pub_->publish(msg);
    }

    std::string  data_port_;
    std::string  config_port_;
    std::string  port_;
    int32_t      fd_;
    std::atomic<bool> running_;
    std::thread  parse_thread_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

} // namespace cuas

// Single sanctioned exception boundary (DEV-001): owned code never
// throws, but rclcpp/rmw, parameter access, and bad_alloc can. Without
// this handler a library throw becomes std::terminate with no fault
// record, invisible to the health monitor. Catch by const ref per
// MISRA C++:2023 18.3.2.
int main(int argc, char ** argv)
{
    int exit_code = 0;
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<cuas::RadarParserNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in RadarParserNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in RadarParserNode\n");
        exit_code = 1;
    }
    return exit_code;
}
