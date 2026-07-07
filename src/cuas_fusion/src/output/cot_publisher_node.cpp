// @file cot_publisher_node.cpp
// @brief ROS 2 node that emits Cursor-on-Target XML events over UDP multicast.
#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>

#include "cuas_fusion/common/fixed_types.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdio>

namespace cuas {

namespace {

// External interface parameters — see docs/ICD.md §4.
constexpr const char* kCotMulticastAddr   = "239.2.3.1";
constexpr uint16_t    kCotMulticastPort   = 6969U;
// TTL 32 reaches LAN-wide ATAK endpoints without crossing the site router.
constexpr uint8_t     kCotMulticastTtl    = 32U;
constexpr float64_t   kCotStaleSeconds    = 30.0;
constexpr float64_t   kThreatPeriodSec    = 1.0;
constexpr float64_t   kFullSweepPeriodSec = 5.0;

} // namespace

class CotPublisherNode : public rclcpp::Node
{
public:
    CotPublisherNode()
    : Node("cot_publisher_node")
    {
        sub_ = create_subscription<cuas_msgs::msg::ThreatReportArray>(
            "/threat/reports", 5,
            std::bind(&CotPublisherNode::threatCallback, this, std::placeholders::_1));

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(get_logger(), "CoT: socket() failed: %s", strerror(errno));
            return;
        }

        const unsigned char ttl = kCotMulticastTtl;
        if (setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_TTL,
                       &ttl, sizeof(ttl)) != 0) {
            RCLCPP_WARN(get_logger(),
                "CoT: setsockopt(IP_MULTICAST_TTL) failed: %s — kernel "
                "default TTL applies", strerror(errno));
        }

        std::memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port = htons(kCotMulticastPort);
        if (inet_aton(kCotMulticastAddr, &dest_.sin_addr) == 0) {
            // An unparsed address leaves sin_addr zeroed and every event
            // would go to 0.0.0.0 — disable output instead.
            RCLCPP_FATAL(get_logger(),
                "CoT: invalid multicast address %s — disabling CoT output",
                kCotMulticastAddr);
            (void)close(sock_);
            sock_ = -1;
            return;
        }

        last_full_send_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(), "CoT publisher node ready (%s:%u)",
                    kCotMulticastAddr,
                    static_cast<uint32_t>(kCotMulticastPort));
    }

    ~CotPublisherNode() override
    {
        if (sock_ >= 0) {
            (void)close(sock_);
        }
    }

private:
    static std::string isoTimestamp(float64_t offset_s = 0.0)
    {
        auto now = std::chrono::system_clock::now();
        now += std::chrono::milliseconds(static_cast<int64_t>(offset_s * 1000.0));
        auto tt = std::chrono::system_clock::to_time_t(now);
        std::tm utc{};
        gmtime_r(&tt, &utc);
        char buf[32];
        if (std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc) == 0U) {
            // The 20-char format always fits, but on a zero return the
            // buffer contents are unspecified — never construct from it.
            return "1970-01-01T00:00:00Z";
        }
        return buf;
    }

    std::string buildCotEvent(const cuas_msgs::msg::ThreatReport& report)
    {
        std::string now_ts = isoTimestamp();
        std::string stale_ts = isoTimestamp(kCotStaleSeconds);

        // CoT course is 0..360 clockwise from north; atan2 returns -180..180.
        float64_t course_deg = std::atan2(
            static_cast<float64_t>(report.position_x_m),
            static_cast<float64_t>(report.position_y_m)) * 180.0 / M_PI;
        if (course_deg < 0.0) {
            course_deg += 360.0;
        }

        std::ostringstream xml;
        xml << "<?xml version=\"1.0\"?>"
            << "<event version=\"2.0\""
            << " uid=\"CUAS-TRACK-" << report.track_id << "\""
            << " type=\"a-u-G\""
            << " time=\"" << now_ts << "\""
            << " start=\"" << now_ts << "\""
            << " stale=\"" << stale_ts << "\""
            << " how=\"m-g\">"
            // GEOREFERENCING NOT IMPLEMENTED: no GNSS/pose source exists, so
            // lat/lon/hae are hardcoded to 0,0 ("null island") and ATAK plots
            // every event there. Sensor-relative geometry lives only in
            // <track course/speed> and <remarks>. Documented in docs/ICD.md §4.
            << "<point lat=\"0.0\" lon=\"0.0\" hae=\"0.0\" ce=\"10.0\" le=\"10.0\"/>"
            << "<detail>"
            << "<track speed=\"" << std::fixed << std::setprecision(2) << report.velocity_mps << "\""
            << " course=\"" << std::fixed << std::setprecision(1)
            << course_deg << "\"/>"
            << "<status readiness=\"true\"/>"
            << "<remarks>ThreatLevel:" << report.threat_level
            << " Quality:" << std::fixed << std::setprecision(2) << report.quality_score
            << " Class:" << report.class_label
            << " Esc:" << report.escalation_state
            << "</remarks>"
            << "</detail>"
            << "</event>";
        return xml.str();
    }

    void sendUdp(const std::string& xml)
    {
        if (sock_ < 0) {
            return;
        }
        const ssize_t sent = sendto(sock_, xml.data(), xml.size(), 0,
               reinterpret_cast<const sockaddr*>(&dest_), sizeof(dest_));
        if (sent < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "CoT: sendto failed: %s", strerror(errno));
        }
    }

    void threatCallback(const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
    {
        auto now = std::chrono::steady_clock::now();
        float64_t elapsed_s = std::chrono::duration<float64_t>(now - last_threatening_send_).count();
        float64_t full_elapsed_s = std::chrono::duration<float64_t>(now - last_full_send_).count();

        // Threatening events fire at 1 Hz; a full sweep fires every 5 s
        if (elapsed_s >= kThreatPeriodSec) {
            for (const auto& report : msg->reports) {
                if (report.escalation_state == "THREATENING" ||
                    report.escalation_state == "ENGAGED") {
                    sendUdp(buildCotEvent(report));
                }
            }
            last_threatening_send_ = now;
        }

        if (full_elapsed_s >= kFullSweepPeriodSec) {
            for (const auto& report : msg->reports) {
                sendUdp(buildCotEvent(report));
            }
            last_full_send_ = now;
        }
    }

    rclcpp::Subscription<cuas_msgs::msg::ThreatReportArray>::SharedPtr sub_;
    int32_t sock_ = -1;
    struct sockaddr_in dest_{};
    std::chrono::steady_clock::time_point last_threatening_send_{};
    std::chrono::steady_clock::time_point last_full_send_{};
};

} // namespace cuas

// Single sanctioned exception boundary (DEV-001): owned code never
// throws, but rclcpp/rmw, parameter access, and bad_alloc can. Without
// this handler a library throw becomes std::terminate with no fault
// record, invisible to the health monitor. Catch by const ref per
// MISRA C++:2023 18.3.2.
int main(int argc, char** argv)
{
    int exit_code = 0;
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<cuas::CotPublisherNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in CotPublisherNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in CotPublisherNode\n");
        exit_code = 1;
    }
    return exit_code;
}
