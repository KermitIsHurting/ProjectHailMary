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

        // TTL 32 reaches LAN-wide ATAK endpoints without crossing the site router
        unsigned char ttl = 32;
        setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

        std::memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port = htons(6969);
        inet_aton("239.2.3.1", &dest_.sin_addr);

        last_full_send_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(), "CoT publisher node ready (239.2.3.1:6969)");
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
        std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc);
        return buf;
    }

    std::string buildCotEvent(const cuas_msgs::msg::ThreatReport& report)
    {
        std::string now_ts = isoTimestamp();
        std::string stale_ts = isoTimestamp(30.0);

        std::ostringstream xml;
        xml << "<?xml version=\"1.0\"?>"
            << "<event version=\"2.0\""
            << " uid=\"CUAS-TRACK-" << report.track_id << "\""
            << " type=\"a-u-G\""
            << " time=\"" << now_ts << "\""
            << " start=\"" << now_ts << "\""
            << " stale=\"" << stale_ts << "\""
            << " how=\"m-g\">"
            << "<point lat=\"0.0\" lon=\"0.0\" hae=\"0.0\" ce=\"10.0\" le=\"10.0\"/>"
            << "<detail>"
            << "<track speed=\"" << std::fixed << std::setprecision(2) << report.velocity_mps << "\""
            << " course=\"" << std::fixed << std::setprecision(1)
            << std::atan2(report.position_x_m, report.position_y_m) * 180.0F / static_cast<float32_t>(M_PI) << "\"/>"
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
        sendto(sock_, xml.data(), xml.size(), 0,
               reinterpret_cast<const sockaddr*>(&dest_), sizeof(dest_));
    }

    void threatCallback(const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
    {
        auto now = std::chrono::steady_clock::now();
        float64_t elapsed_s = std::chrono::duration<float64_t>(now - last_threatening_send_).count();
        float64_t full_elapsed_s = std::chrono::duration<float64_t>(now - last_full_send_).count();

        // Threatening events fire at 1 Hz; a full sweep fires every 5 s
        if (elapsed_s >= 1.0) {
            for (const auto& report : msg->reports) {
                if (report.escalation_state == "THREATENING" ||
                    report.escalation_state == "ENGAGED") {
                    sendUdp(buildCotEvent(report));
                }
            }
            last_threatening_send_ = now;
        }

        if (full_elapsed_s >= 5.0) {
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
