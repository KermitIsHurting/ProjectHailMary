// radar_parser_node.cpp
// Standalone ROS 2 node for the TI IWR6843ISK radar data port.
// Opens /dev/ttyUSB0 at 921600 baud, syncs to the TI mmWave SDK binary
// magic word, parses frame headers and TLV type-1 detected-point payloads,
// publishes sensor_msgs/PointCloud2 to /radar/detections, and prints each
// detection to stdout for hardware verification.
// Parsing runs in a dedicated std::thread; the main thread spins ROS 2.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <cstdio>
#include <cstdint>
#include <array>
#include <atomic>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace cuas {

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

static constexpr std::array<uint8_t, 8> MAGIC_WORD = {
    0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07
};

static constexpr size_t   HEADER_SIZE              = 40;
static constexpr uint32_t TLV_TYPE_DETECTED_POINTS = 1;
static constexpr uint32_t MAX_PACKET_BYTES         = 65536;

// ---------------------------------------------------------------------------
// Packed structs matching TI mmWave SDK 3.x binary output format
// ---------------------------------------------------------------------------

#pragma pack(push, 1)

// 40-byte frame header (SDK 3.4+ / 3.6 layout confirmed from live capture)
// SDK 3.4+ moved subFrameNumber to the END — numTLVs is at 32-35, subFrameNumber at 36-39.
struct FrameHeader {
    uint8_t  magic[8];          // bytes  0– 7: 0x02010403 06050807
    uint32_t version;           // bytes  8–11
    uint32_t totalPacketLen;    // bytes 12–15: full packet size including this header
    uint32_t platform;          // bytes 16–19
    uint32_t frameNumber;       // bytes 20–23
    uint32_t timeCpuCycles;     // bytes 24–27
    uint32_t numDetectedObj;    // bytes 28–31
    uint32_t numTLVs;           // bytes 32–35  (SDK 3.4+: numTLVs before subFrameNumber)
    uint32_t subFrameNumber;    // bytes 36–39
};
static_assert(sizeof(FrameHeader) == HEADER_SIZE, "FrameHeader must be exactly 40 bytes");

// 8-byte TLV tag-length prefix
struct TlvHeader {
    uint32_t type;    // TLV type identifier
    uint32_t length;  // payload length in bytes (excludes this header)
};

// One detected point from TLV type 1 (Cartesian, SDK DPIF_PointCloudCartesian)
struct DetectedPoint {
    float x;        // metres, positive = forward (broadside)
    float y;        // metres, positive = left
    float z;        // metres, positive = up
    float doppler;  // m/s, positive = approaching
};

#pragma pack(pop)

// ---------------------------------------------------------------------------
// RadarParserNode
// ---------------------------------------------------------------------------

class RadarParserNode : public rclcpp::Node
{
public:
    explicit RadarParserNode(const std::string & port = "/dev/ttyUSB2")
    : Node("radar_parser_node"), port_(port), fd_(-1), running_(false)
    {
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/radar/detections", 10);

        open_port();

        running_ = true;
        parse_thread_ = std::thread(&RadarParserNode::parse_loop, this);

        RCLCPP_INFO(get_logger(), "Radar parser started — port: %s", port_.c_str());
    }

    ~RadarParserNode() override
    {
        running_ = false;
        if (parse_thread_.joinable()) {
            parse_thread_.join();
        }
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

private:
    // -----------------------------------------------------------------------
    // Serial port initialisation (POSIX termios, 921600 8N1 raw)
    // -----------------------------------------------------------------------
    void open_port()
    {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            throw std::runtime_error(
                "Cannot open serial port " + port_ + ": " + strerror(errno));
        }

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            throw std::runtime_error(
                std::string("tcgetattr failed: ") + strerror(errno));
        }

        // Raw mode — no special character processing
        cfmakeraw(&tty);

        // 921600 baud
        if (cfsetispeed(&tty, B921600) != 0 || cfsetospeed(&tty, B921600) != 0) {
            throw std::runtime_error("cfsetspeed B921600 failed — check kernel support");
        }

        // 8N1, no flow control
        tty.c_cflag |=  (CLOCAL | CREAD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        // Blocking read: return as soon as ≥1 byte is available
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            throw std::runtime_error(
                std::string("tcsetattr failed: ") + strerror(errno));
        }

        tcflush(fd_, TCIFLUSH);
    }

    // -----------------------------------------------------------------------
    // Read exactly n bytes into buf; returns false if running_ goes false
    // -----------------------------------------------------------------------
    bool read_exact(uint8_t * buf, size_t n)
    {
        size_t total = 0;
        while (total < n) {
            if (!running_) return false;
            ssize_t r = ::read(fd_, buf + total, n - total);
            if (r < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(get_logger(), "Serial read error: %s", strerror(errno));
                return false;
            }
            total += static_cast<size_t>(r);
        }
        return true;
    }

    // -----------------------------------------------------------------------
    // Slide a byte at a time until the 8-byte magic word is matched
    // -----------------------------------------------------------------------
    bool sync_to_magic()
    {
        size_t matched = 0;
        while (running_) {
            uint8_t byte;
            if (!read_exact(&byte, 1)) return false;

            if (byte == MAGIC_WORD[matched]) {
                if (++matched == MAGIC_WORD.size()) return true;
            } else {
                // Restart match; check if this byte is the start of a new sequence
                matched = (byte == MAGIC_WORD[0]) ? 1 : 0;
            }
        }
        return false;
    }

    // -----------------------------------------------------------------------
    // CLOCK_MONOTONIC timestamp → builtin_interfaces::msg::Time
    // -----------------------------------------------------------------------
    static builtin_interfaces::msg::Time monotonic_stamp()
    {
        struct timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        builtin_interfaces::msg::Time t;
        t.sec    = static_cast<int32_t>(ts.tv_sec);
        t.nanosec = static_cast<uint32_t>(ts.tv_nsec);
        return t;
    }

    // -----------------------------------------------------------------------
    // Main parse loop — runs in parse_thread_
    // -----------------------------------------------------------------------
    void parse_loop()
    {
        RCLCPP_INFO(get_logger(), "Parse thread running");

        while (running_) {
            // ---- 1. synchronise to magic word ----
            if (!sync_to_magic()) break;

            // ---- 2. read the remaining 32 bytes of the 40-byte header ----
            //         (magic is already consumed, so we copy it in manually)
            uint8_t hdr_buf[HEADER_SIZE];
            std::memcpy(hdr_buf, MAGIC_WORD.data(), MAGIC_WORD.size());

            if (!read_exact(hdr_buf + MAGIC_WORD.size(),
                            HEADER_SIZE - MAGIC_WORD.size())) {
                break;
            }

            const auto & hdr = *reinterpret_cast<const FrameHeader *>(hdr_buf);

            // ---- 3. validate packet length ----
            if (hdr.totalPacketLen < HEADER_SIZE ||
                hdr.totalPacketLen > MAX_PACKET_BYTES)
            {
                RCLCPP_WARN(get_logger(),
                    "Frame %u: implausible totalPacketLen=%u — resyncing",
                    hdr.frameNumber, hdr.totalPacketLen);
                continue;
            }

            const uint32_t payload_len = hdr.totalPacketLen - HEADER_SIZE;

            // ---- 4. read TLV payload ----
            std::vector<uint8_t> payload(payload_len);
            if (!read_exact(payload.data(), payload_len)) break;

            // ---- 5. timestamp at last-byte-received (CLOCK_MONOTONIC) ----
            const auto stamp = monotonic_stamp();

            // ---- 6. parse TLVs ----
            std::vector<DetectedPoint> points;
            size_t offset = 0;

            for (uint32_t tlv_idx = 0;
                 tlv_idx < hdr.numTLVs && offset + sizeof(TlvHeader) <= payload_len;
                 ++tlv_idx)
            {
                const auto & tlv =
                    *reinterpret_cast<const TlvHeader *>(payload.data() + offset);
                offset += sizeof(TlvHeader);

                if (tlv.type == TLV_TYPE_DETECTED_POINTS) {
                    const size_t num_pts = tlv.length / sizeof(DetectedPoint);
                    for (size_t i = 0;
                         i < num_pts && offset + sizeof(DetectedPoint) <= payload_len;
                         ++i)
                    {
                        points.push_back(
                            *reinterpret_cast<const DetectedPoint *>(
                                payload.data() + offset));
                        offset += sizeof(DetectedPoint);
                    }
                } else {
                    // skip unknown TLV
                    if (offset + tlv.length > payload_len) break;
                    offset += tlv.length;
                }
            }

            // ---- 7. print to terminal for hardware verification ----
            std::printf("[Frame %5u] detections: %zu\n",
                        hdr.frameNumber, points.size());
            for (size_t i = 0; i < points.size(); ++i) {
                std::printf("  [%2zu]  x=%7.2f m  y=%7.2f m  z=%7.2f m  vel=%6.2f m/s\n",
                            i,
                            points[i].x, points[i].y,
                            points[i].z, points[i].doppler);
            }
            if (!points.empty()) std::fflush(stdout);

            // ---- 8. publish PointCloud2 ----
            if (!points.empty()) {
                publish_cloud(points, stamp);
            }
        }

        RCLCPP_INFO(get_logger(), "Parse thread exited");
    }

    // -----------------------------------------------------------------------
    // Build and publish sensor_msgs/PointCloud2
    // -----------------------------------------------------------------------
    void publish_cloud(const std::vector<DetectedPoint> & points,
                       const builtin_interfaces::msg::Time & stamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "radar";
        msg.height          = 1;
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

        for (const auto & p : points) {
            *it_x = p.x;       ++it_x;
            *it_y = p.y;       ++it_y;
            *it_z = p.z;       ++it_z;
            *it_v = p.doppler; ++it_v;
        }

        pub_->publish(msg);
    }

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    std::string  port_;
    int          fd_;
    std::atomic<bool> running_;
    std::thread  parse_thread_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

} // namespace cuas

// ---------------------------------------------------------------------------
// main — ROS 2 spins on the main thread; parser runs in a dedicated thread
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB2";

    auto node = std::make_shared<cuas::RadarParserNode>(port);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
