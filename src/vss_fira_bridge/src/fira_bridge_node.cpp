#include <rclcpp/rclcpp.hpp>
#include <vss_msgs/msg/ball_state.hpp>
#include <vss_msgs/msg/robot_state.hpp>
#include <vss_msgs/msg/robot_command.hpp>
#include <vss_msgs/msg/robot_command_array.hpp>
#include <vss_msgs/msg/game_state.hpp>

#include "proto/packet.pb.h"
#include "proto/command.pb.h"
#include "proto/common.pb.h"
#include "proto/replacement.pb.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <array>
#include <string>
#include <functional>

class FIRABridgeNode : public rclcpp::Node {
public:
    FIRABridgeNode() : Node("fira_bridge") {
        declare_parameter("team_color",   "blue");
        declare_parameter("vision_port",  10002);
        declare_parameter("command_ip",   "127.0.0.1");
        declare_parameter("command_port", 20011);

        team_color_   = get_parameter("team_color").as_string();
        is_blue_team_ = (team_color_ == "blue");
        cmd_ip_       = get_parameter("command_ip").as_string();
        cmd_port_     = get_parameter("command_port").as_int();

        pub_ball_ = create_publisher<vss_msgs::msg::BallState>(
            "/ball_raw", rclcpp::SensorDataQoS());
        for (int i = 0; i < 3; i++) {
            pub_robots_[i] = create_publisher<vss_msgs::msg::RobotState>(
                "/robot_raw_" + std::to_string(i),
                rclcpp::SensorDataQoS());
        }
        pub_game_state_ = create_publisher<vss_msgs::msg::GameState>(
            "/game_state", 10);

        sub_commands_ = create_subscription<vss_msgs::msg::RobotCommandArray>(
            "/robot_commands", 10,
            std::bind(&FIRABridgeNode::onCommand, this,
                      std::placeholders::_1));

        setupVisionSocket();
        setupCommandSocket();

        timer_ = create_wall_timer(
            std::chrono::milliseconds(8),
            std::bind(&FIRABridgeNode::readVision, this));

        auto gs       = vss_msgs::msg::GameState();
        gs.situation  = game_situation_;
        gs.attack_dir = is_blue_team_ ? 1 : -1;
        pub_game_state_->publish(gs);

        RCLCPP_INFO(get_logger(),
            "FIRABridgeNode iniciado — time: %s", team_color_.c_str());
    }

    ~FIRABridgeNode() {
        if (vision_sock_ >= 0) close(vision_sock_);
        if (cmd_sock_    >= 0) close(cmd_sock_);
    }

private:
    rclcpp::Publisher<vss_msgs::msg::BallState>::SharedPtr    pub_ball_;
    rclcpp::Publisher<vss_msgs::msg::GameState>::SharedPtr    pub_game_state_;
    std::array<rclcpp::Publisher<vss_msgs::msg::RobotState>::SharedPtr, 3>
        pub_robots_;
    rclcpp::Subscription<vss_msgs::msg::RobotCommandArray>::SharedPtr
        sub_commands_;
    rclcpp::TimerBase::SharedPtr timer_;

    int  vision_sock_ = -1;
    int  cmd_sock_    = -1;
    std::string cmd_ip_;
    int         cmd_port_;
    sockaddr_in cmd_addr_{};
    bool        is_blue_team_;
    std::string team_color_;
	std::string game_situation_ = "normal_game";

    static constexpr size_t BUFFER_SIZE = 65536;
    std::array<char, BUFFER_SIZE> buf_{};

    void setupVisionSocket() {
        auto port = get_parameter("vision_port").as_int();

        vision_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (vision_sock_ < 0) {
            RCLCPP_ERROR(get_logger(), "Erro ao criar socket de visão");
            return;
        }

        int reuse = 1;
        setsockopt(vision_sock_, SOL_SOCKET, SO_REUSEADDR,
                   &reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_port        = htons(static_cast<uint16_t>(port));
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(vision_sock_, reinterpret_cast<sockaddr*>(&addr),
                 sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "Erro ao fazer bind no socket de visão");
            return;
        }

        // Multicast no loopback (FIRASim envia por lo)
        ip_mreq mreq{};
        mreq.imr_multiaddr.s_addr = inet_addr("224.0.0.1");
        mreq.imr_interface.s_addr = inet_addr("127.0.0.1");
        setsockopt(vision_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                   &mreq, sizeof(mreq));

	// Habilita receber multicast no loopback
	int loop = 1;
	setsockopt(vision_sock_, IPPROTO_IP, IP_MULTICAST_LOOP,
           &loop, sizeof(loop));

        int flags = fcntl(vision_sock_, F_GETFL, 0);
        fcntl(vision_sock_, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(get_logger(), "Socket visão OK — loopback:%ld", port);
    }

    void setupCommandSocket() {
        cmd_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (cmd_sock_ < 0) {
            RCLCPP_ERROR(get_logger(), "Erro ao criar socket de comando");
            return;
        }
        cmd_addr_.sin_family      = AF_INET;
        cmd_addr_.sin_port        = htons(static_cast<uint16_t>(cmd_port_));
        cmd_addr_.sin_addr.s_addr = inet_addr(cmd_ip_.c_str());

        RCLCPP_INFO(get_logger(),
            "Socket comando OK — %s:%d", cmd_ip_.c_str(), cmd_port_);
    }

    void readVision() {
    ssize_t n = recv(vision_sock_, buf_.data(), buf_.size(), 0);
    
    if (n > 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Recebeu %ld bytes do FIRASim", n);
    }
    
    if (n <= 0) return;
		
    // Publica o GameState continuamente a cada frame recebido
    auto gs       = vss_msgs::msg::GameState();
    gs.situation  = game_situation_;
    gs.attack_dir = is_blue_team_ ? 1 : -1;
    pub_game_state_->publish(gs);

    fira_message::sim_to_ref::Environment env;
    if (!env.ParseFromArray(buf_.data(), static_cast<int>(n))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Falha ao parsear Environment proto");
        return;
    }
    const auto& frame = env.frame();
    if (frame.has_ball()) {
        auto msg  = vss_msgs::msg::BallState();
        msg.x     = frame.ball().x();
        msg.y     = frame.ball().y();
        msg.vx    = frame.ball().vx();
        msg.vy    = frame.ball().vy();
        msg.valid = true;
        pub_ball_->publish(msg);
    }
    const auto& our_robots = is_blue_team_
                             ? frame.robots_blue()
                             : frame.robots_yellow();
    for (const auto& r : our_robots) {
        if (r.robot_id() >= 3) continue;
        auto msg    = vss_msgs::msg::RobotState();
        msg.id      = static_cast<uint8_t>(r.robot_id());
        msg.x       = r.x();
        msg.y       = r.y();
        msg.theta   = r.orientation();
        msg.vx      = r.vx();
        msg.vy      = r.vy();
        msg.valid   = true;
        pub_robots_[msg.id]->publish(msg);
    }
}
    void onCommand(const vss_msgs::msg::RobotCommandArray::SharedPtr msg) {
        fira_message::sim_to_ref::Packet packet;
        auto* cmd_packet = packet.mutable_cmd();

        for (const auto& cmd : msg->commands) {
            auto* robot_cmd = cmd_packet->add_robot_commands();
            robot_cmd->set_id(cmd.id);
            robot_cmd->set_yellowteam(!is_blue_team_);
            robot_cmd->set_wheel_left(static_cast<float>(cmd.wheel_left));
            robot_cmd->set_wheel_right(static_cast<float>(cmd.wheel_right));
        }

        std::string serialized;
        if (!packet.SerializeToString(&serialized)) {
            RCLCPP_WARN(get_logger(), "Falha ao serializar comando");
            return;
        }

        sendto(cmd_sock_, serialized.data(), serialized.size(), 0,
               reinterpret_cast<sockaddr*>(&cmd_addr_), sizeof(cmd_addr_));
    }
};

int main(int argc, char** argv) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FIRABridgeNode>());
    rclcpp::shutdown();
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
