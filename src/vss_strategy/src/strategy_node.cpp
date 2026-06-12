#include <rclcpp/rclcpp.hpp>
#include <vss_msgs/msg/ball_state.hpp>
#include <vss_msgs/msg/robot_state.hpp>
#include <vss_msgs/msg/robot_command_array.hpp>
#include <vss_msgs/msg/game_state.hpp>

// Toda a lógica do jogo — header-only
#include "vss_core/vss_core.hpp"

#include <array>
#include <string>
#include <functional>

// ─────────────────────────────────────────────────────────────────────────────
//  StrategyNode
//
//  Nó principal de estratégia. Roda a 60Hz e:
//    1. Monta o GameContext a partir dos tópicos ROS 2
//    2. Chama VSSPlaybook::update(ctx)
//    3. Publica os comandos de roda em /robot_commands
//
//  Subscriptions:
//    /ball_state       → ctx.ball        (vindo do EKF)
//    /robot_raw_0/1/2  → ctx.allies[]    (vindo do bridge)
//    /game_state       → ctx.situation   (vindo do árbitro)
//
//  Publication:
//    /robot_commands   → RobotCommandArray (para o bridge enviar ao FIRASim)
// ─────────────────────────────────────────────────────────────────────────────
class StrategyNode : public rclcpp::Node {
public:
    StrategyNode() : Node("strategy"), playbook_() {
        // ── Parâmetros ────────────────────────────────────────────────────
        declare_parameter("attack_dir",  1);
        declare_parameter("team_color",  "blue");
        declare_parameter("control_hz",  60.0);
        declare_parameter("wheel_base",  0.075);

        ctx_.attack_dir = get_parameter("attack_dir").as_int();
        wheel_base_     = get_parameter("wheel_base").as_double();
        double hz       = get_parameter("control_hz").as_double();

        // ── Subscribers ───────────────────────────────────────────────────
        // Bola filtrada pelo EKF
        sub_ball_ = create_subscription<vss_msgs::msg::BallState>(
            "/ball_state",
            rclcpp::SensorDataQoS(),
            [this](vss_msgs::msg::BallState::SharedPtr msg) {
                ctx_.ball.x     = msg->x;
                ctx_.ball.y     = msg->y;
                ctx_.ball.vx    = msg->vx;
                ctx_.ball.vy    = msg->vy;
                ctx_.ball.valid = msg->valid;
            });

        // Robôs 0, 1, 2
        for (int i = 0; i < 3; i++) {
            sub_robots_[i] = create_subscription<vss_msgs::msg::RobotState>(
                "/robot_raw_" + std::to_string(i),
                rclcpp::SensorDataQoS(),
                [this, i](vss_msgs::msg::RobotState::SharedPtr msg) {
                    ctx_.allies[i].id    = msg->id;
                    ctx_.allies[i].x     = msg->x;
                    ctx_.allies[i].y     = msg->y;
                    ctx_.allies[i].theta = msg->theta;
                    ctx_.allies[i].vx    = msg->vx;
                    ctx_.allies[i].vy    = msg->vy;
                    ctx_.allies[i].valid = msg->valid;
                });
        }

        // Estado do jogo (árbitro VSSRef)
        sub_game_state_ = create_subscription<vss_msgs::msg::GameState>(
            "/game_state", 10,
            [this](vss_msgs::msg::GameState::SharedPtr msg) {
                ctx_.situation   = parseSituation(msg->situation);
                ctx_.attack_dir  = msg->attack_dir;
                ctx_.score_ally  = msg->score_ally;
                ctx_.score_enemy = msg->score_enemy;
            });

        // ── Publisher ─────────────────────────────────────────────────────
        pub_commands_ = create_publisher<vss_msgs::msg::RobotCommandArray>(
            "/robot_commands", 10);

        // ── Timer Fixo de 60Hz ────────────────────────────────────────────
        // Executa a 60Hz fixos (16ms) para sincronizar melhor com o simulador a 59 FPS
        // e evitar engasgos do executor no modo Data-Driven.
        timer_ = create_wall_timer(
            std::chrono::milliseconds(16),
            std::bind(&StrategyNode::controlLoop, this));

        RCLCPP_INFO(get_logger(),
            "StrategyNode iniciado — %.0f Hz, attack_dir=%d",
            hz, ctx_.attack_dir);
    }

private:
    vss::VSSPlaybook       playbook_;
    vss::GameContext       ctx_;
    double                 wheel_base_;
    uint32_t               frame_count_ = 0;
    std::array<vss::RobotCommand, 3> last_cmds_;

    // Subscribers
    rclcpp::Subscription<vss_msgs::msg::BallState>::SharedPtr sub_ball_;
    std::array<rclcpp::Subscription<vss_msgs::msg::RobotState>::SharedPtr, 3>
        sub_robots_;
    rclcpp::Subscription<vss_msgs::msg::GameState>::SharedPtr sub_game_state_;

    // Publisher
    rclcpp::Publisher<vss_msgs::msg::RobotCommandArray>::SharedPtr pub_commands_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ─────────────────────────────────────────────────────────────────────
    //  Loop de controle — chamado a cada frame (60Hz)
    // ─────────────────────────────────────────────────────────────────────
    void controlLoop() {
        // Descarta frames sem dados válidos ainda
        if (!ctx_.ball.valid || !ctx_.allies[0].valid) {
            return;
        }

        ctx_.frame = frame_count_++;
        ctx_.dt    = 1.0 / 60.0;

        // ── Roda o Playbook ───────────────────────────────────────────────
        auto cmds = playbook_.update(ctx_);

        // ── Slew Rate Limiter Dinâmico ────────────────────────────────────
        auto out_msg = vss_msgs::msg::RobotCommandArray();
        for (int i = 0; i < 3; i++) {
            auto& last = last_cmds_[i];
            auto& cur  = cmds[i];

            double max_delta = cur.is_fast_slew ? 0.45 : 0.15;

            if (cur.wheel_left > last.wheel_left + max_delta) 
                cur.wheel_left = last.wheel_left + max_delta;
            else if (cur.wheel_left < last.wheel_left - max_delta) 
                cur.wheel_left = last.wheel_left - max_delta;

            if (cur.wheel_right > last.wheel_right + max_delta) 
                cur.wheel_right = last.wheel_right + max_delta;
            else if (cur.wheel_right < last.wheel_right - max_delta) 
                cur.wheel_right = last.wheel_right - max_delta;

            last = cur;

            auto cmd_msg         = vss_msgs::msg::RobotCommand();
            cmd_msg.id           = cur.id;
            cmd_msg.wheel_left   = cur.wheel_left;
            cmd_msg.wheel_right  = cur.wheel_right;
            out_msg.commands.push_back(cmd_msg);
        }
        pub_commands_->publish(out_msg);

        // ── Log de Telemetria (Restaurado com Throttle de 200ms) ───────────
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
            "Telemetria VSS | Play: %s | Bola: (X:%.2f, Y:%.2f)\n"
            "  -> R0: %s\n"
            "  -> R1: %s\n"
            "  -> R2: %s",
            playbook_.activePlayName().c_str(),
            ctx_.ball.x, ctx_.ball.y,
            playbook_.activeRoleName(0).c_str(),
            playbook_.activeRoleName(1).c_str(),
            playbook_.activeRoleName(2).c_str());
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Converte string do GameState → enum GameSituation
    // ─────────────────────────────────────────────────────────────────────
    vss::GameSituation parseSituation(const std::string& s) const {
        if (s == "normal_game")      return vss::GameSituation::NORMAL_GAME;
        if (s == "halt")             return vss::GameSituation::HALT;
        if (s == "kickoff_ally")     return vss::GameSituation::KICKOFF_ALLY;
        if (s == "kickoff_enemy")    return vss::GameSituation::KICKOFF_ENEMY;
        if (s == "freeball_ally")    return vss::GameSituation::FREEBALL_ALLY;
        if (s == "freeball_enemy")   return vss::GameSituation::FREEBALL_ENEMY;
        if (s == "penalty_ally")     return vss::GameSituation::PENALTY_ALLY;
        if (s == "penalty_enemy")    return vss::GameSituation::PENALTY_ENEMY;
        if (s == "goalkick_ally")    return vss::GameSituation::GOAL_KICK_ALLY;
        if (s == "goalkick_enemy")   return vss::GameSituation::GOAL_KICK_ENEMY;
        if (s == "direct_ally")      return vss::GameSituation::DIRECT_KICK_ALLY;
        if (s == "direct_enemy")     return vss::GameSituation::DIRECT_KICK_ENEMY;
        if (s == "indirect_ally")    return vss::GameSituation::INDIRECT_KICK_ALLY;
        if (s == "indirect_enemy")   return vss::GameSituation::INDIRECT_KICK_ENEMY;
        if (s == "timeout")          return vss::GameSituation::TIMEOUT;
        return vss::GameSituation::NORMAL_GAME;  // fallback seguro
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StrategyNode>());
    rclcpp::shutdown();
    return 0;
}
