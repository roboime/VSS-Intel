#pragma once

#include "skill.hpp"
#include <cmath>
#include <algorithm>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  GoToPositionSkill
//
//  Vai para um ponto fixo do campo (não a bola).
//  Usado por DefenderRole, GoalkeeperRole, e posicionamentos de set-piece.
//
//  Diferença do GoToBallSkill:
//    · O alvo é fixo (tx, ty) em vez de seguir a bola
//    · Sem proteção da área do goleiro (esse robô PODE precisar ir lá)
//    · Aceita theta_desired para parar com orientação específica
// ─────────────────────────────────────────────────────────────────────────────

struct GoToPositionParams {
    double kp_linear          = 3.5;    // Ajustado para 3.5 (valor intermediário estável)
    double kp_angular         = 5.5;    // Ajustado para 5.5 (valor intermediário estável)
    double arrival_threshold  = 0.06;   // metros
    double max_linear_speed   = 2.2;    // m/s (Ajustado para 2.2 m/s)
    double wheel_base         = 0.075;
    bool   stop_on_arrival    = true;
};

class GoToPositionSkill final : public Skill {
public:
    using Params = GoToPositionParams;

    GoToPositionSkill(double tx, double ty,
                      double theta_desired = 1e9,  // 1e9 = "não importa"
                      Params p = Params())
        : tx_(tx), ty_(ty), theta_des_(theta_desired), params_(p) {}

    void setTarget(double tx, double ty, double theta = 1e9) {
        tx_ = tx; ty_ = ty; theta_des_ = theta;
        finished_ = false;
    }

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        double dx   = tx_ - robot.x;
        double dy   = ty_ - robot.y;
        double dist = std::hypot(dx, dy);

        // Chegou ao destino
        if (dist < params_.arrival_threshold) {
            // Se precisa de orientação específica, ajusta antes de parar
            if (theta_des_ < 1e8) {
                double ang_err = normalizeAngle(theta_des_ - robot.theta);
                if (std::abs(ang_err) > 0.05) {
                    double omega = params_.kp_angular * ang_err;
                    return clampCommand(RobotCommand::fromVW(
                        robot.id, 0.0, omega, params_.wheel_base));
                }
            }
            if (params_.stop_on_arrival) finished_ = true;
            return RobotCommand::stop(robot.id);
        }

        double angle_err = normalizeAngle(std::atan2(dy, dx) - robot.theta);
        double align     = std::max(0.0, std::cos(angle_err));
        double v = std::clamp(params_.kp_linear * dist * align,
                              -params_.max_linear_speed,
                               params_.max_linear_speed);
        
        // Desvio de obstáculos e prevenção de colisões
        v = avoidCollisions(robot, ctx, v);

        double omega = params_.kp_angular * angle_err;
        
        // Zona morta para o erro angular para evitar tremedeira/oscilação residual
        if (std::abs(angle_err) < 0.05) {
            omega = 0.0;
        }

        return clampCommand(RobotCommand::fromVW(robot.id, v, omega,
                                                  params_.wheel_base));
    }

    std::string name() const override { return "GoToPositionSkill"; }

private:
    double tx_, ty_, theta_des_;
    Params params_;
};

} // namespace vss
