#pragma once

#include "skill.hpp"
#include <cmath>
#include <algorithm>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  RotateToAngleSkill
//
//  Gira o robô no lugar até atingir um ângulo desejado.
//  Extraído do KickSkillVSS: "Rotate Clockwise" / "Rotate anti-clockwise"
//  com decisão baseada em "y bola < y robo?" e proximidade de paredes.
//
//  -1 = sentido horário (Rotate Clockwise)     → omega negativo
//  +1 = sentido anti-horário (Rotate CCW)      → omega positivo
// ─────────────────────────────────────────────────────────────────────────────

struct RotateToAngleParams {
    double kp_angular         = 6.5;   // Ajustado para 6.5 (valor intermediário estável)
    double arrival_threshold  = 0.25;  // Aumentado para 0.25 rad (~14.3 graus) para evitar oscilações ao mirar
    double max_omega          = 10.0;  // rad/s (Ajustado para 10.0)
    double wheel_base         = 0.075;
};

class RotateToAngleSkill final : public Skill {
public:
    using Params = RotateToAngleParams;

    // target_angle em radianos. Se não fornecido, usa ângulo para o gol.
    explicit RotateToAngleSkill(double target_angle = 0.0, Params p = Params())
        : target_angle_(target_angle), params_(p), use_goal_(false) {}

    // Construtor que calcula o ângulo em direção ao gol no execute()
    static RotateToAngleSkill towardGoal(Params p = Params()) {
        RotateToAngleSkill s(0.0, p);
        s.use_goal_ = true;
        return s;
    }

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
        if (use_goal_) {
            auto goal = ctx.enemyGoalCenter();
            target_angle_ = std::atan2(goal.y - robot.y, goal.x - robot.x);
        }
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        (void)ctx;
        // Erro angular estritamente normalizado em [-PI, PI]
        double err = normalizeAngle(target_angle_ - robot.theta);

        if (std::abs(err) < params_.arrival_threshold) {
            finished_ = true;
            return RobotCommand::stop(robot.id);
        }

        // Controlador P com rampa de desaceleração angular para evitar overshoot
        double omega_raw = params_.kp_angular * err;
        double omega = applyAngularRamp(err, omega_raw, params_.max_omega);

        return clampCommand(RobotCommand::fromVW(robot.id, 0.0, omega,
                                                  params_.wheel_base));
    }

    std::string name() const override { return "RotateToAngleSkill"; }

private:
    double target_angle_;
    Params params_;
    bool   use_goal_ = false;
};

} // namespace vss
