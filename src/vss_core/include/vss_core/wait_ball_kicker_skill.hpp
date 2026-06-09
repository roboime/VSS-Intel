#pragma once

#include "skill.hpp"
#include <cmath>
#include <algorithm>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  WaitBallKickerSkill
//
//  Tradução do WaitBallKickerSkill_class do LabVIEW.
//
//  Comportamento:
//    1. Robô se posiciona NA TRAJETÓRIA da bola (não vai até a bola)
//    2. Espera a bola chegar até ele
//    3. Quando a bola está perto o suficiente → termina (próxima skill = Kick)
//
//  Usado em: WaitBallKickerTactic → WaitBallKickerSkill → KickSkill
//  Cenário típico: receber um passe e chutar
// ─────────────────────────────────────────────────────────────────────────────

struct WaitBallKickerParams {
    double intercept_dist      = 0.08;   // dist. para considerar "bola chegou"
    double position_tolerance  = 0.05;   // tolerância de posicionamento na traj.
    double kp_linear           = 3.0;
    double kp_angular          = 5.0;
    double max_linear_speed    = 1.0;
    double wheel_base          = 0.075;
    // Lookahead: quanto tempo na frente prediz a trajetória da bola
    double lookahead_seconds   = 0.3;
};

class WaitBallKickerSkill final : public Skill {
public:
    using Params = WaitBallKickerParams;

    explicit WaitBallKickerSkill(Params p = Params()) : params_(p) {}

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        const BallState& ball = ctx.ball;

        // ── 1. Bola chegou? → termina, Tactic vai para KickSkill ──────────
        double dist_to_ball = std::hypot(robot.x - ball.x, robot.y - ball.y);
        if (dist_to_ball < params_.intercept_dist) {
            finished_ = true;
            return RobotCommand::stop(robot.id);
        }

        // ── 2. Calcular ponto de intercepção na trajetória da bola ────────
        // Prediz onde a bola estará em lookahead_seconds
        double pred_x = ball.x + ball.vx * params_.lookahead_seconds;
        double pred_y = ball.y + ball.vy * params_.lookahead_seconds;

        // Projeta o robô na linha da trajetória da bola
        double target_x, target_y;
        double ball_speed = std::hypot(ball.vx, ball.vy);

        if (ball_speed < 0.05) {
            // Bola parada: vai até ela diretamente
            target_x = ball.x;
            target_y = ball.y;
        } else {
            // Projeção do robô na reta da trajetória da bola
            // Vetor direção da bola (normalizado)
            double bvx = ball.vx / ball_speed;
            double bvy = ball.vy / ball_speed;

            // Vetor robô → bola
            double rx = robot.x - ball.x;
            double ry = robot.y - ball.y;

            // Projeção escalar
            double proj = rx * bvx + ry * bvy;

            // Ponto mais próximo do robô na trajetória da bola
            target_x = ball.x + proj * bvx;
            target_y = ball.y + proj * bvy;

            // Se o ponto projetado está atrás da bola (bola já passou)
            // vai para a posição predita
            if (proj < 0) {
                target_x = pred_x;
                target_y = pred_y;
            }
        }

        // ── 3. Vai para o ponto de intercepção ────────────────────────────
        double dx   = target_x - robot.x;
        double dy   = target_y - robot.y;
        double dist = std::hypot(dx, dy);

        if (dist < params_.position_tolerance) {
            // Já está na posição — aguarda a bola chegando
            // Orienta para a bola para já estar pronto para o chute
            double ang_to_ball = std::atan2(ball.y - robot.y, ball.x - robot.x);
            double ang_err     = normalizeAngle(ang_to_ball - robot.theta);
            double omega = params_.kp_angular * ang_err;
            return clampCommand(RobotCommand::fromVW(robot.id, 0.0, omega,
                                                      params_.wheel_base));
        }

        double angle_err = normalizeAngle(std::atan2(dy, dx) - robot.theta);
        double align     = std::max(0.0, std::cos(angle_err));
        double v = std::clamp(params_.kp_linear * dist * align,
                              -params_.max_linear_speed, params_.max_linear_speed);
        double omega = params_.kp_angular * angle_err;

        return clampCommand(RobotCommand::fromVW(robot.id, v, omega,
                                                  params_.wheel_base));
    }

    std::string name() const override { return "WaitBallKickerSkill"; }

private:
    Params params_;
};

} // namespace vss
