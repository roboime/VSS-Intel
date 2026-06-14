#pragma once

#include "skill.hpp"
#include <cmath>
#include <algorithm>
#include <tuple>

namespace vss {

// Params fora da classe para evitar problema de default argument
struct GoToBallSkillParams {
    double kp_linear         = 5.0;   // Aumentado para 5.0 para aproximação mais agressiva
    double kp_angular        = 5.0;   // Ajustado para 5.0 (valor intermediário estável)
    double arrival_threshold = 0.04;  // Reduzido para 0.04m para se aproximar mais do alvo
    double ball_offset       = 0.04;  // metros atrás da bola em dir. ao gol
    double area_danger_dist  = 0.12;  // dist. "perto da área" (LabVIEW)
    double area_exit_offset  = 0.12;  // 120mm → LabVIEW "x + 120"
    double max_linear_speed  = 2.2;   // m/s (Ajustado para 2.2 m/s)
    double wheel_base        = 0.075; // metros entre rodas
};

// ─────────────────────────────────────────────────────────────────────────────
//  GoToBallSkill
//
//  Tradução do GoToBallSkillVSS.vi — lógica extraída das fotos do LabVIEW:
//
//  · Destino = posição da bola com offset de alinhamento ao gol
//  · Proteção da área do goleiro:
//      - robô dentro da área  → empurra para fora (area_exit_offset)
//      - robô perto da área + bola perto + ambos no Y do gol → para o robô
//  · Clamp do destino dentro do campo
//  · Controlador proporcional com fator de alinhamento (≈0.35 do LabVIEW)
// ─────────────────────────────────────────────────────────────────────────────
class GoToBallSkill final : public Skill {
public:
    using Params = GoToBallSkillParams;

    explicit GoToBallSkill(Params p = Params()) : params_(p) {}

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
        last_v_ = 0.0;
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        // ── 1. Destino base: bola com offset em direção ao gol ────────────
        auto goal = ctx.enemyGoalCenter();
        double ang = std::atan2(goal.y - ctx.ball.y, goal.x - ctx.ball.x);
        double target_x = ctx.ball.x - std::cos(ang) * params_.ball_offset;
        double target_y = ctx.ball.y - std::sin(ang) * params_.ball_offset;

        // ── 2. Proteção da área do goleiro ────────────────────────────────
        bool should_stop = false;
        std::tie(target_x, target_y, should_stop) =
            goalAreaProtection(robot, target_x, target_y, ctx);
        if (should_stop) return RobotCommand::stop(robot.id);

        // ── 3. Clamp dentro do campo ──────────────────────────────────────
        const double m = 0.08; // Aumentado de 0.05 para 0.08 para evitar colisões com as paredes
        target_x = std::clamp(target_x, ctx.field.min_x()+m, ctx.field.max_x()-m);
        target_y = std::clamp(target_y, ctx.field.min_y()+m, ctx.field.max_y()-m);

        // ── Erro de posição e ângulo ───────────────────────────────────────────
        double dx   = target_x - robot.x;
        double dy   = target_y - robot.y;
        double dist = std::hypot(dx, dy);

        if (dist < params_.arrival_threshold) {
            finished_ = true;
            return RobotCommand::stop(robot.id);
        }

        // ── Direção corrigida pelo APF ─────────────────────────────────────────
        // ignore_robot_id = -1: aplica repulsão de todos os robôs próximos.
        // A bola está no campo; se um adversário está segurando a bola,
        // o atacante deve empurrá-lo — o APF não repele o robô adversário
        // que ESTÁ na posição da bola (distância < arrival_threshold do alvo).
        // Para garantir isso, desativamos a repulsão de qualquer robô cuja
        // distância ao ALVO (target) seja menor que 0.08m.
        int ignore_id = -1;
        for (const auto& enemy : ctx.enemies) {
            if (!enemy.valid) continue;
            double det = std::hypot(enemy.x - target_x, enemy.y - target_y);
            if (det < 0.08) { ignore_id = static_cast<int>(enemy.id); break; }
        }

        double target_angle;
        if (dist >= 0.02) {
            target_angle = applyAPF(robot, ctx, target_x, target_y, ignore_id);
        } else {
            target_angle = robot.theta;
        }

        // ── Erro angular (normalizado em [-PI, PI]) ───────────────────────────
        double angle_err = normalizeAngle(target_angle - robot.theta);

        // ── Zona morta de ângulo ──────────────────────────────────────────────
        if (std::abs(angle_err) < 0.05) angle_err = 0.0;

        // ── Controle proporcional com fator de alinhamento ────────────────────────────
        double align = std::max(0.0, std::cos(angle_err));
        double v     = std::clamp(params_.kp_linear * dist * align,
                                  -params_.max_linear_speed,
                                   params_.max_linear_speed);

        // Velocidade mínima na aproximação final
        if (dist < 0.15 && align > 0.5) {
            v = std::max(v, 1.0 * align);
        }

        // [CORRIGIDO Bug #2D] Piso de velocidade para erros angulares elevados.
        // Para bola lateral (~80°), align ≈ 0.17 → v ≈ 0.26 m/s: o robô quase para
        // e oscila tentando se orientar sem transladar, nunca completando a órbita.
        // Correção: quando |angle_err| > 60° e o robô ainda não chegou,
        // mantém velocidade mínima de 0.5 m/s para que a órbita avance.
        // A rampa de aceleração trapezoidal (applyLinearRamp) absorve a transição suavemente.
        if (std::abs(angle_err) > M_PI / 3.0 && dist > params_.arrival_threshold) {
            v = std::max(v, 0.5);
        }

        // Desvio de obstáculos (frenagem linear)
        v = avoidCollisions(robot, ctx, v);

        // ── Omega com rampa de desaceleração angular ──────────────────────────
        double omega_raw = params_.kp_angular * angle_err;
        double omega = applyAngularRamp(angle_err, omega_raw,
                                        params_.kp_angular * M_PI);

        // ── Perfil de Aceleração Trapezoidal (Linear) ─────────────────────────
        v = applyLinearRamp(v, last_v_, 5.0); // max accel 5.0 m/s^2

        // ── Zona morta de distância ───────────────────────────────────────────
        if (dist < 0.02) { v = 0.0; omega = 0.0; }

        last_v_ = v;

        return clampCommand(RobotCommand::fromVW(robot.id, v, omega,
                                                  params_.wheel_base));
    }

    std::string name() const override { return "GoToBallSkill"; }

private:
    Params params_;
    double last_v_ = 0.0;

    // Extrai as condições comentadas no LabVIEW sobre a área do goleiro
    std::tuple<double,double,bool>
    goalAreaProtection(const RobotState& robot,
                       double tx, double ty,
                       const GameContext& ctx) const
    {
        double area_x = ctx.attack_dir *
                        (ctx.field.max_x() - ctx.field.area_length);

        bool robot_in  = ctx.isInEnemyGoalArea(robot.x,    robot.y);
        bool ball_near = std::abs(std::abs(ctx.ball.x) -
                         (ctx.field.max_x()-ctx.field.area_length))
                         < params_.area_danger_dist;
        bool robot_near = std::abs(std::abs(robot.x) -
                          (ctx.field.max_x()-ctx.field.area_length))
                          < params_.area_danger_dist;
        bool robot_y_goal = std::abs(robot.y)    < ctx.field.goal_width/2.0;
        bool ball_y_goal  = std::abs(ctx.ball.y) < ctx.field.goal_width/2.0;

        if (robot_in) {
            // "Sai da área (x + 120)"
            double exit_x = area_x - ctx.attack_dir * params_.area_exit_offset;
            return {exit_x, robot.y, false};
        }

        // "É o robô perto da área E bola perto E y_robô no gol E y_bola no gol"
        if (robot_near && ball_near && robot_y_goal && ball_y_goal) {
            return {tx, ty, true};  // "fica parado"
        }

        return {tx, ty, false};
    }
};

} // namespace vss
