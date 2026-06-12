#pragma once

#include "types.hpp"
#include "game_context.hpp"
#include <string>
#include <memory>
#include <algorithm>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  Skill  —  nível mais baixo da hierarquia VSS
//
//  Equivale a todas as XxxSkill_class do LabVIEW.
//  Uma Skill recebe o estado do mundo e devolve um comando de roda.
//
//  Ciclo de vida:
//    1. init()    — chamado uma vez quando a Skill é selecionada
//    2. execute() — chamado a cada frame (60 Hz no FIRASim)
//    3. isFinished() — a Tactic verifica isso para avançar para próxima Skill
//
//  Mapeamento do LabVIEW:
//    Robot in + Game in → execute() → Robot out (comando de roda)
// ─────────────────────────────────────────────────────────────────────────────
class Skill {
public:
    virtual ~Skill() = default;

    // Chamado quando a Skill é (re)iniciada
    // Equivale ao "init" implícito do LabVIEW quando o VI é ativado
    virtual void init(const RobotState& robot, const GameContext& ctx) {
        (void)robot; (void)ctx;
        finished_ = false;
    }

    // Executado a cada frame — devolve o comando de roda
    // É o equivalente direto do corpo principal do VI no LabVIEW
    virtual RobotCommand execute(const RobotState& robot,
                                 const GameContext& ctx) = 0;

    // Verdadeiro quando a Skill completou seu objetivo
    // A Tactic usa isso para encadear Skills
    virtual bool isFinished() const { return finished_; }

    // Nome para debug / logs
    virtual std::string name() const = 0;

protected:
    bool finished_ = false;

    static constexpr double MAX_WHEEL_SPEED = 2.5;

    RobotCommand clampCommand(RobotCommand cmd) const {
        cmd.wheel_left  = std::clamp(cmd.wheel_left,  -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        cmd.wheel_right = std::clamp(cmd.wheel_right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        return cmd;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  applyAPF — Campos Potenciais Artificiais (APF)
    //
    //  Combina o vetor de atração ao alvo com forças de repulsão de:
    //    · Robôs próximos (aliados e adversários) dentro de 0.12m
    //    · Paredes do campo dentro de 0.10m
    //    · Força tangencial de escape escolhendo o lado com maior espaço livre
    //      (alinhada com o vetor de atração) para evitar mínimos locais.
    //
    //  ignore_robot_id     — ID do robô a ignorar na repulsão (ex: -1 = nenhum).
    //                        Usado no GoToBall para não repelir o robô que está
    //                        na bola e impediria o atacante de alcançá-la.
    //  robot_repulsion_gain— ganho de repulsão de robôs. Passe 0.0 para
    //                        desativar completamente (fase CHARGE do KickSkill).
    //
    //  Retorna: ângulo alvo corrigido (rad) em [-PI, PI].
    // ─────────────────────────────────────────────────────────────────────────
    double applyAPF(const RobotState& robot,
                    const GameContext& ctx,
                    double target_x, double target_y,
                    int    ignore_robot_id      = -1,
                    double robot_repulsion_gain = 0.08) const
    {
        // ── 1. Vetor de atração unitário ──────────────────────────────────────
        double att_x = target_x - robot.x;
        double att_y = target_y - robot.y;
        double att_dist = std::hypot(att_x, att_y);
        if (att_dist > 1e-6) { att_x /= att_dist; att_y /= att_dist; }

        double rep_x = 0.0, rep_y = 0.0;

        // ── 2. Repulsão de robôs próximos ─────────────────────────────────────
        if (robot_repulsion_gain > 0.0) {
            const double R_robot = 0.12;   // raio de influência (m)
            const double D_min   = 0.076;  // diâmetro físico mínimo (m)

            auto addRobotRepulsion = [&](double ox, double oy) {
                double dx   = robot.x - ox;
                double dy   = robot.y - oy;
                double dist = std::hypot(dx, dy);
                if (dist < R_robot && dist > 1e-6) {
                    double safe_dist = std::max(dist, D_min);
                    double mag = robot_repulsion_gain * (1.0/safe_dist - 1.0/R_robot);
                    rep_x += mag * (dx / dist);
                    rep_y += mag * (dy / dist);
                }
            };

            for (const auto& ally : ctx.allies) {
                if (!ally.valid || ally.id == robot.id) continue;
                if (ignore_robot_id >= 0 && static_cast<int>(ally.id) == ignore_robot_id) continue;
                addRobotRepulsion(ally.x, ally.y);
            }
            for (const auto& enemy : ctx.enemies) {
                if (!enemy.valid) continue;
                if (ignore_robot_id >= 0 && static_cast<int>(enemy.id) == ignore_robot_id) continue;
                addRobotRepulsion(enemy.x, enemy.y);
            }
        }

        // ── 3. Repulsão de paredes ────────────────────────────────────────────
        {
            const double R_wall = 0.10;  // raio de influência das paredes (m)
            const double k_wall = 0.10;  // ganho de repulsão das paredes

            double d_left  = robot.x - ctx.field.min_x();
            double d_right = ctx.field.max_x() - robot.x;
            double d_bot   = robot.y - ctx.field.min_y();
            double d_top   = ctx.field.max_y() - robot.y;

            if (d_left  < R_wall && d_left  > 1e-6) rep_x += k_wall * (1.0/d_left  - 1.0/R_wall);
            if (d_right < R_wall && d_right > 1e-6) rep_x -= k_wall * (1.0/d_right - 1.0/R_wall);
            if (d_bot   < R_wall && d_bot   > 1e-6) rep_y += k_wall * (1.0/d_bot   - 1.0/R_wall);
            if (d_top   < R_wall && d_top   > 1e-6) rep_y -= k_wall * (1.0/d_top   - 1.0/R_wall);
        }

        // ── 4. Força tangencial de escape (evita mínimos locais) ──────────────
        // Perpendicular à repulsão, no sentido mais alinhado com a atração
        // (= lado com mais espaço livre em direção ao alvo).
        {
            double rep_mag = std::hypot(rep_x, rep_y);
            if (rep_mag > 1e-4) {
                double nx = rep_x / rep_mag;
                double ny = rep_y / rep_mag;
                // Dois candidatos perpendiculares ao vetor de repulsão
                double tan_x =  -ny;  // +90 graus
                double tan_y =   nx;
                double dot_pos = att_x * tan_x  + att_y * tan_y;
                double dot_neg = att_x * (-tan_x) + att_y * (-tan_y);
                double k_tan = 0.05;
                if (dot_pos >= dot_neg) {
                    rep_x += k_tan * tan_x;
                    rep_y += k_tan * tan_y;
                } else {
                    rep_x += k_tan * (-tan_x);
                    rep_y += k_tan * (-tan_y);
                }
            }
        }

        // ── 5. Resultado: atração + repulsão ──────────────────────────────────
        double final_x = att_x + rep_x;
        double final_y = att_y + rep_y;
        return normalizeAngle(std::atan2(final_y, final_x));
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  avoidCollisions — frenagem linear por proximidade de robôs e paredes
    //  (escala a velocidade linear, não altera a direção)
    // ─────────────────────────────────────────────────────────────────────────
    double avoidCollisions(const RobotState& robot, const GameContext& ctx, double v) const {
        // Robôs aliados
        for (const auto& ally : ctx.allies) {
            if (ally.id == robot.id || !ally.valid) continue;
            double dx = ally.x - robot.x, dy = ally.y - robot.y;
            double dist = std::hypot(dx, dy);
            if (dist < 0.13) {
                double ang_diff = std::abs(normalizeAngle(std::atan2(dy, dx) - robot.theta));
                if (ang_diff < M_PI / 4.0) {
                    double scale = (dist - 0.075) / (0.13 - 0.075);
                    v *= std::clamp(scale, 0.0, 1.0);
                }
            }
        }
        // Robôs adversários
        for (const auto& enemy : ctx.enemies) {
            if (!enemy.valid) continue;
            double dx = enemy.x - robot.x, dy = enemy.y - robot.y;
            double dist = std::hypot(dx, dy);
            if (dist < 0.13) {
                double ang_diff = std::abs(normalizeAngle(std::atan2(dy, dx) - robot.theta));
                if (ang_diff < M_PI / 4.0) {
                    double scale = (dist - 0.075) / (0.13 - 0.075);
                    v *= std::clamp(scale, 0.0, 1.0);
                }
            }
        }
        // Paredes (frenagem suave a 10 cm da parede)
        {
            const double R_wall = 0.10;
            const double D_min  = 0.025;
            double d_wall = std::min({
                robot.x - ctx.field.min_x(),
                ctx.field.max_x() - robot.x,
                robot.y - ctx.field.min_y(),
                ctx.field.max_y() - robot.y
            });
            if (d_wall < R_wall) {
                double scale = (d_wall - D_min) / (R_wall - D_min);
                v *= std::clamp(scale, 0.0, 1.0);
            }
        }
        return v;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  applyAngularRamp — desaceleração angular suave
    //
    //  Limita omega proporcionalmente ao erro angular residual para evitar
    //  overshoot e giros infinitos/oscilações ao se alinhar em diagonais.
    //    err_rad   — erro angular atual (rad), normalizado em [-PI, PI]
    //    omega_raw — omega calculado pelo controlador P
    //    max_omega — velocidade angular máxima (rad/s)
    // ─────────────────────────────────────────────────────────────────────────
    static double applyAngularRamp(double err_rad, double omega_raw, double max_omega)
    {
        const double ramp_start = 0.60;  // rad — começa a reduzir a partir daqui
        const double ramp_end   = 0.05;  // rad — mínimo de velocidade aqui
        const double min_frac   = 0.20;  // 20% de max_omega no limite mínimo

        double abs_err = std::abs(err_rad);
        double limit;
        if (abs_err >= ramp_start) {
            limit = max_omega;
        } else if (abs_err <= ramp_end) {
            limit = max_omega * min_frac;
        } else {
            double t = (abs_err - ramp_end) / (ramp_start - ramp_end);
            limit = max_omega * (min_frac + (1.0 - min_frac) * t);
        }
        return std::clamp(omega_raw, -limit, limit);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  applyLinearRamp — Perfil de Aceleração Trapezoidal (Slew Rate)
    //
    //  Limita a variação da velocidade linear (dv/dt) para evitar derrapagens
    //  (slippage) e garantir tração máxima das rodas no FIRASim (60 Hz).
    //    target_v   — velocidade linear desejada neste frame
    //    current_v  — velocidade linear comandada no frame anterior
    //    max_accel  — aceleração máxima permitida (m/s^2)
    // ─────────────────────────────────────────────────────────────────────────
    static double applyLinearRamp(double target_v, double current_v, double max_accel = 4.0)
    {
        // Simulador a 60 FPS -> dt = 1/60s = 0.0166s
        const double dt = 0.0166667;
        double max_dv = max_accel * dt; // Variação máxima permitida no frame

        if (target_v > current_v + max_dv) {
            return current_v + max_dv;
        } else if (target_v < current_v - max_dv) {
            return current_v - max_dv;
        }
        return target_v;
    }

    static double distToBall(const RobotState& robot, const GameContext& ctx) {
        return std::hypot(robot.x - ctx.ball.x, robot.y - ctx.ball.y);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  Skill de comportamento nulo — robô para
//  Equivale ao HaltSkill_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class HaltSkill final : public Skill {
public:
    RobotCommand execute(const RobotState& robot,
                         const GameContext&) override
    {
        return RobotCommand::stop(robot.id);
    }
    std::string name() const override { return "HaltSkill"; }
};

} // namespace vss
