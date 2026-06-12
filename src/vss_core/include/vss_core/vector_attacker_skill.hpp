#pragma once

#include "skill.hpp"
#include <cmath>
#include <algorithm>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  VectorAttackerSkill
//
//  Máquina de estados de 3 estágios orientada a vetores (Vector Field Attack).
//  Substitui as antigas jogadas baseadas puramente na coordenada da bola.
//
//  Estados:
//    - BYPASS: Robô está entre a bola e o gol adversário. Contorna a bola.
//    - ALIGN: Robô vai para um ponto 10 cm atrás da bola alinhado com o gol.
//    - PUSH: Robô alinhado. Acelera com tudo mirando DENTRO do gol adversário.
// ─────────────────────────────────────────────────────────────────────────────
class VectorAttackerSkill final : public Skill {
public:
    VectorAttackerSkill() = default;

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
        last_v_ = 0.0;
        stuck_ref_x_ = robot.x;
        stuck_ref_y_ = robot.y;
        stuck_frames_ = 0;
        is_unsticking_ = false;
        unstick_frames_ = 0;
    }

    RobotCommand execute(const RobotState& robot, const GameContext& ctx) override {
        // ── 0. Rotina de Anti-Travamento (Unstick) Sequencial ─────────────
        if (is_unsticking_) {
            unstick_frames_++;
            if (unstick_frames_ > 30) {
                is_unsticking_ = false;
                stuck_frames_ = 0;
                stuck_ref_x_ = robot.x;
                stuck_ref_y_ = robot.y;
            } else {
                if (unstick_frames_ <= 12) {
                    // FASE 1: DESCOLAR (Marcha à ré linear pura, cria espaço)
                    last_v_ = -3.0;
                    return clampCommand(RobotCommand::fromVW(robot.id, -3.0, 0.0, 0.075));
                } else {
                    // FASE 2: GIRAR E ESCAPAR (Ré leve + Giro violento)
                    double spin_dir = (robot.y > 0.0) ? 1.0 : -1.0;
                    last_v_ = -1.0;
                    return clampCommand(RobotCommand::fromVW(robot.id, -1.0, 12.0 * spin_dir, 0.075));
                }
            }
        }

        // Rastreamento de deslocamento por janela de tempo (odometria pela visão)
        if (stuck_frames_ == 0) {
            stuck_ref_x_ = robot.x;
            stuck_ref_y_ = robot.y;
        }

        double real_dist_moved = std::hypot(robot.x - stuck_ref_x_, robot.y - stuck_ref_y_);
        
        // Se estamos comandando movimento ativo (v > 0.5)
        if (std::abs(last_v_) > 0.5) {
            stuck_frames_++;
            // Se o robô conseguir se afastar > 1.5 cm da referência de travamento, resetamos
            if (real_dist_moved > 0.015) {
                stuck_frames_ = 0;
                stuck_ref_x_ = robot.x;
                stuck_ref_y_ = robot.y;
            }
        } else {
            stuck_frames_ = 0;
            stuck_ref_x_ = robot.x;
            stuck_ref_y_ = robot.y;
        }

        // 40 frames (~0.66s) tentando mover e sem conseguir sair do raio de 1.5cm
        if (stuck_frames_ > 40) {
            is_unsticking_ = true;
            unstick_frames_ = 0;
            last_v_ = -2.5;
            return clampCommand(RobotCommand::fromVW(robot.id, -2.5, 12.0, 0.075));
        }

        // ── 1. Bola Preditiva (Predictive Tracking) ───────────────────────
        // Tempo de predição validado: 150ms
        double pred_t = 0.15;
        double pred_x = ctx.ball.x + ctx.ball.vx * pred_t;
        double pred_y = ctx.ball.y + ctx.ball.vy * pred_t;

        // Limita a predição para não sair do campo (evita que o robô ataque a parede)
        const double margin = 0.05;
        pred_x = std::clamp(pred_x, ctx.field.min_x() + margin, ctx.field.max_x() - margin);
        pred_y = std::clamp(pred_y, ctx.field.min_y() + margin, ctx.field.max_y() - margin);

        auto goal = ctx.enemyGoalCenter();
        double theta_goal = std::atan2(goal.y - pred_y, goal.x - pred_x);

        // ── 2. Campo Vetorial Contínuo (Spiral Vector Field) ──────────────
        double dx = pred_x - robot.x;
        double dy = pred_y - robot.y;
        double dist = std::hypot(dx, dy);

        double angle_to_ball = std::atan2(dy, dx);
        double theta_err = normalizeAngle(theta_goal - angle_to_ball);

        // Offset espiral: quando na frente da bola, o vetor desvia em até 90 graus para contornar.
        // À medida que vai para trás da bola, o erro tangencial zera organicamente.
        double max_offset = M_PI / 2.0; 
        double k_circ = 1.5; // Fator de agressividade da curva de contorno
        double offset_angle = std::min(max_offset, k_circ * std::abs(theta_err));

        // Angulação final do Campo Vetorial
        double target_angle = angle_to_ball + (theta_err >= 0 ? offset_angle : -offset_angle);
        target_angle = normalizeAngle(target_angle);

        // ── 3. Controle e Desbloqueio Dinâmico (Slew Rate / Push) ─────────
        // Robô está no Cone de Ataque? (Atrás da bola e direcionado ao gol)
        bool in_attack_cone = (std::abs(theta_err) < 0.35 && dist < 0.18);
        
        double v_max = 2.5;
        double v = v_max;
        bool is_fast_slew = false;

        if (in_attack_cone) {
            // Fase Kamikaze (Desbloqueia acelerador)
            is_fast_slew = true;
            target_angle = theta_goal; // Mira o campo vetorial puramente para dentro do gol
        } else {
            // ── APF: Integra repulsão de obstáculos ao vetor espiral ─────────
            // Converte a direção do campo espiral em vetor unitário (atração)
            double spiral_x = std::cos(target_angle);
            double spiral_y = std::sin(target_angle);

            // Acumula forças de repulsão de robôs próximos
            double rep_x = 0.0, rep_y = 0.0;
            const double R_robot = 0.12;
            const double D_min   = 0.076;
            const double k_rob   = 0.08;

            auto addRepulsion = [&](double ox, double oy) {
                double rdx  = robot.x - ox;
                double rdy  = robot.y - oy;
                double rdist = std::hypot(rdx, rdy);
                if (rdist < R_robot && rdist > 1e-6) {
                    double sd  = std::max(rdist, D_min);
                    double mag = k_rob * (1.0/sd - 1.0/R_robot);
                    rep_x += mag * (rdx / rdist);
                    rep_y += mag * (rdy / rdist);
                }
            };
            for (const auto& ally : ctx.allies) {
                if (!ally.valid || ally.id == robot.id) continue;
                addRepulsion(ally.x, ally.y);
            }
            for (const auto& enemy : ctx.enemies) {
                if (!enemy.valid) continue;
                addRepulsion(enemy.x, enemy.y);
            }

            // Repulsão de paredes com Comportamento de Trilho Guia
            const double R_wall = 0.10;
            const double k_wall = 0.10;
            double d_left  = robot.x - ctx.field.min_x();
            double d_right = ctx.field.max_x() - robot.x;
            double d_bot   = robot.y - ctx.field.min_y();
            double d_top   = ctx.field.max_y() - robot.y;
            if (d_left  < R_wall && d_left  > 1e-6) rep_x += k_wall * (1.0/d_left  - 1.0/R_wall);
            if (d_right < R_wall && d_right > 1e-6) rep_x -= k_wall * (1.0/d_right - 1.0/R_wall);
            if (d_bot   < R_wall && d_bot   > 1e-6) rep_y += k_wall * (1.0/d_bot   - 1.0/R_wall);
            if (d_top   < R_wall && d_top   > 1e-6) rep_y -= k_wall * (1.0/d_top   - 1.0/R_wall);

            // Trilho Guia (Rail Behavior): impede que o bico aponte para a parede colada
            if (d_bot < 0.08) {
                rep_y += 0.5; // Empurra perpendicularmente para longe
                rep_x += (spiral_x >= 0 ? 1.0 : -1.0); // Força tangencial paralela à parede
            } else if (d_top < 0.08) {
                rep_y -= 0.5;
                rep_x += (spiral_x >= 0 ? 1.0 : -1.0);
            }
            
            if (d_left < 0.08) {
                rep_x += 0.5;
                rep_y += (spiral_y >= 0 ? 1.0 : -1.0);
            } else if (d_right < 0.08) {
                rep_x -= 0.5;
                rep_y += (spiral_y >= 0 ? 1.0 : -1.0);
            }

            // Componente tangencial de escape: escolhe o lado com maior espaço livre
            double rep_mag = std::hypot(rep_x, rep_y);
            if (rep_mag > 1e-4) {
                double tan_x =  -rep_y / rep_mag;
                double tan_y =   rep_x / rep_mag;
                double dot_pos = spiral_x * tan_x  + spiral_y * tan_y;
                double dot_neg = spiral_x * (-tan_x) + spiral_y * (-tan_y);
                double k_tan = 0.05;
                if (dot_pos >= dot_neg) { rep_x += k_tan * tan_x;  rep_y += k_tan * tan_y;  }
                else                    { rep_x += k_tan * (-tan_x); rep_y += k_tan * (-tan_y); }
            }

            // Soma espiral + repulsão → ângulo alvo corrigido
            double final_x = spiral_x + rep_x;
            double final_y = spiral_y + rep_y;
            target_angle = normalizeAngle(std::atan2(final_y, final_x));

            // Suaviza a velocidade na curva de aproximação
            double angle_diff = normalizeAngle(target_angle - robot.theta);
            v = std::max(1.2, v_max * std::cos(angle_diff));
            v = avoidCollisions(robot, ctx, v);
        }

        // ── 4. Cinemática de Orientação (P Angular) ───────────────────────
        double angle_err_robot = normalizeAngle(target_angle - robot.theta);
        
        // Estabilização de Singularidade: desativa atan2 terminal
        if (dist < 0.02) {
            angle_err_robot = 0.0;
        }

        double kp_angular = in_attack_cone ? 4.0 : 6.0;
        double omega_raw = kp_angular * angle_err_robot;

        // Rampa angular na aproximação espiral; na fase kamikaze usa omega pleno
        double omega = in_attack_cone
            ? std::clamp(omega_raw, -kp_angular * M_PI, kp_angular * M_PI)
            : applyAngularRamp(angle_err_robot, omega_raw, kp_angular * M_PI);

        // Otimização de giro: Se o erro for muito alto fora do cone de ataque, 
        // sacrifica velocidade linear para rotacionar mais rápido na entrada da espiral.
        if (!in_attack_cone && std::abs(angle_err_robot) > M_PI / 3.0) {
            v *= 0.3;
        }

        // ── 5. Perfil de Aceleração Trapezoidal (Linear) ──────────────────
        v = applyLinearRamp(v, last_v_, 5.0); // max accel 5.0 m/s^2
        last_v_ = v;

        // IMPORTANTE: Não zeramos mais a velocidade por distância (dist < 0.03).
        // Isso força o robô a atravessar a coordenada da bola empurrando-a violentamente.
        return clampCommand(RobotCommand::fromVW(robot.id, v, omega, 0.075, is_fast_slew));
    }

    std::string name() const override { return "VectorAttackerSkill"; }

private:
    double last_v_ = 0.0;
    double stuck_ref_x_ = 0.0;
    double stuck_ref_y_ = 0.0;
    int stuck_frames_ = 0;
    bool is_unsticking_ = false;
    int unstick_frames_ = 0;
};

} // namespace vss
