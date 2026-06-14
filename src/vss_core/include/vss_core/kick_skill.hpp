#pragma once

#include "skill.hpp"
#include <cmath>
#include <algorithm>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  KickSkill
//
//  Tradução direta do KickSkillVSS.vi — extraído das 4 fotos do LabVIEW.
//
//  ESTADO INTERNO: máquina de 4 estados
//    ALIGN   → gira até apontar para o gol
//    CHARGE  → avança em linha reta em direção à bola (GoToKick)
//    UNSTICK → recuo linear bruto (25fr) + giro de escape (20fr)
//    DONE    → chute executado, skill finalizada
//
//  SALVAGUARDAS ANTI-TRAVAMENTO NO CHARGE:
//
//  1. isFrontBlocked() — Sistema Tridente (3 probes paralelos):
//     · Probe Centro  : 8.75 cm à frente no eixo theta
//     · Probe Esquerda: offset lateral -3.75 cm (quina esq.), 6.0 cm à frente
//     · Probe Direita : offset lateral +3.75 cm (quina dir.), 6.0 cm à frente
//     Cobre toda a face frontal do chassi 7.5×7.5 cm. Um único probe central
//     (raycast) dava falso-negativo quando a quina lateral já estava colida.
//
//  2. Stuck detection: 45 frames com |v| > 1.5 m/s e deslocamento < 3 cm
//     → transita para UNSTICK
//
//  ANÁLISE DA RAMPA vs. UNSTICK:
//     fromVW() emite comandos de roda diretamente, sem passar por
//     applyLinearRamp(). O impulso de -3.0 m/s é aplicado instantaneamente
//     no primeiro frame do UNSTICK — sem suavização.
// ─────────────────────────────────────────────────────────────────────────────

struct KickSkillParams {
    // Thresholds de parede (LabVIEW: 90 e 70 unidades → metros)
    double near_vertical_wall   = 0.09;
    double near_horizontal_wall = 0.07;

    // Velocidade máxima de chute
    double kick_speed = 2.5;              // m/s

    // Fator de escala do impulso inicial
    double kick_impulse_factor = 0.8;

    // Threshold de alinhamento
    double alignment_threshold_rad = 0.30;  // ~17.2 graus

    // Ganho de rotação durante ALIGN
    double kp_rotation = 6.0;

    // Distância de conclusão do chute
    double kick_done_dist = 0.04;         // 4 cm — contato físico firme

    // Base das rodas
    double wheel_base = 0.075;
};

class KickSkill final : public Skill {
public:
    using Params = KickSkillParams;

    enum class Mode { NORMAL, PENALTY };

    explicit KickSkill(Params p = Params(), Mode m = Mode::NORMAL)
        : params_(p), mode_(m) {}

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
        state_           = State::ALIGN;
        frames_charging_ = 0;
        stuck_frames_    = 0;
        unstick_frames_  = 0;
        last_kick_v_     = 0.0;
        stuck_ref_x_     = robot.x;
        stuck_ref_y_     = robot.y;
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        switch (state_) {
            case State::ALIGN:   return doAlign(robot, ctx);
            case State::CHARGE:  return doCharge(robot, ctx);
            case State::UNSTICK: return doUnstick(robot, ctx);
            case State::DONE:
                finished_ = true;
                return RobotCommand::stop(robot.id);
        }
        return RobotCommand::stop(robot.id);
    }

    std::string name() const override {
        return mode_ == Mode::PENALTY ? "KickSkill[PENALTY]" : "KickSkill";
    }

private:
    Params params_;
    Mode   mode_;

    enum class State { ALIGN, CHARGE, UNSTICK, DONE };
    State state_           = State::ALIGN;
    int   frames_charging_ = 0;

    // Estado de Unstick
    double stuck_ref_x_    = 0.0;
    double stuck_ref_y_    = 0.0;
    int    stuck_frames_   = 0;
    int    unstick_frames_ = 0;
    double last_kick_v_    = 0.0;

    // ─────────────────────────────────────────────────────────────────────
    //  isFrontBlocked — Sistema Tridente (3 probes paralelos)
    //
    //  ANÁLISE DO FALSO-NEGATIVO DO RAYCAST CENTRAL:
    //  O chassi tem 7.5×7.5 cm. As quinas ficam a √(3.75²+3.75²) = 5.30 cm
    //  do centro. Em abordagem diagonal (ex: 30°), a quina lateral pode estar
    //  a 0 cm do obstáculo enquanto o probe central ainda lê 5+ cm de clearance.
    //
    //  Solução: 3 probes cobrem toda a face frontal do cubo:
    //
    //    Probe Esquerda           Probe Centro           Probe Direita
    //    (quina esq. + 6 cm)    (eixo θ + 8.75 cm)    (quina dir. + 6 cm)
    //         ●                       ●                       ●
    //         |                       |                       |
    //    ┌────┼───────────────────────┼───────────────────────┼────┐
    //    │    ↑ -3.75 cm             centro             +3.75 cm  │
    //    │              FACE FRONTAL DO CHASSI (7.5 cm)           │
    //    └────────────────────────────────────────────────────────┘
    //
    //  Margem do probe lateral: 6.0 cm (além da quina que já está a 5.30 cm).
    //  Margem do probe central: 8.75 cm = 3.75 cm (half-chassis) + 5.0 cm.
    //
    //  Adversários próximos da bola (< 10 cm) são ignorados — são o alvo.
    // ─────────────────────────────────────────────────────────────────────
    bool isFrontBlocked(const RobotState& robot, const GameContext& ctx) const {
        const double cos_t = std::cos(robot.theta);
        const double sin_t = std::sin(robot.theta);

        // Vetores unitários: frente (eixo theta) e lateral (perpendicular)
        // lat_x = -sin(theta), lat_y = cos(theta)  →  lado esquerdo
        const double lat_x = -sin_t;
        const double lat_y =  cos_t;

        // Metade do chassi (offset lateral das quinas)
        const double half_chassis = 0.0375;  // 3.75 cm

        // ── Três pontos de sondagem ───────────────────────────────────────
        // Centro: 8.75 cm à frente (half_chassis + 5 cm de margem)
        // Laterais: quina ± 3.75 cm, 6 cm à frente da quina
        const double fwd_center = half_chassis + 0.05;  // 8.75 cm
        const double fwd_corner = 0.06;                  // 6 cm além da quina

        struct Probe { double x, y; };
        Probe probes[3] = {
            // Centro
            { robot.x + fwd_center * cos_t,
              robot.y + fwd_center * sin_t },
            // Quina Esquerda
            { robot.x - half_chassis * lat_x + fwd_corner * cos_t,
              robot.y - half_chassis * lat_y + fwd_corner * sin_t },
            // Quina Direita
            { robot.x + half_chassis * lat_x + fwd_corner * cos_t,
              robot.y + half_chassis * lat_y + fwd_corner * sin_t },
        };

        const double wall_margin = 0.01;

        for (const auto& p : probes) {
            // ── Parede ────────────────────────────────────────────────────
            if (p.x < ctx.field.min_x() + wall_margin ||
                p.x > ctx.field.max_x() - wall_margin ||
                p.y < ctx.field.min_y() + wall_margin ||
                p.y > ctx.field.max_y() - wall_margin) {
                return true;
            }
        }

        // ── Robôs (aliados e adversários) ──────────────────────────────
        // Limiar: diâmetro físico do cubo (7.6 cm) + 5 cm de margem
        const double robot_block_dist = 0.076 + 0.05;
        const double cone_half = M_PI / 3.0;  // cone frontal de 60°

        for (const auto& ally : ctx.allies) {
            if (!ally.valid || ally.id == robot.id) continue;
            double dist = std::hypot(ally.x - robot.x, ally.y - robot.y);
            if (dist < robot_block_dist) {
                double ang = std::atan2(ally.y - robot.y, ally.x - robot.x);
                if (std::abs(normalizeAngle(ang - robot.theta)) < cone_half)
                    return true;
            }
        }
        for (const auto& enemy : ctx.enemies) {
            if (!enemy.valid) continue;
            // Ignora adversários próximos da bola — são o alvo do chute
            double enemy_to_ball = std::hypot(enemy.x - ctx.ball.x,
                                               enemy.y - ctx.ball.y);
            if (enemy_to_ball < 0.10) continue;

            double dist = std::hypot(enemy.x - robot.x, enemy.y - robot.y);
            if (dist < robot_block_dist) {
                double ang = std::atan2(enemy.y - robot.y, enemy.x - robot.x);
                if (std::abs(normalizeAngle(ang - robot.theta)) < cone_half)
                    return true;
            }
        }
        return false;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Fase 1: ALIGN
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doAlign(const RobotState& robot, const GameContext& ctx) {
        auto goal = ctx.enemyGoalCenter();

        double ang_to_ball = std::atan2(ctx.ball.y - robot.y,
                                         ctx.ball.x - robot.x);
        double ang_to_goal = std::atan2(goal.y - ctx.ball.y,
                                         goal.x - ctx.ball.x);

        double desired_angle = (mode_ == Mode::PENALTY) ? ang_to_goal : ang_to_ball;
        double angle_err = normalizeAngle(desired_angle - robot.theta);

        // Verificação de paredes
        bool near_vertical   = (ctx.field.max_x() - std::abs(robot.x))
                                < params_.near_vertical_wall;
        bool near_horizontal = (ctx.field.max_y() - std::abs(robot.y))
                                < params_.near_horizontal_wall;

        double rotation_dir;
        if (near_vertical || near_horizontal) {
            rotation_dir = (ctx.ball.y < robot.y) ? 1.0 : -1.0;
        } else {
            rotation_dir = (ctx.ball.y < robot.y) ? -1.0 : 1.0;
        }

        if (std::abs(angle_err) < params_.alignment_threshold_rad) {
            state_           = State::CHARGE;
            frames_charging_ = 0;
            stuck_frames_    = 0;
            stuck_ref_x_     = robot.x;
            stuck_ref_y_     = robot.y;
            last_kick_v_     = 0.0;
            return doCharge(robot, ctx);
        }

        double omega_raw = rotation_dir * params_.kp_rotation * std::abs(angle_err);
        double omega = applyAngularRamp(angle_err, omega_raw,
                                        params_.kick_speed / params_.wheel_base * 0.5);

        return clampCommand(RobotCommand::fromVW(robot.id, 0.0, omega,
                                                  params_.wheel_base));
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Fase 2: CHARGE
    //
    //  Salvaguardas:
    //    · isFrontBlocked() (Tridente): para força linear se chassi está
    //      bloqueado. Mantém apenas correção angular.
    //    · Stuck detection: 45 frames → UNSTICK
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doCharge(const RobotState& robot, const GameContext& ctx) {
        double dx   = ctx.ball.x - robot.x;
        double dy   = ctx.ball.y - robot.y;
        double dist = std::hypot(dx, dy);

        if (dist < params_.kick_done_dist) {
            state_       = State::DONE;
            finished_    = true;
            last_kick_v_ = params_.kick_speed;
            return clampCommand(
                RobotCommand::fromVW(robot.id,
                                     params_.kick_speed, 0.0,
                                     params_.wheel_base));
        }

        frames_charging_++;

        // ── Stuck detection ───────────────────────────────────────────────
        double dist_moved = std::hypot(robot.x - stuck_ref_x_,
                                        robot.y - stuck_ref_y_);
        if (last_kick_v_ > 1.5) {
            stuck_frames_++;
            if (dist_moved > 0.030) {
                stuck_frames_ = 0;
                stuck_ref_x_  = robot.x;
                stuck_ref_y_  = robot.y;
            }
        } else {
            stuck_frames_ = 0;
            stuck_ref_x_  = robot.x;
            stuck_ref_y_  = robot.y;
        }

        if (stuck_frames_ > 45) {
            state_          = State::UNSTICK;
            stuck_frames_   = 0;
            unstick_frames_ = 0;
            last_kick_v_    = -3.0;
            return clampCommand(RobotCommand::fromVW(robot.id, -3.0, 0.0,
                                                      params_.wheel_base));
        }

        // ── isFrontBlocked (Tridente) ─────────────────────────────────────
        if (isFrontBlocked(robot, ctx)) {
            last_kick_v_ = 0.0;
            double angle_fwd = normalizeAngle(
                std::atan2(ctx.ball.y - robot.y, ctx.ball.x - robot.x) - robot.theta);
            return clampCommand(RobotCommand::fromVW(robot.id, 0.0, 3.0 * angle_fwd,
                                                      params_.wheel_base));
        }

        // ── Avanço ────────────────────────────────────────────────────────
        double angle_err = normalizeAngle(std::atan2(dy, dx) - robot.theta);

        double v = params_.kick_speed * params_.kick_impulse_factor
                   + params_.kick_speed * (1.0 - params_.kick_impulse_factor)
                     * std::min(1.0, frames_charging_ / 10.0);
        v = std::min(v, params_.kick_speed);
        last_kick_v_ = v;

        double omega = 3.0 * angle_err;
        if (std::abs(angle_err) < 0.05) omega = 0.0;

        return clampCommand(RobotCommand::fromVW(robot.id, v, omega,
                                                  params_.wheel_base));
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Fase 3: UNSTICK — Recuo bruto + giro de escape
    //
    //  ANÁLISE DA RAMPA: fromVW() emite velocidade de roda DIRETAMENTE
    //  sem passar por applyLinearRamp. O -3.0 m/s é aplicado no frame 1
    //  sem qualquer suavização — impulso bruto para romper encaixes.
    //
    //  FASE 1 (25 frames, -3.0 m/s, ω=0):
    //    Recuo bruto puro. A 3.0 m/s e 60 Hz, o impulso inicial é suficiente
    //    para separar as quinas do cubo (gap > 7.5 cm em ~3-4 frames).
    //    25 frames absorvem os primeiros frames de resistência do simulador.
    //
    //  FASE 2 (20 frames, -1.0 m/s, ω=±12):
    //    Giro com clearance já garantido. Sem gap, o giro prenderia as quinas.
    //
    //  Retorna ao ALIGN para reavaliação completa antes do próximo chute.
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doUnstick(const RobotState& robot, const GameContext& ctx) {
        (void)ctx;
        unstick_frames_++;

        if (unstick_frames_ > 45) {
            state_          = State::ALIGN;
            stuck_frames_   = 0;
            unstick_frames_ = 0;
            last_kick_v_    = 0.0;
            stuck_ref_x_    = robot.x;
            stuck_ref_y_    = robot.y;
            return doAlign(robot, ctx);
        }

        if (unstick_frames_ <= 25) {
            // FASE 1: Impulso bruto de ré — SEM omega, SEM rampa interna.
            // [FIX C6] is_fast_slew=true: slew externo usa Δv=0.45/frame
            // em vez de 0.15/frame. Sem o flag, o StrategyNode limitava o
            // impulso a ~-1.75 m/s reais (vs. -3.0 projetados) porque
            // from last_cmd ~+2.0 até -3.0 levaria 33 frames @0.15/frame,
            // mas a fase 1 só dura 25 frames.
            last_kick_v_ = -3.0;
            return clampCommand(RobotCommand::fromVW(robot.id, -3.0, 0.0,
                                                      params_.wheel_base, true));
        } else {
            // FASE 2: Giro com ré leve — clearance já garantido pela FASE 1.
            // [FIX C6] Mantemos is_fast_slew=true na fase 2 também:
            // o -1.0 m/s precisa ser atingido rapidamente para o giro ser efetivo.
            double spin_dir = (robot.y > 0.0) ? 1.0 : -1.0;
            last_kick_v_ = -1.0;
            return clampCommand(RobotCommand::fromVW(robot.id, -1.0, 12.0 * spin_dir,
                                                      params_.wheel_base, true));
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  KickPenaltySkill  —  "caso penalti" do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class KickPenaltySkill final : public Skill {
public:
    explicit KickPenaltySkill() : inner_(KickSkillParams{}, KickSkill::Mode::PENALTY) {}

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
        inner_.init(robot, ctx);
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        auto cmd = inner_.execute(robot, ctx);
        if (inner_.isFinished()) finished_ = true;
        return cmd;
    }

    std::string name() const override { return "KickPenaltySkill"; }

private:
    KickSkill inner_;
};

} // namespace vss
