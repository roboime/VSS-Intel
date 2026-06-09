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
//  LÓGICA COMPLETA IDENTIFICADA:
//
//  1. DETECÇÃO DE PAREDES (limiares em unidades LabVIEW convertidos p/ metros):
//     · "Perto de paredes verticais?"   → threshold 90 unid. ≈ 0.09 m
//     · "Perto de paredes horizontais?" → threshold 70 unid. ≈ 0.07 m
//     (campo VSS ≈ 1500×1300 unidades LabVIEW → 1.5×1.3 m)
//
//  2. DECISÃO DE ROTAÇÃO ("Case de decisão da rotação"):
//     · -1 = sentido horário       (Rotate Clockwise)
//     · +1 = sentido anti-horário  (Rotate anti-clockwise)
//     · Condição: "y bola < y robo?" determina qual sentido girar
//       → bola abaixo do robô  → rotaciona horário (gira para baixo)
//       → bola acima do robô   → rotaciona anti-horário (gira para cima)
//     · Perto de parede vertical/horizontal modifica esse comportamento
//       para evitar travar no canto
//
//  3. QUANDO ALINHADO → "Robo vai pra bola":
//     · Entra no case True do alinhamento
//     · GoToKick com dest_x = -200 unid. ≈ -0.20 m (relativo)
//                    dest_y = -200 unid. ≈ -0.20 m (relativo)
//     · vx_desired = 1000 (velocidade máxima de chute!)
//     · Fator 0.5 + 200 → escala do impulso de chute
//
//  4. CASO ESPECIAL PENALTI:
//     · "caso penalti" → branch separado com KickPenaltyAlly
//     · Verifica VssPlay e KeeperTeam para identificar situação
//
//  ESTADO INTERNO: máquina de 3 estados
//    ALIGN  → gira até apontar para o gol
//    CHARGE → avança em linha reta em direção à bola (GoToKick)
//    DONE   → chute executado, skill finalizada
// ─────────────────────────────────────────────────────────────────────────────

struct KickSkillParams {
    // Thresholds de parede (LabVIEW: 90 e 70 unidades → metros)
    double near_vertical_wall   = 0.09;   // "Perto de paredes verticais?"
    double near_horizontal_wall = 0.07;   // "Perto de paredes horizontais?"

    // Velocidade máxima de chute (LabVIEW: vx_desired = 1000)
    // Normalizado para m/s do FIRASim
    double kick_speed = 1.5;              // m/s (velocidade máxima das rodas)

    // Fator de escala do impulso (LabVIEW: 0.5 × 200)
    double kick_impulse_factor = 0.5;

    // Threshold de alinhamento para considerar "apontando para o gol"
    // Extraído do fluxo: entra em "Robo vai pra bola" quando alinhado
    double alignment_threshold_rad = 0.15;  // ~8.6 graus

    // Ganho de rotação durante a fase ALIGN
    double kp_rotation = 5.0;

    // Distância para considerar que o chute foi executado
    double kick_done_dist = 0.06;   // metros da bola

    // Base das rodas
    double wheel_base = 0.075;
};

class KickSkill final : public Skill {
public:
    using Params = KickSkillParams;

    // Modo: chute normal ou penalti
    enum class Mode { NORMAL, PENALTY };

    explicit KickSkill(Params p = Params(), Mode m = Mode::NORMAL)
        : params_(p), mode_(m) {}

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
        state_  = State::ALIGN;
        frames_charging_ = 0;
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        switch (state_) {
            case State::ALIGN:  return doAlign(robot, ctx);
            case State::CHARGE: return doCharge(robot, ctx);
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

    enum class State { ALIGN, CHARGE, DONE };
    State   state_           = State::ALIGN;
    int     frames_charging_ = 0;

    // ─────────────────────────────────────────────────────────────────────
    //  Fase 1: ALIGN
    //  "Case de decisão da rotação"
    //  Gira o robô até apontar para o gol, considerando paredes próximas.
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doAlign(const RobotState& robot, const GameContext& ctx) {
        auto goal = ctx.enemyGoalCenter();

        // Ângulo desejado: robô → gol passando pela bola
        double ang_to_ball = std::atan2(ctx.ball.y - robot.y,
                                         ctx.ball.x - robot.x);
        double ang_to_goal = std::atan2(goal.y - ctx.ball.y,
                                         goal.x - ctx.ball.x);

        // Alvo de orientação: alinhar bola → gol
        // Em penalti, aponta direto para o gol sem considerar a bola
        double desired_angle = (mode_ == Mode::PENALTY)
                               ? ang_to_goal
                               : ang_to_ball;

        double angle_err = normalizeAngle(desired_angle - robot.theta);

        // ── Verificação de paredes ────────────────────────────────────────
        // "Perto de paredes verticais?"  (paredes em X)
        bool near_vertical   = (ctx.field.max_x() - std::abs(robot.x))
                                < params_.near_vertical_wall;
        // "Perto de paredes horizontais?" (paredes em Y)
        bool near_horizontal = (ctx.field.max_y() - std::abs(robot.y))
                                < params_.near_horizontal_wall;

        // ── Decisão de sentido de rotação ─────────────────────────────────
        // "y bola < y robo?" → horário (-1) ou anti-horário (+1)
        // Quando perto de parede, inverte lógica para não travar no canto
        double rotation_dir;
        if (near_vertical || near_horizontal) {
            // Perto de parede: força rotação contrária para se afastar
            rotation_dir = (ctx.ball.y < robot.y) ? 1.0 : -1.0;
        } else {
            // Lógica normal do LabVIEW:
            // bola abaixo → horário (-1) = gira para alcançar a bola
            rotation_dir = (ctx.ball.y < robot.y) ? -1.0 : 1.0;
        }

        // ── Alinhado? → transita para CHARGE ──────────────────────────────
        if (std::abs(angle_err) < params_.alignment_threshold_rad) {
            state_ = State::CHARGE;
            return doCharge(robot, ctx);
        }

        // ── Rotaciona no lugar ─────────────────────────────────────────────
        // usa rotation_dir para sobrescrever o sinal do controlador P
        // (LabVIEW: -1=horário, +1=anti-horário)
        double omega = rotation_dir * params_.kp_rotation
                       * std::abs(angle_err);
        omega = std::clamp(omega, -params_.kick_speed / params_.wheel_base * 0.5,
                                   params_.kick_speed / params_.wheel_base * 0.5);

        return clampCommand(RobotCommand::fromVW(robot.id, 0.0, omega,
                                                  params_.wheel_base));
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Fase 2: CHARGE  ("Robo vai pra bola" / GoToKick)
    //  LabVIEW: dest_x = -200, dest_y = -200, vx_desired = 1000
    //  Robô avança em linha reta em direção à bola com velocidade máxima.
    //  O fator 0.5 × 200 define a magnitude do impulso inicial.
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doCharge(const RobotState& robot, const GameContext& ctx) {
        double dx   = ctx.ball.x - robot.x;
        double dy   = ctx.ball.y - robot.y;
        double dist = std::hypot(dx, dy);

        // Chute executado: robô chegou perto o suficiente da bola
        if (dist < params_.kick_done_dist) {
            state_   = State::DONE;
            finished_ = true;
            // Último frame: máxima velocidade para garantir o chute
            return clampCommand(
                RobotCommand::fromVW(robot.id,
                                     params_.kick_speed,
                                     0.0,
                                     params_.wheel_base));
        }

        frames_charging_++;

        // Mantém correção angular leve durante a corrida
        double angle_err = normalizeAngle(std::atan2(dy, dx) - robot.theta);

        // vx_desired = 1000 no LabVIEW → máxima velocidade
        // impulso_factor × 200 → escala do arranque
        double v = params_.kick_speed * params_.kick_impulse_factor
                   + params_.kick_speed * (1.0 - params_.kick_impulse_factor)
                     * std::min(1.0, frames_charging_ / 10.0);

        // Velocidade total = máxima quando frames > 10
        v = std::min(v, params_.kick_speed);

        double omega = 3.0 * angle_err;  // correção suave

        return clampCommand(RobotCommand::fromVW(robot.id, v, omega,
                                                  params_.wheel_base));
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  KickPenaltySkill  —  "caso penalti" do LabVIEW
//  Variante do KickSkill com alinhamento direto ao gol e chute mais forte.
//  Identificado no diagrama: branch KickPenaltyAlly com VssPlay/KeeperTeam.
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
