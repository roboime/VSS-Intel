#pragma once

#include "types.hpp"
#include <array>
#include <cstdint>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  GameContext
//
//  Equivale ao cluster "Game" do LabVIEW que era passado por todas as VIs.
//  Contém tudo que qualquer Skill/Tactic/Role/Play pode precisar para decidir.
//  É passado por const-ref — ninguém muda o contexto, só o nó de estratégia.
// ─────────────────────────────────────────────────────────────────────────────
struct GameContext {

    // ── Estado atual do jogo ──────────────────────────────────────────────
    GameSituation situation = GameSituation::HALT;
    uint32_t      frame     = 0;      // número do frame atual
    double        dt        = 0.016;  // segundos entre frames (≈60 Hz FIRASim)

    // ── Bola ──────────────────────────────────────────────────────────────
    BallState ball;

    // ── Nosso time (azul por padrão) ──────────────────────────────────────
    // VSS 3v3: índices 0, 1, 2
    std::array<RobotState, 3> allies;

    // ── Time adversário ───────────────────────────────────────────────────
    std::array<RobotState, 3> enemies;

    // ── Campo ─────────────────────────────────────────────────────────────
    FieldParams field;

    // ── Atribuição de papéis ──────────────────────────────────────────────
    // Qual índice de allies[] está em cada papel
    // -1 = papel não atribuído
    int8_t attacker_id  = 0;
    int8_t defender_id  = 1;
    int8_t goalkeeper_id = 2;

    // ── Lados do campo ────────────────────────────────────────────────────
    // +1 = atacamos para o lado positivo de X (gol adversário em +X)
    // -1 = atacamos para o lado negativo de X
    int8_t attack_dir = 1;

    // ── Pontuação ─────────────────────────────────────────────────────────
    uint8_t score_ally  = 0;
    uint8_t score_enemy = 0;

    // ─────────────────────────────────────────────────────────────────────
    //  Helpers de acesso rápido
    // ─────────────────────────────────────────────────────────────────────

    const RobotState& attacker()   const { return allies[attacker_id];   }
    const RobotState& defender()   const { return allies[defender_id];   }
    const RobotState& goalkeeper() const { return allies[goalkeeper_id]; }

    // Posição do gol adversário (onde queremos chutar)
    Point2D enemyGoalCenter() const {
        return { attack_dir * field.max_x(), 0.0 };
    }

    // Posição do nosso gol (que queremos defender)
    Point2D ourGoalCenter() const {
        return { -attack_dir * field.max_x(), 0.0 };
    }

    // Verifica se um ponto está dentro da área do goleiro adversário
    // Extraído do LabVIEW: "Caso para não deixar o robô entrar na área
    // junto com o goleiro"
    bool isInEnemyGoalArea(double x, double y) const {
        double area_x_start = attack_dir * (field.max_x() - field.area_length);
        bool in_x = (attack_dir > 0)
                    ? (x > area_x_start)
                    : (x < area_x_start);
        bool in_y = std::abs(y) < (field.area_width / 2.0);
        return in_x && in_y;
    }

    // Verifica se está na área do nosso goleiro
    bool isInOurGoalArea(double x, double y) const {
        double area_x_start = -attack_dir * (field.max_x() - field.area_length);
        bool in_x = (attack_dir > 0)
                    ? (x < area_x_start)
                    : (x > area_x_start);
        bool in_y = std::abs(y) < (field.area_width / 2.0);
        return in_x && in_y;
    }

    // Robô mais próximo da bola do nosso time
    uint8_t closestAllyToBall() const {
        double best = 1e9;
        uint8_t idx = 0;
        for (uint8_t i = 0; i < 3; ++i) {
            double d = std::hypot(allies[i].x - ball.x,
                                  allies[i].y - ball.y);
            if (d < best) { best = d; idx = i; }
        }
        return idx;
    }

    // Ângulo do robô até a bola
    double angleToBall(const RobotState& robot) const {
        return std::atan2(ball.y - robot.y, ball.x - robot.x);
    }

    // Ângulo do robô até o gol adversário
    double angleToEnemyGoal(const RobotState& robot) const {
        auto goal = enemyGoalCenter();
        return std::atan2(goal.y - robot.y, goal.x - robot.x);
    }
};

} // namespace vss
