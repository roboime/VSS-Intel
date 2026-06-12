#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <array>

namespace vss {

// ─────────────────────────────────────────────
//  Campo VSS 3v3  (unidades: metros)
//  FIRASim usa metros. Valores retirados do LabVIEW:
//  field_length ≈ 1.50 m  (150 cm  / ~960 unidades LabVIEW × escala)
//  field_width  ≈ 1.30 m  (130 cm)
//  goal_width   ≈ 0.40 m
//  goal_area_x  ≈ 0.15 m  (profundidade da área)
// ─────────────────────────────────────────────
struct FieldParams {
    double length      = 1.50;   // eixo X total
    double width       = 1.30;   // eixo Y total
    double goal_width  = 0.40;
    double goal_depth  = 0.10;
    double area_length = 0.15;   // profundidade da área do goleiro
    double area_width  = 0.50;   // largura da área do goleiro

    // Limites do campo (centro = origem)
    double max_x() const { return  length / 2.0; }
    double min_x() const { return -length / 2.0; }
    double max_y() const { return  width  / 2.0; }
    double min_y() const { return -width  / 2.0; }
};

// ─────────────────────────────────────────────
//  Estado de um robô  (vindo do FIRASim / EKF)
// ─────────────────────────────────────────────
struct RobotState {
    uint8_t id     = 0;
    double  x      = 0.0;   // metros
    double  y      = 0.0;
    double  theta  = 0.0;   // radianos  [-π, π]
    double  vx     = 0.0;   // m/s
    double  vy     = 0.0;
    double  omega  = 0.0;   // rad/s
    bool    valid  = false;
};

// ─────────────────────────────────────────────
//  Estado da bola  (vindo do EKF)
// ─────────────────────────────────────────────
struct BallState {
    double x  = 0.0;
    double y  = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    bool   valid = false;
};

// ─────────────────────────────────────────────
//  Comando de saída para FIRASim
//  FIRASim recebe velocidade individual de cada roda (m/s)
// ─────────────────────────────────────────────
struct RobotCommand {
    uint8_t id          = 0;
    double  wheel_left  = 0.0;   // m/s
    double  wheel_right = 0.0;   // m/s
    bool    is_fast_slew = false; // Flag para desbloqueio dinâmico do acelerador

    // Fábrica a partir de v linear + ω angular
    // L = distância entre rodas (metros)
    static RobotCommand fromVW(uint8_t id, double v, double omega,
                               double wheel_base = 0.075, bool is_fast = false)
    {
        RobotCommand cmd;
        cmd.id          = id;
        cmd.wheel_right = v + (omega * wheel_base / 2.0);
        cmd.wheel_left  = v - (omega * wheel_base / 2.0);
        cmd.is_fast_slew = is_fast;
        return cmd;
    }

    // Comando de parada
    static RobotCommand stop(uint8_t id) {
        RobotCommand cmd;
        cmd.id = id;
        return cmd;
    }
};

// ─────────────────────────────────────────────
//  Situação do jogo  (vinda do VSSRef / árbitro)
//  Equivale ao GameState do LabVIEW
// ─────────────────────────────────────────────
enum class GameSituation : uint8_t {
    HALT = 0,
    NORMAL_GAME,
    KICKOFF_ALLY,
    KICKOFF_ENEMY,
    FREEBALL_ALLY,
    FREEBALL_ENEMY,
    GOAL_KICK_ALLY,
    GOAL_KICK_ENEMY,
    PENALTY_ALLY,
    PENALTY_ENEMY,
    DIRECT_KICK_ALLY,
    DIRECT_KICK_ENEMY,
    INDIRECT_KICK_ALLY,
    INDIRECT_KICK_ENEMY,
    TIMEOUT
};

std::string toString(GameSituation s);

// ─────────────────────────────────────────────
//  Ponto 2D simples
// ─────────────────────────────────────────────
struct Point2D {
    double x = 0.0;
    double y = 0.0;

    double distTo(const Point2D& o) const {
        return std::hypot(x - o.x, y - o.y);
    }

    double distTo(double ox, double oy) const {
        return std::hypot(x - ox, y - oy);
    }
};

// ─────────────────────────────────────────────
//  Utilitários de ângulo
// ─────────────────────────────────────────────
inline double normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline double angleBetween(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

} // namespace vss
