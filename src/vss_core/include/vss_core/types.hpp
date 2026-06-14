#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <array>
#include <algorithm>

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

// ─────────────────────────────────────────────────────────────────────────────
//  Álgebra linear 2D — funções livres usadas pelas Skills
// ─────────────────────────────────────────────────────────────────────────────

/// Produto interno 2D
inline double dot2D(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}

/// Magnitude de um vetor 2D
inline double mag2D(double x, double y) {
    return std::hypot(x, y);
}

/// Normaliza in-place; retorna false se o vetor é zero (não normaliza)
inline bool normalize2D(double& x, double& y) {
    double m = mag2D(x, y);
    if (m < 1e-9) return false;
    x /= m; y /= m;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  namespace geo  —  Constantes físicas reais do VSS 3v3
//
//  Fontes:
//    · Robôs: cubos de 75 × 75 mm (regra IEEE/RoboCup VSS)
//    · Bola:  diâmetro oficial = 43 mm → raio = 21.5 mm
//    · Campo: 150 × 130 cm (FIRASim default, igual ao FieldParams acima)
// ─────────────────────────────────────────────────────────────────────────────
namespace geo {

    constexpr double ROBOT_SIDE        = 0.075;              // 7.5 cm
    constexpr double ROBOT_RADIUS      = ROBOT_SIDE / 2.0;  // 3.75 cm (centro→face)
    // Distância centro→quina: √2/2 × 7.5 cm ≈ 5.30 cm
    constexpr double ROBOT_CORNER_DIST = ROBOT_SIDE * 0.7071067811865476;

    constexpr double BALL_RADIUS       = 0.0215;  // 2.15 cm (raio oficial VSS)

    // Distância mínima de segurança para target_behind_ball:
    //   raio do robô (frente) + raio da bola + margem de 3 cm
    //   = 3.75 + 2.15 + 3.0 = 8.9 cm  → arredondado para 9 cm
    constexpr double BEHIND_BALL_NEAR  = ROBOT_RADIUS + BALL_RADIUS + 0.03;

    // Distância mais conservadora usada na fase APPROACH (12 cm)
    //   Permite que o robô se posicione claramente atrás sem risco de toque
    constexpr double BEHIND_BALL_DIST  = 0.12;

    // Margem mínima de qualquer alvo calculado em relação às paredes
    constexpr double WALL_MARGIN       = 0.05;  // 5 cm

    // ── Funções geométricas ───────────────────────────────────────────────

    /// Calcula o ponto 'target_behind_ball':
    ///   Vetor unitário do CENTRO DO GOL → BOLA, projetado ATRÁS da bola
    ///   a uma distância 'dist_behind'.
    ///   Resultado é clampado a WALL_MARGIN de qualquer parede.
    ///
    ///   Geometria:
    ///     dir = normalize(ball - goal)
    ///     target = ball + dir * dist_behind
    ///
    ///   Parâmetros:
    ///     ball_x/y     — posição atual da bola
    ///     goal_x/y     — centro do gol adversário
    ///     dist_behind  — distância de recuo (use geo::BEHIND_BALL_DIST)
    ///     field_min_x/y, field_max_x/y — limites do campo
    ///
    inline Point2D calcTargetBehindBall(
        double ball_x, double ball_y,
        double goal_x, double goal_y,
        double dist_behind,
        double field_min_x, double field_max_x,
        double field_min_y, double field_max_y)
    {
        // Vetor gol → bola (direção em que o robô empurrará a bola para o gol)
        double dx = ball_x - goal_x;
        double dy = ball_y - goal_y;

        // Se a bola está exatamente no gol (caso degenerado), vai para trás no eixo X
        if (!normalize2D(dx, dy)) {
            dx = 1.0; dy = 0.0;
        }

        // Ponto atrás da bola (lado oposto ao gol)
        double tx = ball_x + dx * dist_behind;
        double ty = ball_y + dy * dist_behind;

        // Clamp rígido: alvo nunca pode estar a menos de WALL_MARGIN de qualquer parede
        tx = std::clamp(tx, field_min_x + WALL_MARGIN, field_max_x - WALL_MARGIN);
        ty = std::clamp(ty, field_min_y + WALL_MARGIN, field_max_y - WALL_MARGIN);

        return {tx, ty};
    }

    /// Retorna true se o robô está entre a bola e o gol adversário.
    ///
    ///   Critério: o produto interno entre
    ///     (bola → robô)  e  (bola → gol)
    ///   é POSITIVO, ou seja, o robô está no hemisfério do gol a partir da bola.
    ///
    ///   Isso indica que o robô está no caminho errado e precisa orbitar.
    inline bool isRobotBetweenBallAndGoal(
        double robot_x, double robot_y,
        double ball_x,  double ball_y,
        double goal_x,  double goal_y)
    {
        // vetor bola → robô
        double br_x = robot_x - ball_x;
        double br_y = robot_y - ball_y;
        // vetor bola → gol
        double bg_x = goal_x - ball_x;
        double bg_y = goal_y - ball_y;
        // Se o produto interno é positivo, o robô está do mesmo lado que o gol
        return dot2D(br_x, br_y, bg_x, bg_y) > 0.0;
    }

} // namespace geo

} // namespace vss
