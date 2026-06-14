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
        // ── 0. Rotina de Anti-Travamento (Unstick) Sequencial ──────────────
        //
        // [AUDITORIA GEOMÉTRICA] Análise de por que o recuo de -2.0 m/s falhava
        // com obstáculos imóveis (robôs amarelos = parede rígida):
        //
        // Problema 1 — Força de encaixe x Força de tração:
        //   Quando o cubo de 7.5 cm aborda o obstáculo em ângulo obliquo,
        //   a quina penetra e cria um "cunha" mecânica. Para sair, a força
        //   tangencial de tração precisa superar a força normal na quina.
        //   -2.0 m/s pode não gerar impulso suficiente no primeiro frame para
        //   romper o engate antes o simulador estabilizar o contato.
        //
        // Problema 2 — Duração vs. Clearance real:
        //   A 3.0 m/s e 60 Hz: deslocamento por frame = 3.0 × 0.01667 = 5.0 cm.
        //   Após 3-4 frames já há clearance > 7.5 cm (chassi inteiro).
        //   Com 25 frames temos folga para absorver os primeiros frames de
        //   resistência do simulador de física.
        //
        // Correção: -3.0 m/s brutos (sem suavização) por 25 frames.
        // Nota: fromVW() bypassa qualquer rampa externa — o comando vai
        // direto ao simulador sem passar por applyLinearRamp.
        if (is_unsticking_) {
            unstick_frames_++;
            if (unstick_frames_ > 45) {
                // Sequência completa: reseta tudo e retoma navegação normal
                is_unsticking_ = false;
                stuck_frames_ = 0;
                stuck_ref_x_ = robot.x;
                stuck_ref_y_ = robot.y;
            } else {
                if (unstick_frames_ <= 25) {
                    // FASE 1: DESCOLAR — Marça à ré linear BRUTA (sem omega)
                    // [FIX C6] is_fast_slew=true: slew externo usa Δv=0.45/frame
                    // em vez de 0.15/frame, garantindo que -3.0 m/s seja
                    // atingido em ~7 frames (vs. ~33 frames sem o flag).
                    last_v_ = -3.0;
                    return clampCommand(RobotCommand::fromVW(robot.id, -3.0, 0.0, 0.075, true));
                } else {
                    // FASE 2: GIRAR E ESCAPAR — Ré leve + giro violento
                    // [FIX C6] Mantemos is_fast_slew=true: -1.0 m/s precisa
                    // ser atingido rapidamente para o giro ser efetivo.
                    double spin_dir = (robot.y > 0.0) ? 1.0 : -1.0;
                    last_v_ = -1.0;
                    return clampCommand(RobotCommand::fromVW(robot.id, -1.0, 12.0 * spin_dir, 0.075, true));
                }
            }
        }

        // ── Rastreamento de deslocamento por janela de tempo (odometria via visão)
        if (stuck_frames_ == 0) {
            stuck_ref_x_ = robot.x;
            stuck_ref_y_ = robot.y;
        }

        double real_dist_moved = std::hypot(robot.x - stuck_ref_x_, robot.y - stuck_ref_y_);

        // [FIX REGRESSÃO] Threshold de velocidade corrigido: 0.3 → 1.5 m/s.
        // O valor 0.3 capturava a rampa trapezoidal (applyLinearRamp a 5.0 m/s²):
        // após apenas 4 frames a roda já marcava |v| > 0.3 durante aceleração
        // normal, disparando o Unstick com o robô se movendo livremente.
        // Critério correto: somente acumular stuck_frames_ quando o controlador
        // está EXPLICITAMENTE comandando velocidade alta (≥ 1.5 m/s) — sinalizando
        // intenção real de movimento — mas o robô não sai do raio de 3 cm.
        // Janela estendida para 60 frames (~1.0 s) para eliminar o efeito
        // "flapping" (disparos repetidos de Unstick em intervalo curto).
        if (std::abs(last_v_) > 1.5) {
            stuck_frames_++;
            // Reset se o robô se afastar > 3 cm da referência (micro-movimentos do APF
            // ficam em 1-2 cm e não resetam mais, exigindo clearance real).
            if (real_dist_moved > 0.030) {
                stuck_frames_ = 0;
                stuck_ref_x_ = robot.x;
                stuck_ref_y_ = robot.y;
            }
        } else {
            // Velocidade baixa: robô ainda acelerando ou parado voluntariamente.
            // Não acumula Unstick — resetar referência para próxima janela.
            stuck_frames_ = 0;
            stuck_ref_x_ = robot.x;
            stuck_ref_y_ = robot.y;
        }

        // 60 frames (~1.0 s) com v ≥ 1.5 m/s e deslocamento < 3 cm → travado de verdade
        if (stuck_frames_ > 60) {
            is_unsticking_ = true;
            unstick_frames_ = 0;
            // [Bug #1B] Frame de transição corrigido: inicia direto com v=-2.0, omega=0
            // para consistência com a FASE 1. O frame anterior gerava v=-2.5, omega=12
            // simultaneamente, o que é conflitante com o recuo linear puro da FASE 1.
            last_v_ = -2.0;
            return clampCommand(RobotCommand::fromVW(robot.id, -2.0, 0.0, 0.075));
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
        // [CORRIGIDO Bug #2B] k_circ era 1.5 — a espiral só atingia offset máximo para
        // theta_err >= 60° (π/3). Para bolas entre 30°–60°, a curvatura era insuficiente
        // e o robô cortava caminho batendo na bola de lado.
        // Com k_circ = 2.5, o offset máximo (90°) é atingido para theta_err >= 36°,
        // garantindo órbita suave mesmo em situações intermediárias.
        double k_circ = 2.5;
        double offset_angle = std::min(max_offset, k_circ * std::abs(theta_err));

        // Angulação final do Campo Vetorial
        double target_angle = angle_to_ball + (theta_err >= 0 ? offset_angle : -offset_angle);
        target_angle = normalizeAngle(target_angle);

        // ── 3. Controle e Desbloqueio Dinâmico (Slew Rate / Push) ─────────
        // [CORRIGIDO Bug #2C] Cone de Ataque: além de theta_err < 0.35 e dist < 0.18,
        // verificamos se o robô está ATRÁS da bola em relação ao gol.
        // Sem essa verificação, o robô podia entrar no cone chegando lateralmente
        // e disparar o Kamikaze com o vetor theta_goal apontando de forma transversal,
        // cortando pela frente da bola em vez de empurrá-la.
        // Critério: o ângulo robô→bola deve ser OPOSTO ao ângulo bola→gol, ou seja,
        // o produto interno entre ambos os vetores deve ser NEGATIVO (robô atrás da bola).
        // cos(angle_to_ball - theta_goal) < 0.1 garante que estamos no hemisfério traseiro.
        bool behind_ball = (std::cos(angle_to_ball - theta_goal) < 0.1);
        bool in_attack_cone = (std::abs(theta_err) < 0.35 && dist < 0.18 && behind_ball);
        
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

            // [CORRIGIDO Bug #2A] Cálculo anterior: v = max(1.2, v_max * cos(angle_diff))
            // Para bola lateral (angle_diff > 61°), cos(angle_diff) < 0.48, e a
            // condição max(1.2, ...) é satisfeita — MAS apenas para cos > 0.48 (i.e., < 61°).
            // Para theta_err entre 61° e 90°, o resultado é v = 1.2 m/s ou até negativo,
            // quebrando a órbita espiral: o robô parava ou recuava em vez de orbitar.
            //
            // Correção: fator de alinhamento suavizado com piso mínimo de 40%.
            // align_factor ∈ [0.40, 1.00] garante que o robô SEMPRE orbita com
            // pelo menos 40% da velocidade máxima, independente do erro angular.
            double angle_diff = normalizeAngle(target_angle - robot.theta);
            double align_factor = 0.40 + 0.60 * std::max(0.0, std::cos(angle_diff));
            v = v_max * align_factor;
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

// ─────────────────────────────────────────────────────────────────────────────
//  OrbitAttackerSkill
//
//  Skill geométrica pura para o atacante que mira o gol (AimGoalTactic).
//  Substitui o encadeamento quebrado AggressiveGoToBall→RotateToAngle→Kick.
//
//  FILOSOFIA: o robô NUNCA vai direto à bola. Ele calcula um ponto
//  'target_behind_ball' que está 12 cm atrás da bola no eixo gol→bola,
//  e só empurra a bola quando está alinhado nessa posição.
//
//  ─── Estados ──────────────────────────────────────────────────────────────
//
//    UNSTICK  → Recuo bruto de 2 fases (ativado pelo detector de travamento)
//               FASE 1 (25 frames): -3.0 m/s, ω=0 (sem rampa — impulso bruto)
//               FASE 2 (20 frames): -1.0 m/s, ω=±12 (giro + ré leve)
//               → retorna ao ORBIT
//
//    ORBIT    → Robô está no lado errado (entre bola e gol) ou longe do
//               target_behind_ball. Orbita lateralmente em arco ao redor da bola.
//               Campo vetorial: ângulo para target_behind_ball + offset lateral
//               proporcional ao quanto o robô precisa contornar.
//               → transita para APPROACH quando angle_err < 15° (histerese)
//
//    APPROACH → Robô vai em linha reta para target_behind_ball.
//               → transita para PUSH quando dist_to_target < PUSH_THRESHOLD
//               → volta para ORBIT se angle_err > 20° (histerese de saída)
//
//    PUSH     → Robô alinhado atrás da bola. Avança em direção à bola,
//               mirando o centro do gol adversário. Velocidade máxima.
//               → volta para ORBIT se desalinhar > 25° (bola se moveu)
//
//  ─── Proteções físicas ────────────────────────────────────────────────────
//
//    · target_behind_ball clampado a 5cm de qualquer parede (geo::WALL_MARGIN)
//    · Tridente (3-probe isFrontBlocked): centro + quina esq. + quina dir.
//      Cobre toda a face frontal do chassi 7.5×7.5cm
//    · Stuck detector: 60 frames com |v_cmd| > 1.5 m/s e desloc. < 3cm
//      → dispara UNSTICK
//    · Histerese de estado: ORBIT→APPROACH exige erro < 15°;
//      saída do APPROACH exige erro > 20°  (gap de 5° evita tremor)
// ─────────────────────────────────────────────────────────────────────────────
class OrbitAttackerSkill final : public Skill {
public:
    OrbitAttackerSkill() = default;

    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
        state_          = State::ORBIT;
        last_v_         = 0.0;
        stuck_frames_   = 0;
        unstick_frames_ = 0;
        stuck_ref_x_    = robot.x;
        stuck_ref_y_    = robot.y;
        log_counter_    = 0;
        (void)ctx;
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        // ── Throttle de log: 5 Hz (a cada 12 frames de 60 Hz) ─────────────
        ++log_counter_;
        bool should_log = (log_counter_ % 12 == 0);

        // ── UNSTICK: bypass de todos os outros estados ─────────────────────
        if (state_ == State::UNSTICK) {
            return doUnstick(robot, ctx, should_log);
        }

        // ── Geometria central ──────────────────────────────────────────────
        auto   goal   = ctx.enemyGoalCenter();
        auto   tbball = geo::calcTargetBehindBall(
                            ctx.ball.x, ctx.ball.y,
                            goal.x, goal.y,
                            geo::BEHIND_BALL_DIST,
                            ctx.field.min_x(), ctx.field.max_x(),
                            ctx.field.min_y(), ctx.field.max_y());

        // Vetor robô → target_behind_ball
        double dx_t  = tbball.x - robot.x;
        double dy_t  = tbball.y - robot.y;
        double dist_t = mag2D(dx_t, dy_t);

        // Ângulo desejado para o target e erro em relação à orientação atual
        double angle_to_target = std::atan2(dy_t, dx_t);
        double angle_err       = normalizeAngle(angle_to_target - robot.theta);

        // Vetor robô → bola (para o push)
        double dx_b  = ctx.ball.x - robot.x;
        double dy_b  = ctx.ball.y - robot.y;
        double dist_b = mag2D(dx_b, dy_b);

        // ── Stuck detector ─────────────────────────────────────────────────
        // Acumula apenas quando o controlador exige velocidade alta (≥1.5 m/s)
        // mas o robô não sai do lugar (< 3cm)
        if (std::abs(last_v_) > 1.5) {
            double moved = mag2D(robot.x - stuck_ref_x_, robot.y - stuck_ref_y_);
            if (moved > 0.030) {
                stuck_frames_  = 0;
                stuck_ref_x_   = robot.x;
                stuck_ref_y_   = robot.y;
            } else {
                stuck_frames_++;
            }
        } else {
            stuck_frames_  = 0;
            stuck_ref_x_   = robot.x;
            stuck_ref_y_   = robot.y;
        }

        if (stuck_frames_ > 60) {
            state_          = State::UNSTICK;
            unstick_frames_ = 0;
            last_v_         = -3.0;
            // Retorno imediato: impulso bruto sem rampa
            return clampCommand(RobotCommand::fromVW(robot.id, -3.0, 0.0, 0.075));
        }

        // ── Máquina de estados ─────────────────────────────────────────────
        RobotCommand cmd;
        switch (state_) {
            case State::ORBIT:    cmd = doOrbit   (robot, ctx, tbball, dist_t, angle_err, should_log); break;
            case State::APPROACH: cmd = doApproach(robot, ctx, tbball, dist_t, angle_err, should_log); break;
            case State::PUSH:     cmd = doPush    (robot, ctx, dist_b,                    should_log); break;
            default:              cmd = RobotCommand::stop(robot.id); break;
        }
        return cmd;
    }

    std::string name() const override { return "OrbitAttackerSkill"; }

private:
    // ── Constantes de estado ──────────────────────────────────────────────
    // Histerese: ORBIT→APPROACH ao atingir < ENTER_APPROACH_ERR (15°)
    //            APPROACH→ORBIT ao superar LEAVE_APPROACH_ERR (20°)
    static constexpr double ENTER_APPROACH_ERR = 0.262;  // 15° em radianos
    static constexpr double LEAVE_APPROACH_ERR = 0.349;  // 20° em radianos
    // APPROACH→PUSH: robô chegou a menos de PUSH_THRESHOLD do target
    static constexpr double PUSH_THRESHOLD     = 0.06;   // 6 cm
    // PUSH→ORBIT: desalinhamento máximo durante o push antes de recalcular
    static constexpr double PUSH_REALIGN_ERR   = 0.436;  // 25°

    enum class State { ORBIT, APPROACH, PUSH, UNSTICK };
    State  state_          = State::ORBIT;
    double last_v_         = 0.0;
    int    stuck_frames_   = 0;
    int    unstick_frames_ = 0;
    double stuck_ref_x_    = 0.0;
    double stuck_ref_y_    = 0.0;
    int    log_counter_    = 0;  // throttle de log a 5 Hz (a cada 12 frames)

    // ── Helper: converte estado em string para log ─────────────────────────
    static const char* stateName(State s) {
        switch (s) {
            case State::ORBIT:    return "ORBIT";
            case State::APPROACH: return "APPROACH";
            case State::PUSH:     return "PUSH";
            case State::UNSTICK:  return "UNSTICK";
            default:              return "?";
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    //  isFrontBlocked — Tridente (3 probes)
    //
    //  Cobre toda a face frontal do chassi 7.5×7.5 cm.
    //    Probe Centro : 8.75 cm à frente no eixo theta
    //    Probe Esq.   : quina −3.75 cm, 6 cm à frente da quina
    //    Probe Dir.   : quina +3.75 cm, 6 cm à frente da quina
    //
    //  Retorna true se QUALQUER probe colidiu com parede ou robô.
    //  Adversários próximos da bola (< 10 cm) são ignorados — são o alvo.
    // ─────────────────────────────────────────────────────────────────────
    bool isFrontBlocked(const RobotState& robot, const GameContext& ctx) const {
        const double cos_t = std::cos(robot.theta);
        const double sin_t = std::sin(robot.theta);
        // Lateral esquerdo: -sin(θ), cos(θ)
        const double lat_x = -sin_t;
        const double lat_y =  cos_t;

        const double half  = geo::ROBOT_RADIUS;        // 3.75 cm
        const double fwd_c = half + 0.05;              // 8.75 cm (centro)
        const double fwd_k = 0.06;                     // 6 cm (quinas)

        struct Probe { double x, y; };
        Probe probes[3] = {
            { robot.x + fwd_c * cos_t,
              robot.y + fwd_c * sin_t },
            { robot.x - half * lat_x + fwd_k * cos_t,
              robot.y - half * lat_y + fwd_k * sin_t },
            { robot.x + half * lat_x + fwd_k * cos_t,
              robot.y + half * lat_y + fwd_k * sin_t },
        };

        const double wm = 0.01;  // margem de parede do probe

        for (const auto& p : probes) {
            if (p.x < ctx.field.min_x() + wm ||
                p.x > ctx.field.max_x() - wm ||
                p.y < ctx.field.min_y() + wm ||
                p.y > ctx.field.max_y() - wm)
                return true;
        }

        // Robôs (aliados e adversários) dentro do cone frontal
        const double block_r    = 0.076 + 0.05;  // diâmetro + 5cm margem
        const double cone_half  = M_PI / 3.0;    // ±60°

        auto checkRobot = [&](double rx, double ry) -> bool {
            double d = mag2D(rx - robot.x, ry - robot.y);
            if (d < block_r) {
                double ang = std::atan2(ry - robot.y, rx - robot.x);
                if (std::abs(normalizeAngle(ang - robot.theta)) < cone_half)
                    return true;
            }
            return false;
        };

        for (const auto& ally : ctx.allies) {
            if (!ally.valid || ally.id == robot.id) continue;
            if (checkRobot(ally.x, ally.y)) return true;
        }
        for (const auto& enemy : ctx.enemies) {
            if (!enemy.valid) continue;
            // Ignora adversários próximos da bola (são o alvo)
            if (mag2D(enemy.x - ctx.ball.x, enemy.y - ctx.ball.y) < 0.10) continue;
            if (checkRobot(enemy.x, enemy.y)) return true;
        }
        return false;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  doOrbit — Contorna a bola lateralmente
    //
    //  O robô está no lado errado (entre bola e gol) ou muito desalinhado.
    //  O campo vetorial direciona para target_behind_ball com um offset
    //  lateral proporcional ao grau de bloqueio.
    //
    //  Histerese de saída: state_ → APPROACH somente se angle_err < 15°
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doOrbit(const RobotState& robot, const GameContext& ctx,
                         const Point2D& tbball,
                         double dist_t, double angle_err,
                         bool should_log)
    {
        // Tridente: se bloqueado, mantém órbita com correção angular apenas
        bool front_blocked = isFrontBlocked(robot, ctx);
        if (front_blocked) {
            double omega_corr = 4.0 * angle_err;
            last_v_ = 0.0;
            if (should_log) {
                fprintf(stderr,
                    "[ORBIT-DBG][ORBIT] FRONT_BLOCKED"
                    " | R=(%.3f,%.3f,%.2frad)"
                    " | Ball=(%.3f,%.3f)"
                    " | TBB=(%.3f,%.3f)"
                    " | dist_ball=? dist_tgt=%.3f ang_err=%.3frad"
                    " | v_pre_avoid=0.000 v_post_avoid=0.000 v_ramp=0.000"
                    " | stuck=%d"
                    " | cmd: L=0.000 R=0.000 omega=%.3f\n",
                    robot.x, robot.y, robot.theta,
                    ctx.ball.x, ctx.ball.y,
                    tbball.x, tbball.y,
                    dist_t, angle_err,
                    stuck_frames_,
                    omega_corr);
            }
            return clampCommand(RobotCommand::fromVW(robot.id, 0.0, omega_corr, 0.075));
        }

        // ── Verificar se robô está entre bola e gol ───────────────────────
        auto goal = ctx.enemyGoalCenter();
        bool between = geo::isRobotBetweenBallAndGoal(
            robot.x, robot.y,
            ctx.ball.x, ctx.ball.y,
            goal.x, goal.y);

        // ── Campo vetorial de órbita ──────────────────────────────────────
        // Vetor robô → bola
        double dx_b = ctx.ball.x - robot.x;
        double dy_b = ctx.ball.y - robot.y;
        double dist_b = mag2D(dx_b, dy_b);

        // Ângulo direto para target_behind_ball
        double angle_to_target = std::atan2(tbball.y - robot.y, tbball.x - robot.x);

        // Offset de órbita: desvia lateralmente para contornar a bola
        // Se entre bola e gol: offset máximo (90°) para girar para fora
        // Caso geral: offset proporcional ao erro de alinhamento
        double orbit_offset = 0.0;
        if (between && dist_b < 0.25) {
            double perp_sign = (robot.y >= ctx.ball.y) ? 1.0 : -1.0;
            orbit_offset = perp_sign * M_PI / 2.0;
        } else {
            double k_orbit = 1.8;
            double abs_err = std::abs(angle_err);
            orbit_offset   = std::copysign(std::min(M_PI / 2.0, k_orbit * abs_err),
                                           angle_err);
        }

        double nav_angle = normalizeAngle(angle_to_target + orbit_offset);

        // ── Controle de velocidade ────────────────────────────────────────
        double angle_diff  = normalizeAngle(nav_angle - robot.theta);
        double align_fac   = 0.35 + 0.65 * std::max(0.0, std::cos(angle_diff));
        double v_pre_avoid = 1.8 * align_fac;      // [DIAG] antes de avoidCollisions

        double v_post_avoid = avoidCollisions(robot, ctx, v_pre_avoid);  // [DIAG]

        double v_ramp = applyLinearRamp(v_post_avoid, last_v_, 4.0);    // [DIAG]
        last_v_  = v_ramp;

        double kp_ang  = 7.0;
        double omega   = applyAngularRamp(angle_diff,
                                          kp_ang * angle_diff,
                                          kp_ang * M_PI);

        // ── Histerese ORBIT→APPROACH ──────────────────────────────────────
        if (!between && std::abs(angle_err) < ENTER_APPROACH_ERR) {
            state_ = State::APPROACH;
        }

        auto cmd = clampCommand(RobotCommand::fromVW(robot.id, v_ramp, omega, 0.075));

        if (should_log) {
            fprintf(stderr,
                "[ORBIT-DBG][ORBIT] between=%d"
                " | R=(%.3f,%.3f,%.2frad)"
                " | Ball=(%.3f,%.3f)"
                " | TBB=(%.3f,%.3f)"
                " | dist_ball=%.3f dist_tgt=%.3f"
                " | ang_err=%.3frad ang_diff=%.3frad"
                " | v_pre=%.3f v_avoid=%.3f v_ramp=%.3f"
                " | stuck=%d"
                " | cmd: L=%.3f R=%.3f omega=%.3f\n",
                (int)between,
                robot.x, robot.y, robot.theta,
                ctx.ball.x, ctx.ball.y,
                tbball.x, tbball.y,
                dist_b, dist_t,
                angle_err, angle_diff,
                v_pre_avoid, v_post_avoid, v_ramp,
                stuck_frames_,
                cmd.wheel_left, cmd.wheel_right, omega);
        }

        return cmd;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  doApproach — Vai em linha reta para target_behind_ball
    //
    //  Histerese de saída para ORBIT: erro > 20°
    //  Transita para PUSH: dist < PUSH_THRESHOLD
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doApproach(const RobotState& robot, const GameContext& ctx,
                            const Point2D& tbball,
                            double dist_t, double angle_err,
                            bool should_log)
    {
        // ── Histerese de saída APPROACH→ORBIT ─────────────────────────────
        if (std::abs(angle_err) > LEAVE_APPROACH_ERR) {
            state_ = State::ORBIT;
            return doOrbit(robot, ctx, tbball, dist_t, angle_err, should_log);
        }

        // ── Tridente ──────────────────────────────────────────────────────
        if (isFrontBlocked(robot, ctx)) {
            last_v_ = 0.0;
            double omega_corr = 5.0 * angle_err;
            if (should_log) {
                fprintf(stderr,
                    "[ORBIT-DBG][APPROACH] FRONT_BLOCKED"
                    " | R=(%.3f,%.3f,%.2frad)"
                    " | Ball=(%.3f,%.3f)"
                    " | TBB=(%.3f,%.3f)"
                    " | dist_tgt=%.3f ang_err=%.3frad"
                    " | v_pre=0.000 v_avoid=0.000 v_ramp=0.000"
                    " | stuck=%d"
                    " | cmd: L=0.000 R=0.000 omega=%.3f\n",
                    robot.x, robot.y, robot.theta,
                    ctx.ball.x, ctx.ball.y,
                    tbball.x, tbball.y,
                    dist_t, angle_err,
                    stuck_frames_,
                    omega_corr);
            }
            return clampCommand(RobotCommand::fromVW(robot.id, 0.0, omega_corr, 0.075));
        }

        // ── Transição para PUSH ───────────────────────────────────────────
        if (dist_t < PUSH_THRESHOLD) {
            state_ = State::PUSH;
            return doPush(robot, ctx, mag2D(ctx.ball.x - robot.x, ctx.ball.y - robot.y), should_log);
        }

        // ── Aproximação direta ────────────────────────────────────────────
        double v_max   = 2.0;
        double angle_diff = normalizeAngle(std::atan2(tbball.y - robot.y,
                                                       tbball.x - robot.x)
                                           - robot.theta);
        double align_f    = std::max(0.3, std::cos(angle_diff));
        double v_pre_avoid = v_max * align_f;           // [DIAG]

        double v_post_avoid = avoidCollisions(robot, ctx, v_pre_avoid);  // [DIAG]
        double v_ramp       = applyLinearRamp(v_post_avoid, last_v_, 5.0); // [DIAG]
        last_v_  = v_ramp;

        double kp_ang  = 7.0;
        double omega   = applyAngularRamp(angle_diff,
                                          kp_ang * angle_diff,
                                          kp_ang * M_PI);

        auto cmd = clampCommand(RobotCommand::fromVW(robot.id, v_ramp, omega, 0.075));

        if (should_log) {
            fprintf(stderr,
                "[ORBIT-DBG][APPROACH]"
                " | R=(%.3f,%.3f,%.2frad)"
                " | Ball=(%.3f,%.3f)"
                " | TBB=(%.3f,%.3f)"
                " | dist_ball=%.3f dist_tgt=%.3f"
                " | ang_err=%.3frad ang_diff=%.3frad"
                " | v_pre=%.3f v_avoid=%.3f v_ramp=%.3f"
                " | stuck=%d"
                " | cmd: L=%.3f R=%.3f omega=%.3f\n",
                robot.x, robot.y, robot.theta,
                ctx.ball.x, ctx.ball.y,
                tbball.x, tbball.y,
                mag2D(ctx.ball.x - robot.x, ctx.ball.y - robot.y), dist_t,
                angle_err, angle_diff,
                v_pre_avoid, v_post_avoid, v_ramp,
                stuck_frames_,
                cmd.wheel_left, cmd.wheel_right, omega);
        }

        return cmd;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  doPush — Empurra a bola em direção ao gol
    //
    //  Robô está alinhado atrás da bola. Avança com velocidade máxima
    //  mirando o CENTRO DO GOL (não a bola) para garantir precisão.
    //
    //  Retorna ao ORBIT se o alinhamento deteriorar > 25°
    //  (bola se moveu, adversário interveio, etc.)
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doPush(const RobotState& robot, const GameContext& ctx,
                        double dist_b, bool should_log)
    {
        auto   goal      = ctx.enemyGoalCenter();
        double angle_goal = std::atan2(goal.y - robot.y, goal.x - robot.x);
        double angle_diff = normalizeAngle(angle_goal - robot.theta);

        // ── Histerese PUSH→ORBIT (desalinhamento excessivo) ────────────────
        if (std::abs(angle_diff) > PUSH_REALIGN_ERR) {
            state_ = State::ORBIT;
            auto tbball = geo::calcTargetBehindBall(
                ctx.ball.x, ctx.ball.y,
                goal.x, goal.y, geo::BEHIND_BALL_DIST,
                ctx.field.min_x(), ctx.field.max_x(),
                ctx.field.min_y(), ctx.field.max_y());
            double dx_t = tbball.x - robot.x;
            double dy_t = tbball.y - robot.y;
            double err  = normalizeAngle(std::atan2(dy_t, dx_t) - robot.theta);
            return doOrbit(robot, ctx, tbball, mag2D(dx_t, dy_t), err, should_log);
        }

        // ── Avanço máximo em direção ao gol ──────────────────────────────
        double v_pre  = 2.5;                               // [DIAG] antes da rampa
        double v_ramp = applyLinearRamp(v_pre, last_v_, 6.0); // [DIAG]
        last_v_ = v_ramp;

        double kp_ang = 4.0;
        double omega  = std::clamp(kp_ang * angle_diff,
                                   -kp_ang * M_PI, kp_ang * M_PI);

        auto cmd = clampCommand(RobotCommand::fromVW(robot.id, v_ramp, omega, 0.075, true));

        if (should_log) {
            fprintf(stderr,
                "[ORBIT-DBG][PUSH]"
                " | R=(%.3f,%.3f,%.2frad)"
                " | Ball=(%.3f,%.3f)"
                " | dist_ball=%.3f"
                " | ang_diff=%.3frad (goal)"
                " | v_pre=%.3f v_ramp=%.3f (no avoidCollisions in PUSH)"
                " | stuck=%d"
                " | cmd: L=%.3f R=%.3f omega=%.3f fast_slew=1\n",
                robot.x, robot.y, robot.theta,
                ctx.ball.x, ctx.ball.y,
                dist_b,
                angle_diff,
                v_pre, v_ramp,
                stuck_frames_,
                cmd.wheel_left, cmd.wheel_right, omega);
        }

        return cmd;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  doUnstick — Recuo bruto de 2 fases
    //
    //  FASE 1 (frames 1–25): -3.0 m/s, ω=0
    //    Impulso bruto SEM rampa para romper encaixe de quina do cubo 7.5cm.
    //    fromVW() bypass qualquer ramp externa — vai direto ao simulador.
    //
    //  FASE 2 (frames 26–45): -1.0 m/s, ω=±12
    //    Clearance já criado. Gira para sair do obstáculo.
    //    Sentido de giro: afasta do lado com menos espaço (y > 0 → gira +).
    //
    //  Após 45 frames → volta ao ORBIT com resetização do stuck_detector.
    // ─────────────────────────────────────────────────────────────────────
    RobotCommand doUnstick(const RobotState& robot, const GameContext& ctx, bool should_log) {
        unstick_frames_++;

        if (unstick_frames_ > 45) {
            // Sequência completa: reseta e volta ao ORBIT
            state_          = State::ORBIT;
            stuck_frames_   = 0;
            unstick_frames_ = 0;
            stuck_ref_x_    = robot.x;
            stuck_ref_y_    = robot.y;
            last_v_         = 0.0;
            auto goal  = ctx.enemyGoalCenter();
            auto tbball = geo::calcTargetBehindBall(
                ctx.ball.x, ctx.ball.y,
                goal.x, goal.y, geo::BEHIND_BALL_DIST,
                ctx.field.min_x(), ctx.field.max_x(),
                ctx.field.min_y(), ctx.field.max_y());
            double dx = tbball.x - robot.x;
            double dy = tbball.y - robot.y;
            double err = normalizeAngle(std::atan2(dy, dx) - robot.theta);
            return doOrbit(robot, ctx, tbball, mag2D(dx, dy), err, should_log);
        }

        RobotCommand cmd;
        double v_cmd, omega_cmd;
        if (unstick_frames_ <= 25) {
            // FASE 1: impulso bruto de ré — SEM omega, SEM rampa
            last_v_   = -3.0;
            v_cmd     = -3.0;
            omega_cmd = 0.0;
            cmd = clampCommand(RobotCommand::fromVW(robot.id, v_cmd, omega_cmd, 0.075));
        } else {
            // FASE 2: ré leve + giro de escape
            double spin = (robot.y >= 0.0) ? 12.0 : -12.0;
            last_v_   = -1.0;
            v_cmd     = -1.0;
            omega_cmd = spin;
            cmd = clampCommand(RobotCommand::fromVW(robot.id, v_cmd, omega_cmd, 0.075));
        }

        if (should_log) {
            fprintf(stderr,
                "[ORBIT-DBG][UNSTICK] fase=%s unstick_frame=%d"
                " | R=(%.3f,%.3f,%.2frad)"
                " | Ball=(%.3f,%.3f)"
                " | v_cmd=%.3f omega_cmd=%.3f"
                " | cmd: L=%.3f R=%.3f\n",
                (unstick_frames_ <= 25 ? "1-RECUO" : "2-GIRO"),
                unstick_frames_,
                robot.x, robot.y, robot.theta,
                ctx.ball.x, ctx.ball.y,
                v_cmd, omega_cmd,
                cmd.wheel_left, cmd.wheel_right);
        }

        return cmd;
    }
};

} // namespace vss

