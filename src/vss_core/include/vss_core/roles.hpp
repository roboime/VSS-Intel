#pragma once

#include "role.hpp"
#include "tactic.hpp"
#include "go_to_ball_skill.hpp"
#include "go_to_position_skill.hpp"
#include "vector_attacker_skill.hpp"
#include "kick_skill.hpp"
#include "wait_ball_kicker_skill.hpp"
#include "rotate_to_angle_skill.hpp"
#include <memory>
#include <cmath>

namespace vss {

// ═════════════════════════════════════════════════════════════════════════════
//  Táticas internas (usadas pelos Roles abaixo)
//  Equivalem aos Tacticbooks do LabVIEW
// ═════════════════════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────────────────────
//  DefendGoalTactic  —  usada pelo KeeperRoleVSS
//  Lógica: goleiro se posiciona na linha do gol, movendo-se lateralmente
//  para interceptar a trajetória da bola. Não sai da área do goleiro.
//
//  [FIX REGRESSÃO] A versão anterior instanciava GoToPositionSkill a CADA
//  frame dentro de execute(), zerado last_v_ a cada ciclo. Isso fazia a
//  rampa trapezoidal recomecer do zero em todo tick — o goleiro nunca
//  superava os primeiros frames de aceleração, resultando no movimento em
//  câmera lenta ("samba").
//  Correção: skill_ é membro persistente da tática. Apenas o alvo é
//  atualizado a cada frame via setTarget(), preservando last_v_ entre ciclos.
//  Parâmetros do goleiro: kp_linear alto + sem stop_on_arrival (loop contínuo).
// ─────────────────────────────────────────────────────────────────────────────
class DefendGoalTactic final : public Tactic {
public:
    DefendGoalTactic() {
        // Parâmetros específicos do goleiro:
        //   kp_linear alto (5.0) → reação rápida no eixo Y.
        //   kp_angular alto (8.0) → alinhamento ágil (sem "samba" de orientação).
        //   arrival_threshold pequeno (0.02) → não para longe do alvo.
        //   stop_on_arrival = false → loop contínuo: nunca para de monitorar.
        GoToPositionParams p;
        p.kp_linear         = 5.0;
        p.kp_angular        = 8.0;
        p.arrival_threshold = 0.02;
        p.max_linear_speed  = 2.2;
        p.stop_on_arrival   = false;
        skill_ = std::make_unique<GoToPositionSkill>(0.0, 0.0, 1e9, p);
    }

    std::string name() const override { return "DefendGoalTactic"; }

    bool isFinished() const override { return false; }  // nunca termina

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        // Posição base do goleiro: na linha do gol, lado de defesa
        // Margem de recuo aumentada para 0.08m para evitar impactos com as traves/fundo
        double goal_x = -ctx.attack_dir * (ctx.field.max_x() - 0.08);

        // Projeta a bola na linha do goleiro para calcular target_y
        // Se a bola vem em direção ao nosso gol, intercepta na linha
        double target_y = ctx.ball.y;

        // Predição simples: onde a bola cruzará a linha do goleiro?
        if (std::abs(ctx.ball.vx) > 0.05) {
            double time_to_line = (goal_x - ctx.ball.x) / ctx.ball.vx;
            if (time_to_line > 0 && time_to_line < 1.5) {
                target_y = ctx.ball.y + ctx.ball.vy * time_to_line;
            }
        }

        // Clamp: goleiro não sai estritamente da largura do gol (desconta 4cm do raio do robô)
        double max_y = ctx.field.goal_width / 2.0 - 0.04;
        target_y = std::clamp(target_y, -max_y, max_y);

        // Atualiza apenas o alvo — last_v_ da skill é preservado entre frames
        skill_->setTarget(goal_x, target_y);
        return skill_->execute(robot, ctx);
    }

private:
    // [FIX] Skill persistente: last_v_ não é mais zerada a cada tick.
    std::unique_ptr<GoToPositionSkill> skill_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  DefendAreaTactic  —  usada pelo DefenderAreaRoleVSS
//  Lógica: defensor se posiciona entre a bola e o nosso gol.
//
//  [FIX REGRESSÃO] Mesmo padrão da DefendGoalTactic: skill persistente
//  para preservar last_v_ entre frames e não recomecar a rampa a cada tick.
// ─────────────────────────────────────────────────────────────────────────────
class DefendAreaTactic final : public Tactic {
public:
    explicit DefendAreaTactic(int num_defenders = 0)
        : num_defenders_(num_defenders) {
        GoToPositionParams p;
        p.kp_linear         = 4.0;
        p.kp_angular        = 6.0;
        p.arrival_threshold = 0.04;
        p.max_linear_speed  = 2.2;
        p.stop_on_arrival   = false;
        skill_ = std::make_unique<GoToPositionSkill>(0.0, 0.0, 1e9, p);
    }

    std::string name() const override { return "DefendAreaTactic"; }
    bool isFinished() const override { return false; }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        auto our_goal = ctx.ourGoalCenter();

        double dx_ball_goal = our_goal.x - ctx.ball.x;
        double dy_ball_goal = our_goal.y - ctx.ball.y;
        double dist_ball_goal = std::hypot(dx_ball_goal, dy_ball_goal);

        // Posição de defesa: entre a bola e o gol, a DEFEND_DIST metros do gol
        double defend_dist = 0.20 + num_defenders_ * 0.05;
        defend_dist = std::min(defend_dist, dist_ball_goal * 0.6);

        double target_x = our_goal.x + (dx_ball_goal / dist_ball_goal) * (-defend_dist);
        double target_y = our_goal.y + (dy_ball_goal / dist_ball_goal) * (-defend_dist);

        // Clamp dentro do campo
        // Margem de segurança aumentada para 0.08m para evitar colisões com as paredes
        target_x = std::clamp(target_x,
                               ctx.field.min_x() + 0.08,
                               ctx.field.max_x() - 0.08);
        target_y = std::clamp(target_y,
                               ctx.field.min_y() + 0.08,
                               ctx.field.max_y() - 0.08);

        skill_->setTarget(target_x, target_y);
        return skill_->execute(robot, ctx);
    }

private:
    int num_defenders_;
    std::unique_ptr<GoToPositionSkill> skill_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  AggressiveGoToBallSkill
//
//  Igual ao GoToBallSkill mas SEM a proteção da área adversária.
//  O atacante designado DEVE poder entrar na área para empurrar a bola.
// ─────────────────────────────────────────────────────────────────────────────
class AggressiveGoToBallSkill final : public Skill {
public:
    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        // Destino direto: bola (sem offset de área adversária)
        double target_x = ctx.ball.x;
        double target_y = ctx.ball.y;

        // Clamp mínimo dentro do campo (apenas paredes)
        const double m = 0.04;
        target_x = std::clamp(target_x, ctx.field.min_x()+m, ctx.field.max_x()-m);
        target_y = std::clamp(target_y, ctx.field.min_y()+m, ctx.field.max_y()-m);

        double dx   = target_x - robot.x;
        double dy   = target_y - robot.y;
        double dist = std::hypot(dx, dy);

        // ── Singularidade Angular (atan2) ─────────────────────────────────
        double angle_err = 0.0;
        if (dist >= 0.02) {
            angle_err = normalizeAngle(std::atan2(dy, dx) - robot.theta);
        }

        // ── Zona Morta de Ângulo ──────────────────────────────────────────
        if (std::abs(angle_err) < 0.05) {
            angle_err = 0.0;
        }

        double align     = std::max(0.0, std::cos(angle_err));

        // Ganho maior para o atacante ser agressivo (aumentado para 7.0)
        double v = std::clamp(7.0 * dist * align, -2.2, 2.2);

        // Garantir velocidade linear mínima na aproximação final para não desacelerar
        if (dist < 0.15 && align > 0.5) {
            v = std::max(v, 1.2 * align);
        }

        double omega = 6.0 * angle_err;

        // Desvio de colisões com aliados (não adversários — atacante empurra adversário)
        for (const auto& ally : ctx.allies) {
            if (ally.id == robot.id || !ally.valid) continue;
            double adx = ally.x - robot.x;
            double ady = ally.y - robot.y;
            double adist = std::hypot(adx, ady);
            if (adist < 0.12) {
                double ang_to_ally = std::atan2(ady, adx);
                double ang_diff = std::abs(normalizeAngle(ang_to_ally - robot.theta));
                if (ang_diff < M_PI / 4.0) {
                    double scale = (adist - 0.075) / (0.12 - 0.075);
                    v *= std::clamp(scale, 0.0, 1.0);
                }
            }
        }

        // ── Zona Morta de Distância ───────────────────────────────────────
        if (dist < 0.02) {
            v = 0.0;
            omega = 0.0;
        }

        // Fim da Skill: se chegou a menos de 4cm da bola, finaliza mas retorna o comando calculado
        if (dist < 0.04) {
            finished_ = true;
            return clampCommand(RobotCommand::fromVW(robot.id, v, omega, 0.075));
        }

        return clampCommand(RobotCommand::fromVW(robot.id, v, omega, 0.075));
    }

    std::string name() const override { return "AggressiveGoToBallSkill"; }
};

// ─────────────────────────────────────────────────────────────────────────────
//  VectorAttackTactic  —  NOVA TÁTICA BASEADA EM VETORES
//  Encapsula a nova VectorAttackerSkill que unifica aproximação e chute fluidamente.
// ─────────────────────────────────────────────────────────────────────────────
class VectorAttackTactic final : public Tactic {
public:
    std::string name() const override { return "VectorAttackTactic"; }
    bool isFinished() const override { return false; }  // Contínuo

    VectorAttackTactic() {
        addSkill<VectorAttackerSkill>();
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        return Tactic::execute(robot, ctx);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  SupportPositionTactic  —  zagueiro se posiciona a meio-campo em apoio
//
//  [FIX REGRESSÃO] Skill persistente para evitar reset de last_v_ a cada
//  frame. Além disso, corrigido o branch "ball_relative_x >= 0" para
//  incluir a bola no meio-campo na fase ofensiva.
// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
//  SupportPositionTactic  —  zagueiro ofensivo se posiciona em apoio
//
//  LÓGICA NOVA (reestruturada):
//
//  Eixo X:  target_x = ball.x − attack_dir × 0.40
//    → Suporte fica SEMPRE 40 cm atrás da linha da bola no eixo de ataque.
//    → Clamp mínimo: nunca fica atrás da posição −0.45 m (perto do nosso gol)
//    → Clamp máximo: nunca ultrapassa 20 cm além da bola no campo ofensivo
//      (evita chegar perto do atacante)
//
//  Eixo Y:  target_y = ball.y × 0.5
//    → Espelha a bola com ganho 0.5: se bola em Y=0.30, suporte fica Y=0.15
//    → Posiciona para rebote sem cruzar o caminho do atacante
//    → Clamp: mínimo 20 cm das paredes laterais
//
//  [FIX REINSTANCIAÇÃO] skill_ é membro persistente — last_v_ nunca é
//  zerado entre frames. Antes, SupportAttackerRoleVSS::selectTactic()
//  chamava make_shared<SupportPositionTactic>() a CADA frame (60x/s),
//  destruindo a skill e reiniciando a rampa do zero.
// ─────────────────────────────────────────────────────────────────────────────
class SupportPositionTactic final : public Tactic {
public:
    SupportPositionTactic() {
        GoToPositionParams p;
        p.kp_linear         = 4.0;
        p.kp_angular        = 6.0;
        p.arrival_threshold = 0.04;  // menor: o suporte deve rastrear a bola
        p.max_linear_speed  = 2.0;
        p.stop_on_arrival   = false; // loop contínuo: sempre segue a bola
        skill_ = std::make_unique<GoToPositionSkill>(0.0, 0.0, 1e9, p);
    }

    std::string name() const override { return "SupportPositionTactic"; }
    bool isFinished() const override { return false; }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        // ── Eixo X: 40 cm atrás da bola no eixo de ataque ────────────────
        //   attack_dir > 0: gol adversário em +X, "atrás" = menos X
        //   attack_dir < 0: gol adversário em -X, "atrás" = mais X
        double target_x = ctx.ball.x - ctx.attack_dir * 0.40;

        // Clamp X — O suporte NÃO pode sair do campo nem entrar na
        // nossa área de gol. Limite mínimo: −0.45 m da origem
        // (campo vai até ±0.75 m; área do goleiro começa em ±0.60 m)
        const double x_min_safe = -ctx.attack_dir * 0.45;  // nosso lado
        const double x_max_safe =  ctx.attack_dir * (ctx.field.max_x() - 0.10);

        if (ctx.attack_dir > 0) {
            // Atacamos para +X: x_min_safe é negativo (nosso lado)
            target_x = std::clamp(target_x, x_min_safe, x_max_safe);
        } else {
            // Atacamos para -X: x_min_safe é positivo
            target_x = std::clamp(target_x, x_max_safe, x_min_safe);
        }

        // ── Eixo Y: espelha a bola com ganho 0.5 ─────────────────────────
        //   Bola em Y=0.30 → suporte fica em Y=0.15 (lado do rebote)
        //   Bola em Y=0.00 → suporte fica em Y=0.00 (centro)
        double target_y = ctx.ball.y * 0.5;

        // Clamp Y: mínimo 20 cm das paredes laterais
        target_y = std::clamp(target_y,
                               ctx.field.min_y() + 0.20,
                               ctx.field.max_y() - 0.20);

        // Skill persistente: last_v_ preservado entre frames
        skill_->setTarget(target_x, target_y);
        return skill_->execute(robot, ctx);
    }

private:
    // Skill persistente: last_v_ NÃO é zerada entre frames
    std::unique_ptr<GoToPositionSkill> skill_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  GoToBallAndKickTactic  —  usada pelos Roles atacantes
//  Sequência: GoToBallSkill → KickSkill
//  Loop: quando KickSkill termina, recomeça do GoToBall
// ─────────────────────────────────────────────────────────────────────────────
class GoToBallAndKickTactic final : public Tactic {
public:
    std::string name() const override { return "GoToBallAndKickTactic"; }
    bool isFinished() const override { return false; }  // loop infinito

    GoToBallAndKickTactic() {
        addSkill<GoToBallSkill>();
        addSkill<KickSkill>();
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        // Usa execute da Tactic base, mas reinicia quando KickSkill termina
        auto cmd = Tactic::execute(robot, ctx);
        if (tactic_finished_) {
            // Reinicia o ciclo GoToBall → Kick
            tactic_finished_   = false;
            current_skill_idx_ = 0;
            skills_[0]->init(robot, ctx);
        }
        return cmd;
    }
};


// ═════════════════════════════════════════════════════════════════════════════
//  ROLES VSS  (extraídos da imagem 6 — hierarquia real do projeto)
// ═════════════════════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────────────────────
//  KeeperRoleVSS
//
//  [FIX C3] tactic_ é membro persistente: a DefendGoalTactic (e sua
//  GoToPositionSkill interna) sobrevivem entre frames, preservando last_v_.
//  Antes: make_shared<DefendGoalTactic>() a cada selectTactic() → last_v_
//  zerado 60x/s → goleiro nunca acelerava ("câmera lenta").
// ─────────────────────────────────────────────────────────────────────────────
class KeeperRoleVSS final : public Role {
public:
    explicit KeeperRoleVSS(uint8_t id)
        : Role(id)
        , tactic_(std::make_shared<DefendGoalTactic>())
    {}
    std::string name() const override { return "KeeperRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return tactic_;  // sempre o mesmo objeto — last_v_ preservado entre frames
    }

private:
    std::shared_ptr<DefendGoalTactic> tactic_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  DefenderAreaRoleVSS
//
//  [FIX C3] Mesmo padrão do KeeperRoleVSS: tactic_ persistente.
//  num_defenders_ é fixo por Role (definido pela Play na criação) — a tática
//  é construída uma única vez com esse valor e reutilizada em todos os frames.
// ─────────────────────────────────────────────────────────────────────────────
class DefenderAreaRoleVSS final : public Role {
public:
    explicit DefenderAreaRoleVSS(uint8_t id, int num_defenders = 0)
        : Role(id)
        , num_defenders_(num_defenders)
        , tactic_(std::make_shared<DefendAreaTactic>(num_defenders))
    {}

    std::string name() const override { return "DefenderAreaRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return tactic_;  // sempre o mesmo objeto — last_v_ preservado entre frames
    }

private:
    int num_defenders_;
    std::shared_ptr<DefendAreaTactic> tactic_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  NormalAttackerRoleVSS
//
//  [FIX C4] tactic_ persistente: preserva last_v_, stuck_frames_,
//  is_unsticking_ e unstick_frames_ da VectorAttackerSkill interna.
//  Antes: make_shared<VectorAttackTactic>() a cada selectTactic() zeravam
//  todos esses contadores 60x/s — o Unstick Detector nunca disparava e
//  o atacante nunca atingia velocidade de cruzeiro.
// ─────────────────────────────────────────────────────────────────────────────
class NormalAttackerRoleVSS final : public Role {
public:
    explicit NormalAttackerRoleVSS(uint8_t id)
        : Role(id)
        , tactic_(std::make_shared<VectorAttackTactic>())
    {}
    std::string name() const override { return "NormalAttackerRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return tactic_;  // sempre o mesmo objeto — estado da VectorAttackerSkill preservado
    }

private:
    std::shared_ptr<VectorAttackTactic> tactic_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  SupportAttackerRoleVSS
//
//  NOVO: Robô de suporte/zagueiro ofensivo.
//  Em vez de perseguir a bola (causando colisões com o atacante principal),
//  este robô se posiciona estrategicamente para receber passes e cobrir
//  o espaço. Evita o "fogo amigo" onde dois robôs vão na mesma bola.
// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
//  SupportAttackerRoleVSS
//
//  [FIX CRÍTICO] Bug de reinstanciação corrigido.
//
//  O padrão anterior: selectTactic() retornava make_shared<SupportPositionTactic>()
//  a cada chamada. Como Role::execute() chama selectTactic() a cada frame (60Hz),
//  a SupportPositionTactic (e sua skill interna GoToPositionSkill) eram destruídas
//  e recriadas 60 vezes por segundo. O last_v_ da skill era zerado a cada ciclo,
//  reiniciando a rampa trapezoidal do zero — o robô nunca acelerava.
//
//  Correção: tactic_ é membro persistente, instanciada apenas uma vez no
//  construtor. selectTactic() retorna sempre o mesmo objeto compartilhado.
// ─────────────────────────────────────────────────────────────────────────────
class SupportAttackerRoleVSS final : public Role {
public:
    explicit SupportAttackerRoleVSS(uint8_t id)
        : Role(id)
        , tactic_(std::make_shared<SupportPositionTactic>())
    {}
    std::string name() const override { return "SupportAttackerRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        // Retorna SEMPRE o mesmo objeto — tactic_ e sua skill_ internas
        // nunca são destruídas, preservando last_v_ entre todos os frames.
        return tactic_;
    }

private:
    // Tática persistente: criada uma vez, reutilizada durante toda a vida do Role
    std::shared_ptr<SupportPositionTactic> tactic_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  AttackerAimGoalRoleVSS
//
//  Atacante que mira o gol antes de chutar.
//  Diferença do NormalAttacker: adiciona fase de alinhamento explícita.
// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
//  AimGoalTactic  —  Tática de ataque ao gol com posicionamento orbital
//
//  REESCRITA COMPLETA. O encadeamento antigo
//    AggressiveGoToBallSkill → RotateToAngleSkill → KickSkill
//  causava visão de túnel: o robô ia direto à bola, ficava preso em cima
//  dela e nunca chegava ao KickSkill.
//
//  NOVA LÓGICA: Uma única OrbitAttackerSkill persistente (skill_ é membro),
//  com máquina de estados interna:
//    ORBIT → APPROACH → PUSH
//  O robô NUNCA vai direto à bola. Sempre navega para target_behind_ball
//  (12 cm atrás da bola no eixo gol→bola) antes de empurrar.
//
//  [FIX REINSTANCIAÇÃO] skill_ é membro desta tática — não é recriada a
//  cada frame. A tática em si também é armazenada de forma persistente em
//  AttackerAimGoalRoleVSS (ver abaixo).
// ─────────────────────────────────────────────────────────────────────────────
class AimGoalTactic final : public Tactic {
public:
    std::string name() const override { return "AimGoalTactic"; }
    bool isFinished() const override { return false; }  // loop contínuo

    AimGoalTactic() {
        // OrbitAttackerSkill encapsula toda a lógica ORBIT→APPROACH→PUSH
        // incluindo Tridente, Unstick e histerese de estado
        skill_ = std::make_unique<OrbitAttackerSkill>();
    }

    void init(const RobotState& robot, const GameContext& ctx) override {
        Tactic::init(robot, ctx);
        skill_->init(robot, ctx);
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        // Delega diretamente: a OrbitAttackerSkill é a única skill e é persistente
        return skill_->execute(robot, ctx);
    }

private:
    std::unique_ptr<OrbitAttackerSkill> skill_;
};

// [FIX REINSTANCIAÇÃO] Mesmo padrão aplicado ao AttackerAimGoalRoleVSS:
// tactic_ persistente para preservar o estado ORBIT/APPROACH/PUSH da
// OrbitAttackerSkill entre frames.
class AttackerAimGoalRoleVSS final : public Role {
public:
    explicit AttackerAimGoalRoleVSS(uint8_t id)
        : Role(id)
        , tactic_(std::make_shared<AimGoalTactic>())
    {}
    std::string name() const override { return "AttackerAimGoalRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return tactic_;  // Sempre o mesmo objeto — estado da máquina preservado
    }

private:
    std::shared_ptr<AimGoalTactic> tactic_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  BallChallengerAttackerRoleVSS
//
//  Atacante agressivo: sempre vai para a bola independente de distância.
//  Usado quando o time precisa pressionar.
//
//  [FIX C3] tactic_ persistente: mesmo problema e mesma solução do
//  NormalAttackerRoleVSS. VectorAttackerSkill precisa sobreviver entre
//  frames para que stuck_frames_ acumule e o Unstick dispare.
// ─────────────────────────────────────────────────────────────────────────────
class BallChallengerAttackerRoleVSS final : public Role {
public:
    explicit BallChallengerAttackerRoleVSS(uint8_t id)
        : Role(id)
        , tactic_(std::make_shared<VectorAttackTactic>())
    {}
    std::string name() const override { return "BallChallengerAttackerRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return tactic_;  // sempre o mesmo objeto — estado da VectorAttackerSkill preservado
    }

private:
    std::shared_ptr<VectorAttackTactic> tactic_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  GoAndKickRoleVSS
//
//  Vai até a bola e chuta uma vez (não loop).
//  Usado em set-pieces: kickoff, freeball, direct kick.
// ─────────────────────────────────────────────────────────────────────────────
class GoAndKickOneTimeTactic final : public Tactic {
public:
    std::string name() const override { return "GoAndKickOneTimeTactic"; }

    GoAndKickOneTimeTactic() {
        addSkill<AggressiveGoToBallSkill>();
        addSkill<KickSkill>();
    }
    // isFinished() da Tactic base retorna true após KickSkill terminar
};

class GoAndKickRoleVSS final : public Role {
public:
    explicit GoAndKickRoleVSS(uint8_t id) : Role(id) {}
    std::string name() const override { return "GoAndKickRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return std::make_shared<GoAndKickOneTimeTactic>();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  GoalKickKeeperRoleVSS
//
//  Goleiro executa o tiro de meta.
//  Sai da posição de goleiro, vai até a bola e chuta uma vez.
// ─────────────────────────────────────────────────────────────────────────────
class GoalKickKeeperRoleVSS final : public Role {
public:
    explicit GoalKickKeeperRoleVSS(uint8_t id) : Role(id) {}
    std::string name() const override { return "GoalKickKeeperRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        double dist = distToBall(robot, ctx);

        // Se a bola está perto, executa o chute de meta
        if (dist < 0.35) {
            return std::make_shared<GoAndKickOneTimeTactic>();
        }

        // Senão, vai buscar a bola
        return std::make_shared<SingleSkillTactic>(
            std::make_shared<GoToBallSkill>());
    }
};

} // namespace vss
