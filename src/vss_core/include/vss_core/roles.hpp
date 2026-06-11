#pragma once

#include "role.hpp"
#include "tactic.hpp"
#include "go_to_ball_skill.hpp"
#include "go_to_position_skill.hpp"
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
// ─────────────────────────────────────────────────────────────────────────────
class DefendGoalTactic final : public Tactic {
public:
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

        // Clamp: goleiro não sai da largura da área do gol
        double max_y = ctx.field.goal_width / 2.0 + 0.05;
        target_y = std::clamp(target_y, -max_y, max_y);

        // Vai para posição de defesa
        GoToPositionSkill gtp(goal_x, target_y);
        return gtp.execute(robot, ctx);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  DefendAreaTactic  —  usada pelo DefenderAreaRoleVSS
//  Lógica: defensor se posiciona entre a bola e o nosso gol.
// ─────────────────────────────────────────────────────────────────────────────
class DefendAreaTactic final : public Tactic {
public:
    explicit DefendAreaTactic(int num_defenders = 0)
        : num_defenders_(num_defenders) {}

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

        GoToPositionSkill gtp(target_x, target_y);
        return gtp.execute(robot, ctx);
    }

private:
    int num_defenders_;
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

        if (dist < 0.06) {
            finished_ = true;
            return RobotCommand::stop(robot.id);
        }

        double angle_err = normalizeAngle(std::atan2(dy, dx) - robot.theta);
        double align     = std::max(0.0, std::cos(angle_err));

        // Ganho maior para o atacante ser agressivo
        double v = std::clamp(4.0 * dist * align, -2.2, 2.2);

        // Zona morta angular
        double omega = (std::abs(angle_err) < 0.05) ? 0.0 : 6.0 * angle_err;

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

        return clampCommand(RobotCommand::fromVW(robot.id, v, omega, 0.075));
    }

    std::string name() const override { return "AggressiveGoToBallSkill"; }
};

// ─────────────────────────────────────────────────────────────────────────────
//  AggressivePushTactic  —  vai direto na bola sem hesitar, para agressão
//  Sequência: AggressiveGoToBallSkill → KickSkill (loop)
// ─────────────────────────────────────────────────────────────────────────────
class AggressivePushTactic final : public Tactic {
public:
    std::string name() const override { return "AggressivePushTactic"; }
    bool isFinished() const override { return false; }  // loop infinito

    AggressivePushTactic() {
        addSkill<AggressiveGoToBallSkill>();
        addSkill<KickSkill>();
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        auto cmd = Tactic::execute(robot, ctx);
        if (tactic_finished_) {
            tactic_finished_   = false;
            current_skill_idx_ = 0;
            skills_[0]->init(robot, ctx);
        }
        return cmd;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  SupportPositionTactic  —  zagueiro se posiciona a meio-campo em apoio
//
//  Em vez de perseguir a bola (causando fogo amigo), o zagueiro/suporte
//  se posiciona estrategicamente entre a bola e o nosso gol,
//  pronto para defender mas também para receber a bola.
// ─────────────────────────────────────────────────────────────────────────────
class SupportPositionTactic final : public Tactic {
public:
    std::string name() const override { return "SupportPositionTactic"; }
    bool isFinished() const override { return false; }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        // Posição de suporte: atrás da bola (em relação ao nosso gol),
        // a uma distância lateral de 0.20m para não bloquear o atacante
        double our_goal_x = -ctx.attack_dir * ctx.field.max_x();

        // Fica entre a bola e o gol próprio, a ~35% da distância do gol
        double support_x = ctx.ball.x * 0.35 + our_goal_x * 0.65;

        // Offset lateral para não cobrir o mesmo corredor do atacante
        double lateral_offset = (ctx.ball.y > 0.0) ? -0.18 : 0.18;
        double support_y = std::clamp(ctx.ball.y + lateral_offset,
                                      ctx.field.min_y() + 0.10,
                                      ctx.field.max_y() - 0.10);

        // Clamp no eixo X: o suporte não vai para o campo adversário
        support_x = std::clamp(support_x,
                                ctx.field.min_x() + 0.08,
                                0.0);  // não passa do meio campo

        GoToPositionSkill gtp(support_x, support_y);
        return gtp.execute(robot, ctx);
    }
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
// ─────────────────────────────────────────────────────────────────────────────
class KeeperRoleVSS final : public Role {
public:
    explicit KeeperRoleVSS(uint8_t id) : Role(id) {}
    std::string name() const override { return "KeeperRoleVSS"; }

protected:
    // LabVIEW: sempre DefendGoalTactic, sem condições de troca
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return std::make_shared<DefendGoalTactic>();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  DefenderAreaRoleVSS
// ─────────────────────────────────────────────────────────────────────────────
class DefenderAreaRoleVSS final : public Role {
public:
    explicit DefenderAreaRoleVSS(uint8_t id, int num_defenders = 0)
        : Role(id), num_defenders_(num_defenders) {}

    std::string name() const override { return "DefenderAreaRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return std::make_shared<DefendAreaTactic>(num_defenders_);
    }

private:
    int num_defenders_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  NormalAttackerRoleVSS
//
//  CORRIGIDO: Atacante agressivo que vai direto na bola e entra na área.
//  Usa AggressivePushTactic para não ser bloqueado pela proteção de área.
//  A atribuição de papel (atacante) é feita pelo Play baseado em distância.
// ─────────────────────────────────────────────────────────────────────────────
class NormalAttackerRoleVSS final : public Role {
public:
    explicit NormalAttackerRoleVSS(uint8_t id) : Role(id) {}
    std::string name() const override { return "NormalAttackerRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        // Atacante sempre usa a tática agressiva: vai direto na bola, entra na área
        return std::make_shared<AggressivePushTactic>();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  SupportAttackerRoleVSS
//
//  NOVO: Robô de suporte/zagueiro ofensivo.
//  Em vez de perseguir a bola (causando colisões com o atacante principal),
//  este robô se posiciona estrategicamente para receber passes e cobrir
//  o espaço. Evita o "fogo amigo" onde dois robôs vão na mesma bola.
// ─────────────────────────────────────────────────────────────────────────────
class SupportAttackerRoleVSS final : public Role {
public:
    explicit SupportAttackerRoleVSS(uint8_t id) : Role(id) {}
    std::string name() const override { return "SupportAttackerRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return std::make_shared<SupportPositionTactic>();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  AttackerAimGoalRoleVSS
//
//  Atacante que mira o gol antes de chutar.
//  Diferença do NormalAttacker: adiciona fase de alinhamento explícita.
// ─────────────────────────────────────────────────────────────────────────────
class AimGoalTactic final : public Tactic {
public:
    std::string name() const override { return "AimGoalTactic"; }
    bool isFinished() const override { return false; }

    AimGoalTactic() {
        addSkill<AggressiveGoToBallSkill>();
        addSkill<RotateToAngleSkill>(RotateToAngleSkill::towardGoal());
        addSkill<KickSkill>();
    }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        auto cmd = Tactic::execute(robot, ctx);
        if (tactic_finished_) {
            tactic_finished_   = false;
            current_skill_idx_ = 0;
            skills_[0]->init(robot, ctx);
        }
        return cmd;
    }
};

class AttackerAimGoalRoleVSS final : public Role {
public:
    explicit AttackerAimGoalRoleVSS(uint8_t id) : Role(id) {}
    std::string name() const override { return "AttackerAimGoalRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        return std::make_shared<AimGoalTactic>();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  BallChallengerAttackerRoleVSS
//
//  Atacante agressivo: sempre vai para a bola independente de distância.
//  Usado quando o time precisa pressionar.
// ─────────────────────────────────────────────────────────────────────────────
class BallChallengerAttackerRoleVSS final : public Role {
public:
    explicit BallChallengerAttackerRoleVSS(uint8_t id) : Role(id) {}
    std::string name() const override { return "BallChallengerAttackerRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        (void)robot; (void)ctx;
        // Sempre ciclo agressivo GoToBall → Kick sem espera
        return std::make_shared<AggressivePushTactic>();
    }
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
