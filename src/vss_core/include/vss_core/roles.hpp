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
//  LabVIEW: KeeperRole → Tacticbook → DefendGoalTactic → States
//
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
        double goal_x = -ctx.attack_dir * (ctx.field.max_x() - 0.05);

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
//  LabVIEW: DefenderAreaRole → Tacticbook → DefendAreaTactic → States
//  Parâmetro: Number of Defenders Without Duelist (default = 0 no LabVIEW)
//
//  Lógica: defensor se posiciona entre a bola e o nosso gol,
//  a uma distância calculada. Se a bola está muito perto, vai buscar.
// ─────────────────────────────────────────────────────────────────────────────
class DefendAreaTactic final : public Tactic {
public:
    // num_defenders_without_duelist: parâmetro do LabVIEW (default 0)
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
        // Ajuste por número de defensores (LabVIEW: Number of Defenders)
        double defend_dist = 0.20 + num_defenders_ * 0.05;
        defend_dist = std::min(defend_dist, dist_ball_goal * 0.6);

        double target_x = our_goal.x + (dx_ball_goal / dist_ball_goal) * (-defend_dist);
        double target_y = our_goal.y + (dy_ball_goal / dist_ball_goal) * (-defend_dist);

        // Clamp dentro do campo
        target_x = std::clamp(target_x,
                               ctx.field.min_x() + 0.05f,
                               ctx.field.max_x() - 0.05f);
        target_y = std::clamp(target_y,
                               ctx.field.min_y() + 0.05f,
                               ctx.field.max_y() - 0.05f);

        GoToPositionSkill gtp(target_x, target_y);
        return gtp.execute(robot, ctx);
    }

private:
    int num_defenders_;
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
//  LabVIEW (imagem 2):
//    Tacticbook com UMA tática: DefendGoalTactic
//    Sem condições de troca — sempre DefendGoalTactic
//    First call? → inicializa tacticbook com tactic_index = 0
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
//
//  LabVIEW (imagem 5):
//    Tacticbook com UMA tática: DefendAreaTactic
//    Parâmetro: Number of Defenders Without Duelist = 0 (default do LabVIEW)
//    Sem condições de troca — sempre DefendAreaTactic
// ─────────────────────────────────────────────────────────────────────────────
class DefenderAreaRoleVSS final : public Role {
public:
    // num_defenders: "Number of Defenders Without Duelist" do LabVIEW
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
//  LabVIEW: NormalAttackerRoleVSS — atacante padrão do jogo normal
//  Lógica de seleção de tática:
//    · Longe da bola   → GoToBallSkill (aproxima)
//    · Perto da bola + mal alinhado → RotateToAngleSkill (alinha)
//    · Perto da bola + alinhado     → KickSkill (chuta)
//  Loop: depois do chute, volta para GoToBall
// ─────────────────────────────────────────────────────────────────────────────
class NormalAttackerRoleVSS final : public Role {
public:
    explicit NormalAttackerRoleVSS(uint8_t id) : Role(id) {}
    std::string name() const override { return "NormalAttackerRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                          const GameContext& ctx) override
    {
        double dist = distToBall(robot, ctx);

        // Longe da bola → vai buscar
        if (dist > BALL_NEAR_THRESHOLD) {
            return std::make_shared<SingleSkillTactic>(
                std::make_shared<GoToBallSkill>());
        }

        // Perto da bola → ciclo de chute
        return std::make_shared<GoToBallAndKickTactic>();
    }

private:
    static constexpr double BALL_NEAR_THRESHOLD = 0.25;  // metros
};

// ─────────────────────────────────────────────────────────────────────────────
//  AttackerAimGoalRoleVSS
//
//  LabVIEW: AttackerAimGoalRoleVSS — atacante que mira o gol antes de chutar
//  Diferença do NormalAttacker: adiciona fase de alinhamento explícita
//    1. GoToBall (aproxima pela traseira da bola em relação ao gol)
//    2. RotateToAngle (alinha para o gol)
//    3. Kick (chuta)
// ─────────────────────────────────────────────────────────────────────────────
class AimGoalTactic final : public Tactic {
public:
    std::string name() const override { return "AimGoalTactic"; }
    bool isFinished() const override { return false; }

    AimGoalTactic() {
        addSkill<GoToBallSkill>();
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
//  LabVIEW: BallChallengerAttackerRoleVSS
//  Atacante agressivo: sempre vai para a bola independente de distância.
//  Sem fase de alinhamento — chuta de onde estiver.
//  Usado quando o time precisa pressionar e não tem tempo para alinhar.
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
        return std::make_shared<GoToBallAndKickTactic>();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  GoAndKickRoleVSS
//
//  LabVIEW: GoAndKickRoleVSS — vai até a bola e chuta uma vez (não loop)
//  Usado em set-pieces: kickoff, freeball, direct kick
//  Executa GoToBall → Kick uma única vez e finaliza
// ─────────────────────────────────────────────────────────────────────────────
class GoAndKickOneTimeTactic final : public Tactic {
public:
    std::string name() const override { return "GoAndKickOneTimeTactic"; }

    GoAndKickOneTimeTactic() {
        addSkill<GoToBallSkill>();
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
//  LabVIEW: GoalKickKeeperRoleVSS — goleiro executa o tiro de meta
//  Sai da posição de goleiro, vai até a bola e chuta uma vez,
//  depois retorna para DefendGoalTactic
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
