#pragma once

#include "play.hpp"
#include "roles.hpp"
#include <cmath>

namespace vss {

// ═════════════════════════════════════════════════════════════════════════════
//
//  PLAYS VSS — todos os plays do seu projeto LabVIEW traduzidos
//
//  Ordem de prioridade (maior = verificado primeiro pelo Playbook):
//    100  HaltPlay          — HALT/TIMEOUT (segurança)
//     90  PenaltyAttackPlay — penalty_ally
//     90  PenaltyDefensePlay— penalty_enemy
//     80  KickoffAllyPlay   — kickoff_ally
//     80  KickoffEnemyPlay  — kickoff_enemy
//     70  GoalKickAllyPlay  — goalkick_ally
//     70  GoalKickEnemyPlay — goalkick_enemy
//     60  FreeBallPlay      — freeball_ally / freeball_enemy
//     50  DirectKickPlay    — direct_kick_ally/enemy
//     50  IndirectKickPlay  — indirect_kick_ally/enemy
//     10  NormalDefensivePlay — jogo normal, bola no nosso campo
//     10  OffensivePlay     — jogo normal, bola no campo adversário
//      0  NormalGamePlayVSS — fallback de jogo normal
//
// ═════════════════════════════════════════════════════════════════════════════


// ─────────────────────────────────────────────────────────────────────────────
//  Helpers internos
// ─────────────────────────────────────────────────────────────────────────────

// Retorna true se a bola está no nosso campo (lado de defesa)
static bool ballInOurHalf(const GameContext& ctx) {
    return (ctx.ball.x * ctx.attack_dir) < 0.0;
}

// Retorna true se a bola está no campo adversário
static bool ballInEnemyHalf(const GameContext& ctx) {
    return (ctx.ball.x * ctx.attack_dir) > 0.0;
}

// Retorna o robô mais próximo da bola (índice 0-2)
static uint8_t closestToBall(const GameContext& ctx) {
    return ctx.closestAllyToBall();
}

// Retorna o robô mais longe da bola (índice 0-2) — bom para goleiro
static uint8_t farthestFromBall(const GameContext& ctx) {
    double worst = -1.0;
    uint8_t idx  = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        double d = std::hypot(ctx.allies[i].x - ctx.ball.x,
                              ctx.allies[i].y - ctx.ball.y);
        if (d > worst) { worst = d; idx = i; }
    }
    return idx;
}

// Terceiro robô (nem o mais próximo nem o mais longe da bola)
static uint8_t middleRobot(const GameContext& ctx) {
    uint8_t closest  = closestToBall(ctx);
    uint8_t farthest = farthestFromBall(ctx);
    for (uint8_t i = 0; i < 3; ++i)
        if (i != closest && i != farthest) return i;
    return 1;
}


// ─────────────────────────────────────────────────────────────────────────────
//  NormalGamePlayVSS  —  fallback de jogo normal
//
//  Atribuição DINÂMICA baseada em posição (reavaliada a cada frame):
//    · Robô mais perto da bola   → NormalAttackerRoleVSS (agressivo)
//    · Robô do meio              → SupportAttackerRoleVSS (suporte, não fogo amigo)
//    · Robô mais longe da bola   → KeeperRoleVSS
//
//  A reavaliação dinâmica garante que se o goleiro está mais próximo da bola
//  (bola no nosso gol), ele se torna temporariamente o atacante.
// ─────────────────────────────────────────────────────────────────────────────
class NormalGamePlayVSS final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::NORMAL_GAME;
    }

    void init(const GameContext& ctx) override {
        // Inicializa com base na distância atual
        reassignIfNeeded(ctx);
    }

    std::array<RobotCommand, 3> execute(const GameContext& ctx) override {
        // REATRIBUI PAPÉIS A CADA FRAME baseado em distâncias atuais
        reassignIfNeeded(ctx);
        return Play::execute(ctx);
    }

    std::string name() const override { return "NormalGamePlayVSS"; }
    int priority() const override { return 0; }

private:
    uint8_t last_atk_ = 255;
    uint8_t last_gk_  = 255;
    uint8_t last_def_ = 255;

    void reassignIfNeeded(const GameContext& ctx) {
        uint8_t atk = closestToBall(ctx);
        uint8_t gk  = farthestFromBall(ctx);
        uint8_t def = middleRobot(ctx);

        // Reatribui apenas se os papéis mudaram (evita reset desnecessário)
        if (atk == last_atk_ && gk == last_gk_ && def == last_def_) return;
        last_atk_ = atk; last_gk_ = gk; last_def_ = def;

        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == atk) r[i] = std::make_shared<NormalAttackerRoleVSS>(i);
            else if (i == gk)  r[i] = std::make_shared<KeeperRoleVSS>(i);
            else r[i] = std::make_shared<SupportAttackerRoleVSS>(i);
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }
};


// ─────────────────────────────────────────────────────────────────────────────
//  NormalDefensivePlay  —  jogo normal com bola no nosso campo
//
//  Atribuição DINÂMICA (reavaliada a cada frame):
//    · Robô mais perto da bola   → BallChallengerAttackerRoleVSS (pressão)
//    · Robô do meio              → DefenderAreaRoleVSS (bloqueia linha)
//    · Robô mais longe           → KeeperRoleVSS
// ─────────────────────────────────────────────────────────────────────────────
class NormalDefensivePlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::NORMAL_GAME
            && ballInOurHalf(ctx);
    }

    void init(const GameContext& ctx) override {
        reassignIfNeeded(ctx);
    }

    std::array<RobotCommand, 3> execute(const GameContext& ctx) override {
        reassignIfNeeded(ctx);
        return Play::execute(ctx);
    }

    std::string name() const override { return "NormalDefensivePlay"; }
    int priority() const override { return 10; }

private:
    uint8_t last_atk_ = 255;
    uint8_t last_gk_  = 255;

    void reassignIfNeeded(const GameContext& ctx) {
        uint8_t atk = closestToBall(ctx);
        uint8_t gk  = farthestFromBall(ctx);
        uint8_t def = middleRobot(ctx);

        if (atk == last_atk_ && gk == last_gk_) return;
        last_atk_ = atk; last_gk_ = gk;

        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == atk) r[i] = std::make_shared<BallChallengerAttackerRoleVSS>(i);
            else if (i == gk) r[i] = std::make_shared<KeeperRoleVSS>(i);
            else r[i] = std::make_shared<DefenderAreaRoleVSS>(i);
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }
};


// ─────────────────────────────────────────────────────────────────────────────
//  OffensivePlay  —  bola no campo adversário
//
//  Atribuição DINÂMICA (reavaliada a cada frame):
//    · Robô mais perto da bola   → AttackerAimGoalRoleVSS (mira e chuta)
//    · Robô do meio              → SupportAttackerRoleVSS (posição, NÃO fogo amigo)
//    · Robô mais longe           → KeeperRoleVSS
//
//  CRÍTICO: O robô do meio usa SupportAttackerRoleVSS, não NormalAttackerRoleVSS,
//  para evitar que dois robôs corram para a mesma bola simultaneamente.
// ─────────────────────────────────────────────────────────────────────────────
class OffensivePlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::NORMAL_GAME
            && ballInEnemyHalf(ctx);
    }

    void init(const GameContext& ctx) override {
        reassignIfNeeded(ctx);
    }

    std::array<RobotCommand, 3> execute(const GameContext& ctx) override {
        reassignIfNeeded(ctx);
        return Play::execute(ctx);
    }

    std::string name() const override { return "OffensivePlay"; }
    int priority() const override { return 10; }

private:
    uint8_t last_atk_ = 255;
    uint8_t last_gk_  = 255;

    void reassignIfNeeded(const GameContext& ctx) {
        uint8_t atk = closestToBall(ctx);
        uint8_t gk  = farthestFromBall(ctx);
        uint8_t sup = middleRobot(ctx);

        if (atk == last_atk_ && gk == last_gk_) return;
        last_atk_ = atk; last_gk_ = gk;

        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == atk) r[i] = std::make_shared<AttackerAimGoalRoleVSS>(i);
            else if (i == gk) r[i] = std::make_shared<KeeperRoleVSS>(i);
            else r[i] = std::make_shared<SupportAttackerRoleVSS>(i);
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }
};


// ─────────────────────────────────────────────────────────────────────────────
//  KickoffAllyPlay  —  nosso kickoff
//
//  Robô mais perto do centro → GoAndKickRoleVSS  (executa o kickoff)
//  Outros dois              → posições fixas de suporte
//
//  Equivale ao KickoffAllyPlayVSS_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class KickoffAllyPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::KICKOFF_ALLY;
    }

    void init(const GameContext& ctx) override {
        // Robô mais perto do centro do campo executa o kickoff
        uint8_t kicker = 0;
        double  best   = 1e9;
        for (uint8_t i = 0; i < 3; ++i) {
            double d = std::hypot(ctx.allies[i].x, ctx.allies[i].y);
            if (d < best) { best = d; kicker = i; }
        }

        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == kicker) {
                r[i] = std::make_shared<GoAndKickRoleVSS>(i);
            } else {
                // Posições de suporte: um lateral, um goleiro
                double sx = -ctx.attack_dir * 0.10;
                double sy = (i == 0) ? 0.20 : -0.20;
                r[i] = std::make_shared<DefenderAreaRoleVSS>(i);
            }
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "KickoffAllyPlay"; }
    int priority() const override { return 80; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  KickoffEnemyPlay  —  kickoff do adversário
//
//  Defensivo: todos recuam para posição defensiva
//  Equivale ao KickoffEnemyPlayVSS_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class KickoffEnemyPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::KICKOFF_ENEMY;
    }

    void init(const GameContext& ctx) override {
        // Um desafiador perto do centro, defensor e goleiro atrás
        uint8_t challenger = closestToBall(ctx);
        uint8_t gk         = farthestFromBall(ctx);
        uint8_t def        = middleRobot(ctx);

        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == challenger) r[i] = std::make_shared<BallChallengerAttackerRoleVSS>(i);
            else if (i == gk)    r[i] = std::make_shared<KeeperRoleVSS>(i);
            else                 r[i] = std::make_shared<DefenderAreaRoleVSS>(i);
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "KickoffEnemyPlay"; }
    int priority() const override { return 80; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  PenaltyAttackPlay  —  penalty a nosso favor
//
//  Cobrador → GoAndKickRoleVSS (PenaltyAttackerRoleVSS no LabVIEW)
//  Outros   → posições fixas fora da área
//
//  Equivale ao PenaltyAttackTacticVSS_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class PenaltyAttackPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::PENALTY_ALLY;
    }

    void init(const GameContext& ctx) override {
        // Cobrador: robô mais perto da bola
        uint8_t shooter = closestToBall(ctx);

        // Kick especial de penalti
        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == shooter) {
                // Usa KickPenalty via GoAndKick com modo penalti
                r[i] = std::make_shared<GoAndKickRoleVSS>(i);
            } else {
                r[i] = std::make_shared<HaltRole>(i);  // fora da área
            }
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "PenaltyAttackPlay"; }
    int priority() const override { return 90; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  PenaltyDefensePlay  —  penalty contra nós
//
//  Goleiro → KeeperRoleVSS (defende o penalty)
//  Outros  → HaltRole (parados fora da área)
//
//  Equivale ao PenaltyDefenseRole_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class PenaltyDefensePlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::PENALTY_ENEMY;
    }

    void init(const GameContext& ctx) override {
        // Goleiro: robô mais perto do nosso gol
        auto our_goal = ctx.ourGoalCenter();
        double best   = 1e9;
        uint8_t gk    = 2;
        for (uint8_t i = 0; i < 3; ++i) {
            double d = std::hypot(ctx.allies[i].x - our_goal.x,
                                  ctx.allies[i].y - our_goal.y);
            if (d < best) { best = d; gk = i; }
        }

        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            r[i] = (i == gk)
                   ? std::static_pointer_cast<Role>(std::make_shared<KeeperRoleVSS>(i))
                   : std::static_pointer_cast<Role>(std::make_shared<HaltRole>(i));
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "PenaltyDefensePlay"; }
    int priority() const override { return 90; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  FreeBallPlay  —  bola livre (freeball ally ou enemy)
//
//  Nossa freeball  → GoAndKickRoleVSS + suporte
//  Freeball deles  → defensivo (challenger + defender + keeper)
//
//  Equivale ao FreeBallPlayVSS_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class FreeBallPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::FREEBALL_ALLY
            || ctx.situation == GameSituation::FREEBALL_ENEMY;
    }

    void init(const GameContext& ctx) override {
        uint8_t closest  = closestToBall(ctx);
        uint8_t farthest = farthestFromBall(ctx);
        uint8_t middle   = middleRobot(ctx);

        std::array<std::shared_ptr<Role>, 3> r;

        if (ctx.situation == GameSituation::FREEBALL_ALLY) {
            for (uint8_t i = 0; i < 3; ++i) {
                if (i == closest)  r[i] = std::make_shared<GoAndKickRoleVSS>(i);
                else if (i == farthest) r[i] = std::make_shared<KeeperRoleVSS>(i);
                else r[i] = std::make_shared<DefenderAreaRoleVSS>(i);
            }
        } else {
            // Freeball adversário: defensivo
            for (uint8_t i = 0; i < 3; ++i) {
                if (i == closest)  r[i] = std::make_shared<BallChallengerAttackerRoleVSS>(i);
                else if (i == farthest) r[i] = std::make_shared<KeeperRoleVSS>(i);
                else r[i] = std::make_shared<DefenderAreaRoleVSS>(i);
            }
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "FreeBallPlay"; }
    int priority() const override { return 60; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  GoalKickAllyPlay  —  tiro de meta a nosso favor
//
//  Goleiro executa o tiro → GoalKickKeeperRoleVSS
//  Outros dois abrem para receber o passe
//
//  Equivale ao GoalKickAllyPlayVSS_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class GoalKickAllyPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::GOAL_KICK_ALLY;
    }

    void init(const GameContext& ctx) override {
        // Goleiro: robô mais perto do nosso gol
        auto our_goal = ctx.ourGoalCenter();
        double best   = 1e9;
        uint8_t gk    = 2;
        for (uint8_t i = 0; i < 3; ++i) {
            double d = std::hypot(ctx.allies[i].x - our_goal.x,
                                  ctx.allies[i].y - our_goal.y);
            if (d < best) { best = d; gk = i; }
        }

        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == gk) r[i] = std::make_shared<GoalKickKeeperRoleVSS>(i);
            else         r[i] = std::make_shared<NormalAttackerRoleVSS>(i);
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "GoalKickAllyPlay"; }
    int priority() const override { return 70; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  GoalKickEnemyPlay  —  tiro de meta do adversário
//
//  Defensivo: todos recuam
//  Equivale ao GoalKickEnemyPlayVSS_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class GoalKickEnemyPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::GOAL_KICK_ENEMY;
    }

    void init(const GameContext& ctx) override {
        uint8_t challenger = closestToBall(ctx);
        uint8_t gk         = farthestFromBall(ctx);

        std::array<std::shared_ptr<Role>, 3> r;
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == challenger) r[i] = std::make_shared<BallChallengerAttackerRoleVSS>(i);
            else if (i == gk)    r[i] = std::make_shared<KeeperRoleVSS>(i);
            else                 r[i] = std::make_shared<DefenderAreaRoleVSS>(i);
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "GoalKickEnemyPlay"; }
    int priority() const override { return 70; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  DirectKickPlay / IndirectKickPlay
//  Equivalem a DirectEnemyPlay_class e IndirectEnemyPlay_class do LabVIEW
//  Comportamento idêntico ao FreeBall — reutiliza lógica
// ─────────────────────────────────────────────────────────────────────────────
class DirectKickPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::DIRECT_KICK_ALLY
            || ctx.situation == GameSituation::DIRECT_KICK_ENEMY;
    }

    void init(const GameContext& ctx) override {
        uint8_t closest  = closestToBall(ctx);
        uint8_t farthest = farthestFromBall(ctx);

        std::array<std::shared_ptr<Role>, 3> r;
        bool ours = (ctx.situation == GameSituation::DIRECT_KICK_ALLY);
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == closest)
                r[i] = ours
                    ? std::static_pointer_cast<Role>(std::make_shared<GoAndKickRoleVSS>(i))
                    : std::static_pointer_cast<Role>(std::make_shared<BallChallengerAttackerRoleVSS>(i));
            else if (i == farthest)
                r[i] = std::make_shared<KeeperRoleVSS>(i);
            else
                r[i] = std::make_shared<DefenderAreaRoleVSS>(i);
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "DirectKickPlay"; }
    int priority() const override { return 50; }
};


class IndirectKickPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::INDIRECT_KICK_ALLY
            || ctx.situation == GameSituation::INDIRECT_KICK_ENEMY;
    }

    void init(const GameContext& ctx) override {
        uint8_t closest  = closestToBall(ctx);
        uint8_t farthest = farthestFromBall(ctx);

        std::array<std::shared_ptr<Role>, 3> r;
        bool ours = (ctx.situation == GameSituation::INDIRECT_KICK_ALLY);
        for (uint8_t i = 0; i < 3; ++i) {
            if (i == closest)
                r[i] = ours
                    ? std::static_pointer_cast<Role>(std::make_shared<GoAndKickRoleVSS>(i))
                    : std::static_pointer_cast<Role>(std::make_shared<BallChallengerAttackerRoleVSS>(i));
            else if (i == farthest)
                r[i] = std::make_shared<KeeperRoleVSS>(i);
            else
                r[i] = std::make_shared<DefenderAreaRoleVSS>(i);
        }
        assignRoles(r[0], r[1], r[2], ctx);
    }

    std::string name() const override { return "IndirectKickPlay"; }
    int priority() const override { return 50; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  VSSPlaybook  —  Playbook pré-configurado com todos os plays
//
//  Use este em vez do Playbook base. Já registra todos os plays
//  na ordem correta de prioridade.
//
//  Uso:
//    VSSPlaybook pb;
//    // a cada frame:
//    auto cmds = pb.update(ctx);
// ─────────────────────────────────────────────────────────────────────────────
class VSSPlaybook : public Playbook {
public:
    VSSPlaybook() {
        // Prioridade alta → baixa (HaltPlay já registrado na base com p=100)
        addPlay(std::make_shared<PenaltyAttackPlay>());   // p=90
        addPlay(std::make_shared<PenaltyDefensePlay>());  // p=90
        addPlay(std::make_shared<KickoffAllyPlay>());     // p=80
        addPlay(std::make_shared<KickoffEnemyPlay>());    // p=80
        addPlay(std::make_shared<GoalKickAllyPlay>());    // p=70
        addPlay(std::make_shared<GoalKickEnemyPlay>());   // p=70
        addPlay(std::make_shared<FreeBallPlay>());        // p=60
        addPlay(std::make_shared<DirectKickPlay>());      // p=50
        addPlay(std::make_shared<IndirectKickPlay>());    // p=50
        addPlay(std::make_shared<OffensivePlay>());       // p=10
        addPlay(std::make_shared<NormalDefensivePlay>()); // p=10
        addPlay(std::make_shared<NormalGamePlayVSS>());   // p=0  fallback
    }
};

} // namespace vss
