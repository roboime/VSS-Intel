#pragma once

#include "tactic.hpp"
#include <memory>
#include <string>
#include <functional>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  Role  —  terceiro nível da hierarquia VSS
//
//  Equivale a todas as XxxRole_class do LabVIEW.
//  Um Role define o PAPEL de um robô específico dentro do play atual
//  (atacante, defensor, goleiro, duelist, marker, etc.).
//
//  O Role é responsável por:
//    1. Avaliar o GameContext e decidir QUAL Tactic usar agora
//    2. Trocar de Tactic quando a situação mudar (ex: bola longe → GoToBall,
//       bola perto → Kick)
//    3. Delegar o execute() para a Tactic ativa
//
//  Mapeamento do LabVIEW:
//    Os Role_class tinham estruturas Case que selecionavam sub-VIs de Tactic
//    baseados em condições do campo. Aqui isso vira selectTactic().
// ─────────────────────────────────────────────────────────────────────────────
class Role {
public:
    virtual ~Role() = default;

    // ID do robô que este Role controla (índice 0, 1 ou 2)
    explicit Role(uint8_t robot_id) : robot_id_(robot_id) {}

    // ── Interface obrigatória ─────────────────────────────────────────────

    // Chamado uma vez quando o Role é ativado pelo Play
    virtual void init(const GameContext& ctx) {
        (void)ctx;
        current_tactic_.reset();
        initialized_ = true;
    }

    // Chamado a cada frame:
    //   1. Avalia o contexto e decide a tática correta (selectTactic)
    //   2. Troca a tática se necessário
    //   3. Executa a tática ativa
    virtual RobotCommand execute(const GameContext& ctx) {
        const RobotState& robot = ctx.allies[robot_id_];

        // Deixa a subclasse decidir qual tática usar agora
        auto desired = selectTactic(robot, ctx);

        // Troca de tática se mudou
        if (!current_tactic_ ||
            current_tactic_->name() != desired->name() ||
            current_tactic_->isFinished())
        {
            current_tactic_ = std::move(desired);
            current_tactic_->init(robot, ctx);
        }

        return current_tactic_->execute(robot, ctx);
    }

    virtual std::string name() const = 0;

    uint8_t robotId() const { return robot_id_; }

protected:
    uint8_t robot_id_;
    std::shared_ptr<Tactic> current_tactic_;
    bool initialized_ = false;

    // ── Método que subclasses DEVEM implementar ───────────────────────────
    //
    // Retorna a Tactic que deve estar ativa agora, dado o contexto.
    // É aqui que fica toda a lógica de decisão do Role.
    //
    // Exemplo para AttackerRole:
    //   if (dist_to_ball < 0.15) return make_shared<KickTactic>();
    //   else                     return make_shared<GoToBallTactic>();
    //
    // A troca SÓ acontece se o nome da tática retornada for diferente
    // da tática atual OU se a atual terminou — evita reinicializações
    // desnecessárias a cada frame.
    virtual std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                                  const GameContext& ctx) = 0;

    // ── Helpers geométricos para subclasses ──────────────────────────────

    double distToBall(const RobotState& robot, const GameContext& ctx) const {
        return std::hypot(robot.x - ctx.ball.x, robot.y - ctx.ball.y);
    }

    double distToPoint(const RobotState& robot, double tx, double ty) const {
        return std::hypot(robot.x - tx, robot.y - ty);
    }

    // Verifica se o robô está alinhado para chutar em direção ao gol
    // threshold em radianos (~15° por padrão)
    bool isAlignedToShoot(const RobotState& robot,
                          const GameContext& ctx,
                          double threshold_rad = 0.26) const
    {
        auto goal = ctx.enemyGoalCenter();
        double angle_to_goal = std::atan2(goal.y - robot.y, goal.x - robot.x);
        double error = normalizeAngle(angle_to_goal - robot.theta);
        return std::abs(error) < threshold_rad;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  HaltRole  —  robô parado, usado em HALT e TimeoutPlay
// ─────────────────────────────────────────────────────────────────────────────
class HaltRole final : public Role {
public:
    explicit HaltRole(uint8_t id) : Role(id) {}
    std::string name() const override { return "HaltRole"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState&,
                                          const GameContext&) override
    {
        return std::make_shared<HaltTactic>();
    }
};

} // namespace vss
