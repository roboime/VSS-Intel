#pragma once

#include "skill.hpp"
#include <vector>
#include <memory>
#include <string>
#include <stdexcept>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  Tactic  —  segundo nível da hierarquia VSS
//
//  Equivale a todas as XxxTactic_class do LabVIEW.
//  Uma Tactic contém uma sequência ordenada de Skills e avança entre elas
//  conforme cada uma termina (isFinished() == true).
//
//  Exemplo da hierarquia original:
//    AttackerBlockTactic → GoToBallSkill → KickSkill
//    GoToBallTactic      → GoToBallSkill (loop)
//    KickToPassReceiverTactic → OrientSkill → KickSkill
//
//  Mapeamento do LabVIEW:
//    O VI da tática encadeava sub-VIs de Skill via estruturas Case/Sequence.
//    Aqui isso vira um índice (current_skill_idx_) que avança automaticamente.
// ─────────────────────────────────────────────────────────────────────────────
class Tactic {
public:
    virtual ~Tactic() = default;

    // ── Interface obrigatória ─────────────────────────────────────────────

    virtual void init(const RobotState& robot, const GameContext& ctx) {
        current_skill_idx_ = 0;
        tactic_finished_   = false;
        if (!skills_.empty()) {
            skills_[0]->init(robot, ctx);
        }
    }

    virtual RobotCommand execute(const RobotState& robot,
                                 const GameContext& ctx)
    {
        if (skills_.empty() || tactic_finished_) {
            return RobotCommand::stop(robot.id);
        }

        Skill* current = skills_[current_skill_idx_].get();
        RobotCommand cmd = current->execute(robot, ctx);

        // Avança para próxima Skill quando a atual terminar
        if (current->isFinished()) {
            current_skill_idx_++;
            if (current_skill_idx_ >= skills_.size()) {
                tactic_finished_ = true;
            } else {
                skills_[current_skill_idx_]->init(robot, ctx);
            }
        }

        return cmd;
    }

    virtual bool isFinished() const { return tactic_finished_; }

    virtual std::string name() const = 0;

    // Reinicia do início (útil quando o Role troca de tática)
    void reset(const RobotState& robot, const GameContext& ctx) {
        init(robot, ctx);
    }

    // Skill atual para debug
    std::string currentSkillName() const {
        if (current_skill_idx_ < skills_.size())
            return skills_[current_skill_idx_]->name();
        return "none";
    }

protected:
    std::vector<std::shared_ptr<Skill>> skills_;
    size_t current_skill_idx_ = 0;
    bool   tactic_finished_   = false;

    // Helper para subclasses adicionarem skills na construção
    // Uso:  addSkill<GoToBallSkill>();
    //       addSkill<KickSkill>(KickSkill::Params{...});
    template<typename T, typename... Args>
    void addSkill(Args&&... args) {
        skills_.push_back(std::make_shared<T>(std::forward<Args>(args)...));
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  SingleSkillTactic  —  Tática que executa uma única Skill indefinidamente
//
//  Usada para táticas simples como "ficar parado" ou "ir para posição"
//  onde não há transição de estado.
//  Equivale aos casos do LabVIEW onde a tática tinha apenas um sub-VI.
// ─────────────────────────────────────────────────────────────────────────────
class SingleSkillTactic : public Tactic {
public:
    explicit SingleSkillTactic(std::shared_ptr<Skill> skill) {
        skills_.push_back(std::move(skill));
    }

    // Nunca termina — o Role decide quando trocar de tática
    bool isFinished() const override { return false; }

    RobotCommand execute(const RobotState& robot,
                         const GameContext& ctx) override
    {
        return skills_[0]->execute(robot, ctx);
    }

    std::string name() const override {
        return "SingleSkill[" + skills_[0]->name() + "]";
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  HaltTactic  —  para o robô
// ─────────────────────────────────────────────────────────────────────────────
class HaltTactic final : public Tactic {
public:
    HaltTactic() { addSkill<HaltSkill>(); }
    bool isFinished() const override { return false; }
    std::string name() const override { return "HaltTactic"; }
};

} // namespace vss
