#pragma once

#include "types.hpp"
#include "game_context.hpp"
#include <string>
#include <memory>
#include <algorithm>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  Skill  —  nível mais baixo da hierarquia VSS
//
//  Equivale a todas as XxxSkill_class do LabVIEW.
//  Uma Skill recebe o estado do mundo e devolve um comando de roda.
//
//  Ciclo de vida:
//    1. init()    — chamado uma vez quando a Skill é selecionada
//    2. execute() — chamado a cada frame (60 Hz no FIRASim)
//    3. isFinished() — a Tactic verifica isso para avançar para próxima Skill
//
//  Mapeamento do LabVIEW:
//    Robot in + Game in → execute() → Robot out (comando de roda)
// ─────────────────────────────────────────────────────────────────────────────
class Skill {
public:
    virtual ~Skill() = default;

    // Chamado quando a Skill é (re)iniciada
    // Equivale ao "init" implícito do LabVIEW quando o VI é ativado
    virtual void init(const RobotState& robot, const GameContext& ctx) {
        (void)robot; (void)ctx;
        finished_ = false;
    }

    // Executado a cada frame — devolve o comando de roda
    // É o equivalente direto do corpo principal do VI no LabVIEW
    virtual RobotCommand execute(const RobotState& robot,
                                 const GameContext& ctx) = 0;

    // Verdadeiro quando a Skill completou seu objetivo
    // A Tactic usa isso para encadear Skills
    virtual bool isFinished() const { return finished_; }

    // Nome para debug / logs
    virtual std::string name() const = 0;

protected:
    bool finished_ = false;

    // Helper: limita velocidade das rodas aos limites físicos do robô VSS
    // Valor típico para robôs VSS: ~1.5 m/s por roda
    static constexpr double MAX_WHEEL_SPEED = 1.5;

    RobotCommand clampCommand(RobotCommand cmd) const {
        cmd.wheel_left  = std::clamp(cmd.wheel_left,
                                     -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        cmd.wheel_right = std::clamp(cmd.wheel_right,
                                     -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        return cmd;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  Skill de comportamento nulo — robô para
//  Equivale ao HaltSkill_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class HaltSkill final : public Skill {
public:
    RobotCommand execute(const RobotState& robot,
                         const GameContext&) override
    {
        return RobotCommand::stop(robot.id);
    }
    std::string name() const override { return "HaltSkill"; }
};

} // namespace vss
