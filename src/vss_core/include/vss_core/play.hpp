#pragma once

#include "role.hpp"
#include <array>
#include <memory>
#include <string>

namespace vss {

// ─────────────────────────────────────────────────────────────────────────────
//  Play  —  nível mais alto da hierarquia VSS
//
//  Equivale a todas as XxxPlay_class do LabVIEW.
//  Um Play controla os 3 robôs do time simultaneamente, atribuindo um Role
//  para cada um e executando todos a cada frame.
//
//  O Playbook (classe separada) chama isApplicable() em todos os plays
//  registrados e ativa o primeiro que retornar true.
//
//  Mapeamento do LabVIEW:
//    Os Play_class recebiam o cluster Game, distribuíam para sub-VIs de Role
//    e devolviam os comandos dos 3 robôs.
// ─────────────────────────────────────────────────────────────────────────────
class Play {
public:
    virtual ~Play() = default;

    // ── Interface obrigatória ─────────────────────────────────────────────

    // Retorna true se este Play deve estar ativo na situação atual
    // O Playbook percorre os plays em prioridade e ativa o primeiro válido
    virtual bool isApplicable(const GameContext& ctx) const = 0;

    // Chamado uma vez quando o play é ativado
    // Aqui a subclasse cria os Roles e faz atribuição (quem é atacante, etc.)
    virtual void init(const GameContext& ctx) = 0;

    // Chamado a cada frame — executa os 3 roles e devolve 3 comandos
    virtual std::array<RobotCommand, 3> execute(const GameContext& ctx) {
        std::array<RobotCommand, 3> cmds;
        for (int i = 0; i < 3; ++i) {
            if (roles_[i]) {
                cmds[i] = roles_[i]->execute(ctx);
            } else {
                cmds[i] = RobotCommand::stop(static_cast<uint8_t>(i));
            }
        }
        return cmds;
    }

    // Nome para logs e debug
    virtual std::string name() const = 0;

    // Retorna o nome do papel atual do robô i
    virtual std::string roleName(int i) const {
        if (i >= 0 && i < 3 && roles_[i]) return roles_[i]->name();
        return "None";
    }

    // Prioridade — plays com maior prioridade são testados primeiro
    // Override para ajustar (default = 0)
    virtual int priority() const { return 0; }

protected:
    // roles_[0], roles_[1], roles_[2] → robôs 0, 1, 2
    std::array<std::shared_ptr<Role>, 3> roles_;

    // Helper: atribui roles baseado no GameContext
    // Subclasses chamam isso no init()
    void assignRoles(std::shared_ptr<Role> r0,
                     std::shared_ptr<Role> r1,
                     std::shared_ptr<Role> r2,
                     const GameContext& ctx)
    {
        roles_[0] = std::move(r0);
        roles_[1] = std::move(r1);
        roles_[2] = std::move(r2);
        for (int i = 0; i < 3; ++i) {
            if (roles_[i]) roles_[i]->init(ctx);
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  HaltPlay  —  todos os robôs parados
//  Ativado quando situation == HALT ou TIMEOUT
//  Equivale ao HaltPlay_class do LabVIEW
// ─────────────────────────────────────────────────────────────────────────────
class HaltPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::HALT ||
               ctx.situation == GameSituation::TIMEOUT;
    }

    void init(const GameContext& ctx) override {
        assignRoles(
            std::make_shared<HaltRole>(0),
            std::make_shared<HaltRole>(1),
            std::make_shared<HaltRole>(2),
            ctx
        );
    }

    std::string name() const override { return "HaltPlay"; }
    int priority() const override { return 100; }  // máxima prioridade
};

// ─────────────────────────────────────────────────────────────────────────────
//  Playbook  —  seleciona e gerencia o Play ativo
//
//  Equivale ao Playbook_class do LabVIEW.
//  Registre todos os plays com addPlay() em ordem de prioridade.
//  A cada frame, chame update() e depois execute().
// ─────────────────────────────────────────────────────────────────────────────
class Playbook {
public:
    Playbook() {
        // HaltPlay sempre registrado como fallback
        addPlay(std::make_shared<HaltPlay>());
    }

    void addPlay(std::shared_ptr<Play> play) {
        plays_.push_back(std::move(play));
        // Ordena por prioridade decrescente
        std::sort(plays_.begin(), plays_.end(),
            [](const auto& a, const auto& b) {
                return a->priority() > b->priority();
            });
    }

    // Chame a cada frame — seleciona o play correto e executa
    std::array<RobotCommand, 3> update(const GameContext& ctx) {
        // Encontra o play aplicável de maior prioridade
        for (auto& play : plays_) {
            if (play->isApplicable(ctx)) {
                // Troca de play se necessário
                if (!active_play_ || active_play_->name() != play->name()) {
                    active_play_ = play;
                    active_play_->init(ctx);
                }
                return active_play_->execute(ctx);
            }
        }

        // Fallback de segurança — nunca deve chegar aqui se HaltPlay registrado
        return {
            RobotCommand::stop(0),
            RobotCommand::stop(1),
            RobotCommand::stop(2)
        };
    }

    std::string activePlayName() const {
        return active_play_ ? active_play_->name() : "none";
    }

    std::string activeRoleName(int i) const {
        return active_play_ ? active_play_->roleName(i) : "none";
    }

private:
    std::vector<std::shared_ptr<Play>> plays_;
    std::shared_ptr<Play> active_play_;
};

} // namespace vss
