# VSS IEEE — Arquitetura do Software de Estratégia

Documentação técnica do sistema de tomada de decisão dos robôs VSS IEEE 3v3,
implementado em ROS 2 (C++17) com simulação via FIRASim.

---

## Índice

1. [Visão Geral](#visão-geral)
2. [Estrutura de Pacotes ROS 2](#estrutura-de-pacotes-ros-2)
3. [Hierarquia de Decisão](#hierarquia-de-decisão)
4. [Fluxo de Execução (60 Hz)](#fluxo-de-execução-60-hz)
5. [Camadas em Detalhe](#camadas-em-detalhe)
   - [Skill](#skill)
   - [Tactic](#tactic)
   - [Role](#role)
   - [Play](#play)
   - [Playbook](#playbook)
6. [Skills Implementadas](#skills-implementadas)
7. [Táticas e Roles](#táticas-e-roles)
8. [Plays e Situações de Jogo](#plays-e-situações-de-jogo)
9. [OrbitAttackerSkill — Máquina de Estados do Atacante](#orbitattackerskill--máquina-de-estados-do-atacante)
10. [Pipeline de Controle de Velocidade](#pipeline-de-controle-de-velocidade)
11. [Tipos de Dados Fundamentais](#tipos-de-dados-fundamentais)
12. [Como Adicionar um Novo Comportamento](#como-adicionar-um-novo-comportamento)
13. [Registro de Bugs Corrigidos](#registro-de-bugs-corrigidos)

---

## Visão Geral

O projeto implementa um **time completo de futebol de robôs VSS 3v3** rodando em ROS 2.
Cada robô é um cubo diferencial de **7,5 × 7,5 cm**, controlado por velocidade de roda individual (m/s).

```
FIRASim ──── vss_vision ──── vss_fira_bridge ──── vss_strategy ──── FIRASim
  (simulador)   (visão/EKF)      (bridge)          (estratégia)      (comandos)
```

O nó de estratégia (`vss_strategy`) roda a **60 Hz fixos**, lê o estado do mundo via tópicos ROS 2
e publica comandos de velocidade de roda para os 3 robôs.

---

## Estrutura de Pacotes ROS 2

```
src/
├── vss_core/           # Toda a lógica de jogo — header-only (sem .cpp)
│   └── include/vss_core/
│       ├── types.hpp           # Structs base: RobotState, BallState, RobotCommand, geo::
│       ├── game_context.hpp    # GameContext — cluster de dados passado para tudo
│       ├── skill.hpp           # Skill (interface) + APF + avoidCollisions + applyLinearRamp
│       ├── tactic.hpp          # Tactic (interface) + SingleSkillTactic + HaltTactic
│       ├── role.hpp            # Role (interface) + HaltRole
│       ├── play.hpp            # Play (interface) + HaltPlay + Playbook
│       ├── go_to_ball_skill.hpp        # Skill de perseguição da bola com APF
│       ├── go_to_position_skill.hpp    # Skill de posicionamento fixo (goleiro/defensor)
│       ├── kick_skill.hpp              # Skill de chute: ALIGN → CHARGE → UNSTICK → DONE
│       ├── vector_attacker_skill.hpp   # VectorAttackerSkill + OrbitAttackerSkill
│       ├── wait_ball_kicker_skill.hpp  # Aguarda posição de chute
│       ├── rotate_to_angle_skill.hpp   # Rotação para ângulo alvo
│       ├── roles.hpp                   # Todas as táticas e roles concretos
│       ├── plays.hpp                   # Todos os plays + VSSPlaybook
│       └── vss_core.hpp                # Include único (use este nos nós ROS 2)
│
├── vss_strategy/       # Nó ROS 2 de estratégia
│   └── src/strategy_node.cpp   # Subscribers, timer 60Hz, slew limiter, publisher
│
├── vss_fira_bridge/    # Bridge FIRASim ↔ ROS 2
├── vss_vision/         # Visão computacional + filtro EKF
├── vss_msgs/           # Mensagens ROS 2 customizadas
└── vss_bringup/        # Launch files
```

> **Nota:** `vss_core` é **header-only**. Não há arquivos `.cpp` — toda a lógica compila
> junto com qualquer nó que incluir `vss_core/vss_core.hpp`.

---

## Hierarquia de Decisão

O sistema segue uma hierarquia de 5 camadas inspirada na arquitetura original em LabVIEW:

```
┌─────────────────────────────────────────────────────┐
│  PLAYBOOK  — seleciona qual Play está ativo          │
│  (baseado em GameSituation: normal, kickoff, penalty)│
└──────────────────┬──────────────────────────────────┘
                   │ 1 Play ativo por vez
┌──────────────────▼──────────────────────────────────┐
│  PLAY  — controla os 3 robôs simultaneamente         │
│  (atribui um Role para cada robô)                    │
└──────────┬───────────┬───────────┬──────────────────┘
           │R0         │R1         │R2
     ┌─────▼──┐  ┌─────▼──┐  ┌────▼───┐
     │  ROLE  │  │  ROLE  │  │  ROLE  │
     │(atacante│  │(defens.)│  │(goleiro│
     └─────┬──┘  └─────┬──┘  └────┬───┘
           │           │          │
     ┌─────▼──┐  ┌─────▼──┐  ┌────▼───┐
     │ TACTIC │  │ TACTIC │  │ TACTIC │
     │(sequência│  │        │  │        │
     │de skills)│  │        │  │        │
     └─────┬──┘  └─────┬──┘  └────┬───┘
           │           │          │
     ┌─────▼──┐  ┌─────▼──┐  ┌────▼───┐
     │ SKILL  │  │ SKILL  │  │ SKILL  │
     │v,ω→cmd │  │        │  │        │
     └────────┘  └────────┘  └────────┘
```

---

## Fluxo de Execução (60 Hz)

A cada frame de 16ms, o `StrategyNode` executa:

```
1. Monta GameContext a partir dos tópicos ROS 2
   (ball_state, robot_raw_0/1/2, game_state)

2. VSSPlaybook::update(ctx)
   └── Playbook::update()
       ├── Itera plays em ordem de prioridade
       ├── Ativa o primeiro Play::isApplicable() == true
       └── Play::execute()
           ├── Role[0]::execute() → Tactic::execute() → Skill::execute() → RobotCommand
           ├── Role[1]::execute() → ...
           └── Role[2]::execute() → ...

3. Slew Rate Limiter Dinâmico (strategy_node.cpp)
   ├── Se cmd.is_fast_slew == false: Δv_max = 0,15 m/s por frame
   └── Se cmd.is_fast_slew == true:  Δv_max = 0,45 m/s por frame

4. Publica RobotCommandArray em /robot_commands
```

---

## Camadas em Detalhe

### Skill

**Arquivo:** [`skill.hpp`](src/vss_core/include/vss_core/skill.hpp)

A camada mais baixa. Recebe o estado do mundo e devolve **um comando de roda** (`RobotCommand`).

```cpp
class Skill {
    virtual void init(const RobotState& robot, const GameContext& ctx);
    virtual RobotCommand execute(const RobotState& robot, const GameContext& ctx) = 0;
    virtual bool isFinished() const;  // usado pela Tactic para avançar
};
```

Utilitários disponíveis para todas as Skills (herdados):

| Método | Descrição |
|--------|-----------|
| `applyAPF(...)` | Campo Potencial Artificial: atração ao alvo + repulsão de robôs/paredes |
| `avoidCollisions(v)` | Escala `v` linear quando há robôs/paredes próximos no cone frontal |
| `applyLinearRamp(target, current, accel)` | Perfil trapezoidal de aceleração (slew interno) |
| `applyAngularRamp(err, omega, max)` | Suavização de velocidade angular |
| `clampCommand(cmd)` | Limita rodas a ±2,5 m/s |

---

### Tactic

**Arquivo:** [`tactic.hpp`](src/vss_core/include/vss_core/tactic.hpp)

Encadeia uma sequência de Skills. Avança para a próxima quando `isFinished() == true`.

```cpp
class Tactic {
    // Executa skill atual; avança quando ela terminar
    virtual RobotCommand execute(const RobotState& robot, const GameContext& ctx);
    virtual bool isFinished() const;
};
```

**Tipos disponíveis:**

| Classe | Comportamento |
|--------|---------------|
| `Tactic` (base) | Sequência de Skills, avança automaticamente |
| `SingleSkillTactic` | Uma única Skill, nunca termina (loop contínuo) |
| `HaltTactic` | Para o robô |

**Importante:** A `Tactic` deve ser um **membro persistente** do Role (não recriada a cada frame).
Se a Tactic for destruída e recriada a cada frame, o estado interno das Skills (rampas, máquinas de estado) é reiniciado 60×/s.

---

### Role

**Arquivo:** [`role.hpp`](src/vss_core/include/vss_core/role.hpp)

Define o **papel** de um robô. Decide qual `Tactic` usar a cada frame.

```cpp
class Role {
    // Subclasses implementam este método:
    virtual std::shared_ptr<Tactic> selectTactic(const RobotState& robot,
                                                  const GameContext& ctx) = 0;
};
```

O `Role::execute()` base compara o nome da tática retornada com a atual — a troca só acontece se a tática **mudou de nome ou terminou**, evitando reinicializações desnecessárias.

**Padrão correto (tactic_ persistente):**

```cpp
class MeuRoleVSS final : public Role {
public:
    explicit MeuRoleVSS(uint8_t id)
        : Role(id)
        , tactic_(std::make_shared<MinhaTatica>())  // criada UMA vez
    {}
protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState&, const GameContext&) override {
        return tactic_;  // sempre o mesmo objeto — estado preservado
    }
private:
    std::shared_ptr<MinhaTatica> tactic_;
};
```

**Anti-padrão (reinicia o estado 60×/s — EVITAR):**

```cpp
// ❌ ERRADO: destrói e recria a tática (e a skill interna) todo frame
std::shared_ptr<Tactic> selectTactic(...) override {
    return std::make_shared<MinhaTatica>();  // last_v_ zerado 60×/s
}
```

---

### Play

**Arquivo:** [`play.hpp`](src/vss_core/include/vss_core/play.hpp)

Controla os **3 robôs simultaneamente**. Atribui Roles via `assignRoles()`.

```cpp
class Play {
    virtual bool isApplicable(const GameContext& ctx) const = 0;  // quando usar?
    virtual void init(const GameContext& ctx) = 0;                 // atribui roles
    virtual std::array<RobotCommand, 3> execute(const GameContext& ctx);
    virtual int priority() const { return 0; }
};
```

---

### Playbook

**Arquivo:** [`play.hpp`](src/vss_core/include/vss_core/play.hpp)

Seleciona o Play ativo a cada frame por ordem de prioridade.

```
Prioridade | Play                  | Situação de jogo
-----------|-----------------------|-----------------
    100    | HaltPlay              | HALT / TIMEOUT
     90    | PenaltyAttackPlay     | penalty_ally
     90    | PenaltyDefensePlay    | penalty_enemy
     80    | KickoffAllyPlay       | kickoff_ally
     80    | KickoffEnemyPlay      | kickoff_enemy
     70    | GoalKickAllyPlay      | goalkick_ally
     70    | GoalKickEnemyPlay     | goalkick_enemy
     60    | FreeBallPlay          | freeball_ally/enemy
     50    | DirectKickPlay        | direct_kick_ally/enemy
     50    | IndirectKickPlay      | indirect_kick_ally/enemy
     10    | NormalDefensivePlay   | jogo normal, bola no nosso campo
     10    | OffensivePlay         | jogo normal, bola no campo adversário
      0    | NormalGamePlayVSS     | fallback de jogo normal
```

---

## Skills Implementadas

### `GoToBallSkill`
Persegue a bola usando APF (Campo Potencial Artificial).
- Usa `applyAPF()` para desviar de robôs e paredes no caminho
- Termina quando `dist_to_ball < arrival_threshold` (padrão: 5 cm)

### `GoToPositionSkill`
Vai para uma posição fixa no campo.
- Usado por goleiro (centro do gol) e defensor (posição calculada pela tática)
- Usa `applyLinearRamp()` para suavizar a aceleração

### `KickSkill`
Máquina de estados de chute com 4 estados:

```
ALIGN → CHARGE → UNSTICK → DONE
```

- **ALIGN:** Gira até apontar para o gol (threshold: 17°)
- **CHARGE:** Avança com `v=kick_speed`. Sistema Tridente verifica bloqueio frontal
- **UNSTICK:** Recuo de 2 fases ao detectar travamento (60 frames com `|v| > 1,5 m/s` e deslocamento `< 3 cm`)
- **DONE:** Chute executado, `isFinished() = true`

### `VectorAttackerSkill`
Campo vetorial contínuo para perseguição de bola. Alternativa ao `GoToBallSkill` para o atacante.
- Predição de posição da bola (`t = 150ms`)
- Spiral vector field: rotação ao redor da bola para posicionamento atrás dela
- Sistema UNSTICK idêntico ao `KickSkill`

### `OrbitAttackerSkill`
Atacante avançado com máquina de estados de 4 estados — ver seção dedicada abaixo.

---

## Táticas e Roles

### Táticas

| Tática | Skills encadeadas | Uso |
|--------|-------------------|-----|
| `DefendGoalTactic` | `GoToPositionSkill` (loop) | Goleiro oscila na linha do gol |
| `DefendAreaTactic` | `GoToPositionSkill` (loop) | Defensor na área defensiva |
| `VectorAttackTactic` | `VectorAttackerSkill` (loop) | Atacante normal |
| `AimGoalTactic` | `OrbitAttackerSkill` (loop) | Atacante que orbita antes de chutar |
| `SupportPositionTactic` | `GoToPositionSkill` (loop) | Robô de suporte/cobertura |
| `GoToBallAndKickTactic` | `GoToBallSkill` → `KickSkill` | Atacante de set-piece |

### Roles

| Role | Tática | Persistente? |
|------|--------|--------------|
| `KeeperRoleVSS` | `DefendGoalTactic` | ✅ |
| `DefenderAreaRoleVSS` | `DefendAreaTactic` | ✅ |
| `NormalAttackerRoleVSS` | `VectorAttackTactic` | ✅ |
| `BallChallengerAttackerRoleVSS` | `VectorAttackTactic` | ✅ |
| `AttackerAimGoalRoleVSS` | `AimGoalTactic` | ✅ |
| `SupportAttackerRoleVSS` | `SupportPositionTactic` | ✅ |
| `GoAndKickRoleVSS` | `GoToBallAndKickTactic` | ❌ (reinicia por design) |
| `GoalKickKeeperRoleVSS` | `GoToBallAndKickTactic` | ❌ (reinicia por design) |

> **Por que alguns Roles reiniciam intencionalmente?**
> `GoAndKickRoleVSS` e `GoalKickKeeperRoleVSS` são usados em set-pieces com começo e fim
> definidos. Devem reiniciar a sequência GoToBall→Kick quando o Play os ativa novamente.

---

## OrbitAttackerSkill — Máquina de Estados do Atacante

O atacante principal (`AttackerAimGoalRoleVSS`) usa a `OrbitAttackerSkill`, que implementa
a estratégia de orbitar ao redor da bola antes de empurrar.

```
         ┌──────────────────────────────────────────┐
         │                                          │
         ▼                                          │ ang_err > 20° (LEAVE)
    ┌─────────┐   ang_err < 15° (ENTER)   ┌────────┴──┐
    │  ORBIT  │ ────────────────────────► │  APPROACH │
    │         │ ◄────────────────────────  │           │
    └─────────┘                            └─────┬─────┘
         ▲                                       │ dist_tgt < 6cm
         │   ang_diff > 25°                      ▼
         │ (PUSH_REALIGN)               ┌─────────────┐
         └──────────────────────────────│     PUSH    │
                                        │  v=2.5 m/s  │
                                        └─────────────┘
                      stuck_frames > 60 →  UNSTICK (qualquer estado)
```

### Estado ORBIT
- Calcula `target_behind_ball` (12 cm atrás da bola, no vetor gol→bola)
- Campo vetorial com offset lateral para contornar a bola sem colidir
- Velocidade máxima: `1,8 m/s × align_factor` (reduzida se desalinhado)
- Transita para APPROACH quando `|ang_err| < 15°` e robô não está entre bola e gol

### Estado APPROACH
- Vai em linha reta para `target_behind_ball`
- Velocidade máxima: `2,0 m/s × align_factor`
- Transita para PUSH quando `dist_tgt < 6 cm`
- Volta para ORBIT se `|ang_err| > 20°`

### Estado PUSH
- Avança com `v = 2,5 m/s` em direção ao **centro do gol** (não à bola)
- `is_fast_slew = true`: bypassa o limitador externo do StrategyNode
- Volta para ORBIT se `|ang_diff_para_gol| > 25°`

### Estado UNSTICK
- Ativado após `stuck_frames > 60` (robô parado > 1s com `|v| > 1,5 m/s`)
- **FASE 1** (frames 1–25): `v = -3,0 m/s`, `ω = 0` — recuo bruto
- **FASE 2** (frames 26–45): `v = -1,0 m/s`, `ω = ±12` — giro de escape
- Retorna ao ORBIT após 45 frames

### Sistema Tridente (isFrontBlocked)
Três probes verificam bloqueio à frente do robô:

```
Probe Esquerda          Probe Centro          Probe Direita
(quina -3,75cm,         (eixo θ + 8,75cm)     (quina +3,75cm,
 6cm à frente)                                 6cm à frente)
     ●                       ●                       ●
     |                       |                       |
┌────┼───────────────────────┼───────────────────────┼────┐
│    ↑ -3.75cm            centro             +3.75cm  │
│              FACE FRONTAL DO CHASSI (7.5cm)          │
└──────────────────────────────────────────────────────┘
```

Quando FRONT_BLOCKED é verdadeiro, o robô **ainda transita de estado** (desde a correção C8),
mas emite `v = 0` neste frame. O próximo frame já usa o novo estado.

---

## Pipeline de Controle de Velocidade

Um comando de roda passa por **3 estágios de limitação** antes de chegar ao simulador:

```
Skill::execute()
  │
  ├─ applyLinearRamp(target_v, last_v_, accel)   [interno à skill]
  │   Limita Δv = accel × dt por frame
  │   Acelerações típicas: ORBIT=4,0 | APPROACH=5,0 | PUSH=6,0 m/s²
  │
  ├─ clampCommand(cmd)                           [interno à skill]
  │   Limita rodas a ±2,5 m/s
  │
  └─ RobotCommand.is_fast_slew (flag)
      │
StrategyNode::controlLoop()
  │
  └─ Slew Rate Limiter Externo                   [strategy_node.cpp:137]
      is_fast_slew == false → Δv_max = 0,15 m/s/frame (~9 m/s²)
      is_fast_slew == true  → Δv_max = 0,45 m/s/frame (~27 m/s²)
```

**Quando usar `is_fast_slew = true`:**
- Fase PUSH do `OrbitAttackerSkill` (necessidade de aceleração rápida)
- Fases UNSTICK do `KickSkill` e `VectorAttackerSkill` (impulso de escape)

---

## Tipos de Dados Fundamentais

**Arquivo:** [`types.hpp`](src/vss_core/include/vss_core/types.hpp)

```cpp
struct RobotState {
    uint8_t id;
    double x, y;       // posição (metros)
    double theta;      // orientação (radianos, [-π, π])
    double vx, vy;     // velocidade (m/s)
    double omega;      // velocidade angular (rad/s)
    bool valid;
};

struct BallState {
    double x, y;       // posição (metros)
    double vx, vy;     // velocidade (m/s)
    bool valid;
};

struct RobotCommand {
    uint8_t id;
    double wheel_left, wheel_right;  // velocidade de roda (m/s)
    bool is_fast_slew;               // false = limitado; true = impulso rápido

    static RobotCommand fromVW(uint8_t id, double v, double omega,
                               double wheel_base = 0.075, bool is_fast = false);
};
```

**Namespace `geo` — constantes físicas:**

```cpp
namespace geo {
    constexpr double ROBOT_SIDE        = 0.075;   // 7,5 cm
    constexpr double ROBOT_RADIUS      = 0.0375;  // 3,75 cm (centro→face)
    constexpr double ROBOT_CORNER_DIST = 0.05303; // √2/2 × 7,5cm (centro→quina)
    constexpr double BALL_RADIUS       = 0.0215;  // 2,15 cm
    constexpr double BEHIND_BALL_DIST  = 0.12;    // 12 cm (target_behind_ball)
    constexpr double WALL_MARGIN       = 0.05;    // 5 cm
}
```

**Campo VSS 3v3:**

```
     -0.75m                        +0.75m
       ├──────────────────────────────┤   +0.65m
       │           CAMPO              │
       │   (origem = centro)          │
       │                              │
       ├──────────────────────────────┤   -0.65m
```

---

## Como Adicionar um Novo Comportamento

### Nova Skill

```cpp
// em: vss_core/include/vss_core/minha_skill.hpp
class MinhaSkill final : public Skill {
public:
    void init(const RobotState& robot, const GameContext& ctx) override {
        Skill::init(robot, ctx);
        // inicializa estado interno
        last_v_ = 0.0;
    }

    RobotCommand execute(const RobotState& robot, const GameContext& ctx) override {
        // calcula v e omega
        double v = 1.5;
        double omega = 0.0;

        v = avoidCollisions(robot, ctx, v);        // freia perto de obstáculos
        v = applyLinearRamp(v, last_v_, 4.0);      // suaviza aceleração
        last_v_ = v;

        if (chegou_ao_alvo) finished_ = true;      // sinaliza conclusão

        return clampCommand(RobotCommand::fromVW(robot.id, v, omega, 0.075));
    }

    std::string name() const override { return "MinhaSkill"; }

private:
    double last_v_ = 0.0;
};
```

### Nova Tática

```cpp
class MinhaTatica final : public Tactic {
public:
    MinhaTatica() {
        addSkill<GoToBallSkill>();   // skill 1
        addSkill<MinhaSkill>();      // skill 2 (ativa quando skill 1 terminar)
    }
    bool isFinished() const override { return false; }  // ou true se sequencial
    std::string name() const override { return "MinhaTatica"; }
};
```

### Novo Role

```cpp
class MeuRoleVSS final : public Role {
public:
    explicit MeuRoleVSS(uint8_t id)
        : Role(id)
        , tactic_(std::make_shared<MinhaTatica>())
    {}
    std::string name() const override { return "MeuRoleVSS"; }

protected:
    std::shared_ptr<Tactic> selectTactic(const RobotState&, const GameContext&) override {
        return tactic_;
    }
private:
    std::shared_ptr<MinhaTatica> tactic_;
};
```

### Novo Play

```cpp
class MeuPlay final : public Play {
public:
    bool isApplicable(const GameContext& ctx) const override {
        return ctx.situation == GameSituation::NORMAL_GAME && minha_condicao(ctx);
    }

    void init(const GameContext& ctx) override {
        assignRoles(
            std::make_shared<NormalAttackerRoleVSS>(closestToBall(ctx)),
            std::make_shared<DefenderAreaRoleVSS>(middleRobot(ctx)),
            std::make_shared<KeeperRoleVSS>(farthestFromBall(ctx)),
            ctx
        );
    }

    std::string name() const override { return "MeuPlay"; }
    int priority() const override { return 5; }
};
```

---

## Registro de Bugs Corrigidos

### C3/C4 — Reinstanciação de Tactic a cada frame
**Problema:** Roles como `KeeperRoleVSS` criavam `std::make_shared<Tactic>()` dentro de
`selectTactic()`, que é chamado 60×/s. Isso destruía a Tactic (e a Skill interna) todo frame,
zerando `last_v_`, `stuck_frames_` e todos os contadores internos. O robô nunca acelerava.

**Correção:** `tactic_` virou membro persistente, instanciada uma vez no construtor.

**Roles corrigidos:** `KeeperRoleVSS`, `DefenderAreaRoleVSS`, `NormalAttackerRoleVSS`, `BallChallengerAttackerRoleVSS`

---

### C6 — UNSTICK sem `is_fast_slew`
**Problema:** As fases UNSTICK de `KickSkill` e `VectorAttackerSkill` emitiam `v = -3,0 m/s`
sem o flag `is_fast_slew = true`. O slew externo do StrategyNode limitava o impulso real a
`~-1,75 m/s` (em 25 frames com Δv=0,15/frame), insuficiente para desencaixar o robô.

**Correção:** Todas as fases UNSTICK agora passam `is_fast_slew = true` como 5º argumento de `fromVW()`.

---

### C7-CMake — `vss_core` sem export de includes
**Problema:** `vss_fira_bridge` e outros pacotes falhavam ao fazer `find_package(vss_core)` porque o
`CMakeLists.txt` do `vss_core` não chamava `ament_export_include_directories(include)`.

**Correção:** Adicionado `ament_export_include_directories(include)` em `vss_core/CMakeLists.txt`.

---

### C8 — FRONT_BLOCKED impedindo transições de estado permanentemente
**Problema:** Em `doOrbit()` e `doApproach()` da `OrbitAttackerSkill`, o bloco `FRONT_BLOCKED`
continha um `early return` **antes** das verificações de transição de estado. Com o robô na
posição correta (`dist_tgt=3cm, ang_err=-0.008rad`), a parede disparava `FRONT_BLOCKED` e
as transições `ORBIT→APPROACH` e `APPROACH→PUSH` **nunca eram avaliadas**.

Efeito secundário: `last_v_ = 0.0` dentro do `FRONT_BLOCKED` resetava `stuck_frames_` continuamente,
impedindo também que o UNSTICK disparasse.

**Correção:** As verificações de transição de estado foram movidas para **antes** do bloco
`FRONT_BLOCKED`. O FRONT_BLOCKED ainda controla a velocidade do frame atual (`v=0`), mas o
`state_` é atualizado primeiro. No próximo frame, `execute()` já chama o estado correto.

---

*Documentação gerada em 14/06/2026 — commit de referência: correção C8 (FRONT_BLOCKED ordering)*
