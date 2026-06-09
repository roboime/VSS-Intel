#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  vss_core — include único para toda a lógica do jogo VSS
//
//  Use em qualquer nó ROS 2:
//    #include "vss_core/vss_core.hpp"
//
//  Ordem de include respeita dependências:
//    types → game_context → skill → tactic → role → play
//    skills concretas → roles → plays → playbook
// ─────────────────────────────────────────────────────────────────────────────

// ── 1. Estruturas de dados base ───────────────────────────────────────────────
#include "vss_core/types.hpp"           // RobotState, BallState, RobotCommand,
                                        // FieldParams, GameSituation, Point2D

// ── 2. Contexto do jogo (= cluster Game do LabVIEW) ──────────────────────────
#include "vss_core/game_context.hpp"    // GameContext — passado para tudo

// ── 3. Interfaces base da hierarquia ─────────────────────────────────────────
#include "vss_core/skill.hpp"           // Skill (interface) + HaltSkill
#include "vss_core/tactic.hpp"          // Tactic + SingleSkillTactic + HaltTactic
#include "vss_core/role.hpp"            // Role (interface) + HaltRole
#include "vss_core/play.hpp"            // Play (interface) + HaltPlay + Playbook

// ── 4. Skills concretas (traduzidas do LabVIEW) ───────────────────────────────
#include "vss_core/go_to_ball_skill.hpp"      // GoToBallSkill
                                               //   ← GoToBallSkillVSS.vi
#include "vss_core/go_to_position_skill.hpp"  // GoToPositionSkill
                                               //   ← posicionamento defensor/goleiro
#include "vss_core/kick_skill.hpp"            // KickSkill + KickPenaltySkill
                                               //   ← KickSkillVSS.vi
#include "vss_core/wait_ball_kicker_skill.hpp"// WaitBallKickerSkill
                                               //   ← WaitBallKickerSkill_class
#include "vss_core/rotate_to_angle_skill.hpp" // RotateToAngleSkill
                                               //   ← "Rotate CW / CCW" do KickSkillVSS

// ── 5. Roles e Táticas internas ───────────────────────────────────────────────
#include "vss_core/roles.hpp"   // Táticas: DefendGoalTactic, DefendAreaTactic,
                                 //          GoToBallAndKickTactic, AimGoalTactic
                                 // Roles:   KeeperRoleVSS, DefenderAreaRoleVSS,
                                 //          NormalAttackerRoleVSS,
                                 //          AttackerAimGoalRoleVSS,
                                 //          BallChallengerAttackerRoleVSS,
                                 //          GoAndKickRoleVSS,
                                 //          GoalKickKeeperRoleVSS

// ── 6. Plays e Playbook ───────────────────────────────────────────────────────
#include "vss_core/plays.hpp"   // NormalGamePlayVSS, NormalDefensivePlay,
                                 // OffensivePlay, KickoffAllyPlay,
                                 // KickoffEnemyPlay, PenaltyAttackPlay,
                                 // PenaltyDefensePlay, FreeBallPlay,
                                 // GoalKickAllyPlay, GoalKickEnemyPlay,
                                 // DirectKickPlay, IndirectKickPlay,
                                 // VSSPlaybook  ← use este no strategy_node
