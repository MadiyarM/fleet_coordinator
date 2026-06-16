# fleet_coordinator

A ROS 2 package for multi-robot fleet coordination in NVIDIA Isaac Sim 5.1.

Three Nova Carter robots navigate a hospital environment via Nav2. A custom coordinator node manages inter-robot collision avoidance using hysteresis-based blocking — no hardcoded velocity commands, coordination goes entirely through the Nav2 action interface.

![Demo](demo.gif)

---

## How it works

Each robot's position is read from `/carterN/amcl_pose` (map frame). The coordinator computes pairwise distances at 5 Hz and applies hysteresis:

- **Block** at 3m → lower-priority robot receives `CancelGoal`
- **Release** at 5m → robot receives `NavigateToPose` to resume

Priority is static: `carter1 > carter2 > carter3`. An already-blocked robot cannot block others.

```
carter1 ──────────────────────────────► goal
carter2 ──────────► BLOCKED ──────────► goal  (carter1 within 3m)
carter3 ──────────────────────────────► goal
                        ↑
              released when dist > 5m
```

---

## Stack

| Component | Version |
|---|---|
| NVIDIA Isaac Sim | 5.1 |
| ROS 2 | Humble |
| Nav2 | Humble |
| Python | 3.10 |
| Ubuntu | 22.04 |

---

## Prerequisites

- NVIDIA Isaac Sim 5.1 with `isaacsim.ros2.bridge` enabled
- Isaac ROS Nova Carter navigation workspace (`carter_ws`)
- Scene: `multiple_robot_carter_hospital_navigation.usd`

---

## Usage

**Terminal 1 — Isaac Sim:**
```bash
~/run_isaacsim.sh
# Script Editor → load scene → Play ▶
```

**Terminal 2 — Nav2:**
```bash
source /opt/ros/humble/setup.bash
source ~/carter_ws/install/setup.bash
ros2 launch carter_navigation multiple_robot_carter_navigation_hospital.launch.py
```

In RViz: set **2D Pose Estimate** for each robot, then wait for AMCL to publish poses.

**Terminal 3 — Fleet Coordinator:**
```bash
source /opt/ros/humble/setup.bash
source ~/carter_ws/install/setup.bash
ros2 launch fleet_coordinator fleet_coordinator.launch.py
```

---

## Parameters

Edit constants at the top of `fleet_coordinator_node.py`:

| Constant | Default | Description |
|---|---|---|
| `BLOCK_DIST` | 3.0 m | Distance at which lower-priority robot is blocked |
| `RELEASE_DIST` | 5.0 m | Distance at which blocked robot is released |
| `GOALS` | see file | Waypoints per robot in map frame |
| `ROBOT_NAMES` | carter1/2/3 | Priority order (index 0 = highest) |

---

## Architecture

```
/carter1/amcl_pose ──┐
/carter2/amcl_pose ──┤  fleet_coordinator_node
/carter3/amcl_pose ──┘         │
                        ┌──────┴──────┐
                        ↓             ↓
               /carter2/navigate_to_pose   (CancelGoal / NavigateToPose)
               /carter3/navigate_to_pose
```

The coordinator never writes to `cmd_vel`. All motion control goes through the Nav2 action server — robots stop cleanly and resume with a fresh goal rather than fighting the planner.

---

## Repo structure

```
fleet_coordinator/
├── fleet_coordinator/
│   └── fleet_coordinator_node.py   # coordinator logic
├── launch/
│   └── fleet_coordinator.launch.py
├── package.xml
├── setup.py
└── README.md
```

---

## Background

Migrated from Stage simulator (geometric models, no physics) to Isaac Sim 5.1 (PhysX, RTX rendering, realistic sensors). The hospital scene and Nova Carter robots use NVIDIA's official USD assets with namespaced OmniGraphs per robot, the coordinator is custom ROS 2 code written on top.
