# Markovito Pick-and-Place State Machine (TMR 2026)

ROS-based task coordinator and state machine developed for **Markovito**, our service robot for the **TMR 2026 (Torneo Mexicano de Robótica)** competition, where our team achieved **2nd place**.

This repository contains the state-machine implementation for the **pick-and-place task**, designed for autonomous object sorting and manipulation in domestic service environments.

## Overview

This system coordinates:

- Object detection using perception modules (SAM-based detection)
- Point cloud centroid extraction
- Offset-based grasp/place target generation
- Motion planning using MoveIt
- Autonomous manipulation through a finite state machine
- Category-based object sorting:
  - Trash → Trash bin
  - Dishes → Dishwasher
  - Groceries → Shelf placement (partially implemented)

## Task Pipeline

### Trash Objects
1. Detect object on table
2. Classify as trash
3. Navigate to trash can
4. Detect trash can with SAM + point cloud
5. Compute centroid and apply placement offset
6. Plan trajectory with MoveIt
7. Move manipulator and release object

## Dish Objects
Same pipeline, but objects are delivered into the dishwasher.

## Shelf Placement (Work in Progress)
Includes:
- Shelf inspection
- Category-level assignment
- Empty-space reasoning
- Planned autonomous shelf placement

This branch was partially implemented but not fully tested during competition due to time constraints.

---

## State Machine Features

Implemented using:
- ROS
- Python
- transitions state machine library
- MoveIt
- Point cloud perception

Capabilities include:

- Reactive event-driven transitions
- Failure recovery
- Re-detection loops
- Base adjustment for grasping
- Cartesian placement using centroid offsets
- Category-dependent task branching

---

## Repository Contents

```bash
State-Machine_pick_and_place.py
```

Main ROS coordinator for pick-and-place task execution.

---

## Requirements

System:
- Ubuntu 20.04
- ROS Noetic
- MoveIt
- Python 3.8+

Python dependencies:

```bash
pip install -r requirements.txt
```

---

## requirements.txt

```txt
transitions>=0.9.0
numpy>=1.24
```

(ROS packages are installed through ROS and are not included in pip requirements.)

---

## Run

```bash
rosrun <your_package> State-Machine_pick_and_place.py
```

---

## Demo

Videos and experimental results available in this repository (add links or media here).

You can include:


## Trash Disposal Demo

[Watch Trash Demo](images/trash_demo1.mp4)


- Competition runs
- Object sorting demos
- Trash disposal
- Dishwasher placement
- State machine diagrams

---

## Competition

Developed for:

Torneo Mexicano de Robótica 2026 (TMR)
Service Robotics / Domestic Manipulation
Team Markovito — 2nd Place

---

## Future Work

- Complete and validate shelf-placement branch
- Extend category reasoning
- Integrate LLM-driven task commands
- Improve grasp robustness

---

## Citation

If you use this repository in research, please cite or reference this project.
