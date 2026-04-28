# Markovito Pick-and-Place State Machine (TMR 2026)

ROS-based task coordinator and finite state machine developed for **Markovito**, our service robot for the **Torneo Mexicano de Robótica 2026 (TMR 2026)**, where our team achieved **2nd Place**.

This repository contains the implementation of our **autonomous pick-and-place task**, designed for domestic service robotics applications involving object detection, categorization, manipulation, and category-based sorting.

---

## Competition Demonstrations

### Object Sorting and Manipulation

<p align="center">
  <img src="images/trash_demo1.gif" width="700">
</p>

<p align="center">
  <img src="images/trash_demo2.gif" width="700">
</p>

<p align="center">
  <img src="images/trash_demo3.gif" width="700">
</p>

---

### Dishwasher Placement

<p align="center">
  <img src="images/trash_demo4.gif" width="700">
</p>

<p align="center">
  <img src="images/trash_demo5.gif" width="700">
</p>

---

### Additional Task Behaviors

<p align="center">
  <img src="images/trash_demo6.gif" width="700">
</p>

---

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

---

## Task Pipeline

## Trash Objects

1. Detect object on table  
2. Classify object as trash  
3. Navigate to trash can  
4. Detect trash can using SAM + point cloud  
5. Compute centroid and apply placement offset  
6. Plan manipulation trajectory with MoveIt  
7. Move manipulator and release object  

---

## Dish Objects

Same pipeline as above, but dish-category objects are placed into the dishwasher.

---

## Shelf Placement (Work in Progress)

Includes:

- Shelf inspection
- Category-level assignment
- Empty-space reasoning
- Planned autonomous shelf placement

This branch was partially implemented and reserved for future validation.

---

## State Machine Features

Implemented using:

- ROS
- Python
- MoveIt
- Point cloud perception
- `transitions` state machine library

Capabilities include:

- Reactive event-driven transitions  
- Failure recovery behaviors  
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

System Requirements

- Ubuntu 20.04  
- ROS Noetic  
- MoveIt  
- Python 3.8+  

Install Python dependencies:

```bash
pip install -r requirements.txt
```

### requirements.txt

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

## System Highlights

This implementation integrates:

- Perception-guided manipulation  
- Point-cloud centroid reasoning  
- Offset-based placement strategies  
- MoveIt trajectory planning  
- Category-aware autonomous sorting  
- State-machine task orchestration  

---

## Competition

Developed for:

**Torneo Mexicano de Robótica 2026 (TMR)**  
Service Robotics / Domestic Manipulation  
**Team Markovito — 2nd Place**

---

## Future Work

- Complete and validate shelf-placement branch  
- Extend category reasoning  
- Improve grasp robustness  
- Integrate higher-level task planning with LLM-based commands  
- Expand manipulation behaviors for additional service tasks  

---

## Citation

If you use this repository in research, please cite or reference this project.

---

## Authors

Developed by Team Markovito for TMR 2026.
