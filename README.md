# Creation of a local path planning algorithm for a drone
This project aims to implement a 2-D local planning algorithm for a drone. Current candidates include a modified version of the classic Artificial Potential Field algorithm.
## Team members
- Kenechukwu Jesse Chukwuorji
- Rinka Izumi
- Dinesh Elumalai

# Drone Navigation

This repository contains implementations of:

- **Improved Artificial Potential Field (IAPF)** navigation
- **Wall Following + IAPF Hybrid** navigation

---

# How to Launch

## Improved APF (Grid of Cylinders World)

### 1. Launch the simulation with an empty world

```bash
ros2 launch px4_drone_sim sim.launch.py
```

### 2. Generate the obstacles

> **Note:** You must specify the path to the `model.sdf` file.

```bash
ros2 launch drone_navigation custom_world.launch.py
```

### 3. Run the obstacle detection node

```bash
ros2 run obstacles obstacle_detector
```

### 4. Run the Improved APF node

The optimal parameters depend on:

- Obstacle dimensions
- Initial distance to the goal
- Environment configuration

Example:

```bash
ros2 run drone_navigation apf_improved --ros-args \
-p goal_x:=15.0 \
-p goal_y:=0.0 \
-p K_att:=2.0 \
-p K_rep:=15.0 \
-p wp_dist:=0.5 \
-p threat_dist:=3.0
```

---

# Wall Following + IAPF Hybrid

### 1. Modify `sim.launch.py`

Set the following parameter to **true**:

```text
always_send_full_costmap = true
```

This simplifies obstacle detection by always using the full costmap instead of merging `costmap_update` data with the existing costmap.

---

### 2. Run the Wall Following node

```bash
ros2 run wall_follower_pkg wall_follower_node
```

### 3. Run the obstacle detection node

```bash
ros2 run obstacles obstacle_detector
```

### 4. Run the Hybrid APF node

```bash
ros2 run drone_navigation apf_wall_following --ros-args \
-p goal_x:=5.0 \
-p goal_y:=80.0 \
-p K_att:=2.0 \
-p K_rep:=15.0 \
-p wp_dist:=0.5 \
-p threat_dist:=3.0
```

---

# Possible Future Updates

- Create launch files to simplify launching multiple nodes.
- Include the custom world `model.sdf` file.
- Include the modified `sim.launch.py` file.

---

# Important Notes

> **Improved APF**
>
> The modified APF works reliably provided the parameters are well tuned and the obstacles are relatively small compared to the drone. It is **not** suitable for navigating around very large obstacles such as long walls or large buildings.

> **Wall Following + IAPF Hybrid**
>
> The current implementation is a proof of concept. It is still rudimentary and requires significant additional work to improve its reliability and robustness.

