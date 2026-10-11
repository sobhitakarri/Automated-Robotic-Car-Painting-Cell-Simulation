# Robotic Car Painting Cell Simulation

**YUKTII AI LABS – Robotics Simulation Intern Practical Assessment**

* **Candidate Name:** K S V S Sobhita
* **Simulation Software:** CoppeliaSim (V-REP) / Python 3.x
* **Robot Selected:** 6-DOF UR10 Articulated Industrial Manipulator
* **Target Role:** Robotics Simulation Intern – Yuktii AI Labs

---

## 📌 Project Overview

This repository contains the design, kinematic modeling, multi-surface trajectory planning algorithms, collision checking, and 3D simulation for an automated **Robotic Car Painting Cell**.

The project demonstrates an industrial 6-DOF UR10 articulated robot applying a uniform automotive spray coating across exterior vehicle panels of a full-scale passenger sedan ($4,800\,\text{mm} \times 1,900\,\text{mm} \times 1,600\,\text{mm}$) within a $12\,\text{m} \times 8\,\text{m}$ cleanroom spray booth.

---

## 🚗 Reference Car & Cell Specifications

| Parameter | Engineering Specification |
| :--- | :--- |
| **Car Dimensions** | Length: 4,800 mm \| Width: 1,900 mm \| Height: 1,600 mm |
| **Spray Booth Cell** | 12,000 mm $\times$ 8,000 mm cleanroom enclosure with downdraft ventilation |
| **Robot Manipulator** | 6-DOF UR10 Serial Articulated Robot |
| **End-Effector Tool** | Rotary Bell Spray Atomizer (Nominal Standoff: 250 mm) |
| **Surfaces Coated** | Hood, Roof, Left Side Panels & Doors, Trunk Deck |
| **Safety System** | Perimeter safety barriers, light curtains, emergency stop envelope |

---

## 📁 Repository Structure

```
├── Simulation_Files/
│   ├── final.ttt                        # Complete CoppeliaSim 3D simulation scene
│   ├── trajectory_generator.py          # Multi-surface raster toolpath planner (Python)
│   └── kinematics_and_collision.py      # 6-DOF D-H kinematics & AABB collision engine (Python)
│
├── Report/
│   ├── Technical_Report.docx            # Editable Word report with embedded screenshots
│   └── Technical_Report.pdf             # Exported final technical report (3-5 pages)
│
├── Screenshots/                         # High-res simulation viewport captures
├── Demo_Video/                          # 3-7 minute simulation walkthrough video
└── README.md                            # Main project documentation
```

---

## 🚀 How to Run the Simulation & Scripts

### 1. CoppeliaSim 3D Simulation
1. Launch **CoppeliaSim**.
2. Open the scene file: [`Simulation_Files/final.ttt`](Simulation_Files/final.ttt).
3. Click the **Play button (▶)** in the top toolbar.
4. The 6-DOF UR10 robot will autonomously execute the multi-surface spray coating sequence across the hood, roof, side panels, and trunk with real-time spray particle visualization.

### 2. Standalone Python Planning & Kinematics Algorithms
Both Python modules are completely self-contained and run directly in any standard Python 3 environment:

* **Generate Multi-Surface Painting Trajectories**:
  ```bash
  python Simulation_Files/trajectory_generator.py
  ```
  *Outputs waypoint coordinates, step resolutions, surface normal vectors, and standoff offsets across all vehicle panels.*

* **Run Kinematic Verification & Collision Checking**:
  ```bash
  python Simulation_Files/kinematics_and_collision.py
  ```
  *Computes 6-DOF Denavit-Hartenberg (D-H) forward kinematics, tests joint mechanical limits, and performs dual-tier Axis-Aligned Bounding Box (AABB) intersection tests.*

---

## 🔬 Technical Highlights

1. **Multi-Surface Raster Planning**: Synthesizes continuous serpentine/meander toolpaths maintaining a strict $250\,\text{mm}$ standoff distance and surface-perpendicular orientation.
2. **Denavit-Hartenberg Kinematics**: Implements standard 6-DOF D-H forward kinematics to transform Tool Center Point (TCP) coordinates from joint space to world coordinates.
3. **Collision Safety Envelope**: Dual-tier collision verification inspecting robot link segments against bounding volumes (AABBs) for the vehicle body, cabin, and conveyor fixtures.
4. **Visual Spray Deposition**: Realistic spray particle atomization cone and dynamic surface coating color deposition.

---

## 📧 Submission Info
* **Author:** K S V S Sobhita
* **Target Role:** Robotics Simulation Intern – Yuktii AI Labs
* **Submission Email:** `connect@yuktiiai.in`
