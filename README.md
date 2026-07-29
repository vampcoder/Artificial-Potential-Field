# Artificial Potential Field Motion Planning

Implementation of the **Artificial Potential Field (APF)** reactive method for mobile robot motion planning and obstacle avoidance.

---

## Concept Overview

In Artificial Potential Field planning:
- The **Robot** is modeled as a positively charged particle.
- The **Goal Position** is modeled as an attractive (negatively charged) field.
- **Obstacles** are modeled as repulsive (positively charged) potential fields.

The net potential force vector `F_total = F_att + F_rep` drives the robot towards the target while steering away from surrounding obstacles.

---

## Force Formulations & Variable Definitions

### 1. Attractive Force Formulation

- `F_att(q) = -tau * (q - q_goal)` if `d(q, q_goal) <= d*`
- `F_att(q) = -d* * tau * (q - q_goal) / d(q, q_goal)` if `d(q, q_goal) > d*`

### 2. Repulsive Force Formulation

- `F_rep(q) = eta * (1/D(q) - 1/Q*) * (1/D(q))^2 * d'(q)` if `D(q) < Q*`
- `F_rep(q) = 0` if `D(q) >= Q*`

### 3. Variable Dictionary

| Variable | Description |
|---|---|
| `q` | Current 2D position vector of the robot $(x, y)$. |
| `q_goal` | Destination target coordinates $(x_{\text{goal}}, y_{\text{goal}})$. |
| `d(q, q_goal)` | Euclidean distance between robot position `q` and target `q_goal`. |
| `d*` | Distance threshold switching between parabolic and conic attractive potential fields. |
| `tau` ($\tau$) | Gain/scaling constant determining attractive force strength. |
| `D(q)` | Shortest distance from robot position `q` to the nearest obstacle boundary. |
| `Q*` | Maximum influence radius of obstacle repulsive fields (obstacles beyond $Q^*$ exert 0 force). |
| `eta` ($\eta$) | Gain/scaling constant determining repulsive force strength. |
| `d'(q)` / $\nabla D(q)$ | Gradient vector pointing directly away from the nearest obstacle boundary. |

---

## Sample Visualizations

| Sample Input Frame | Potential Field Planned Output |
|---|---|
| ![Sample Input 1](1.jpg) | ![Planned Output 1](output/1.jpg) |
| ![Sample Input 2](2.jpg) | ![Planned Output 2](output/2.jpg) |

---

## Prerequisites & Setup

Clone the repository and install dependencies:

```bash
git clone https://github.com/vampcoder/Artificial-Potential-Field.git
cd Artificial-Potential-Field
pip install -r requirements.txt
```

---

## Execution

Run the potential field motion planning simulations:

```bash
# Main Artificial Potential Field Simulation
python3 Artificial-Potential-final.py

# Potential Field Controller Simulation
python3 Artificial-potential-controller.py
```

---

## License

MIT License
