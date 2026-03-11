# Analytical IK Solver vs KDL: Full Explanation

## The Big Picture: What is Inverse Kinematics?

Your robot arm has **joints** (angles) and a **tool tip** (position + orientation in space).

- **Forward Kinematics (FK)**: Given joint angles → where is the tool tip? This is easy — just multiply matrices.
- **Inverse Kinematics (IK)**: Given a desired tool tip pose → what joint angles get you there? This is hard.

It's like the difference between: "I moved my shoulder 30° and my elbow 45° — where's my hand?" (easy geometry) vs. "I want my hand *here* — what should my shoulder and elbow angles be?" (much harder).

---

## How KDL Solves IK (The Old Way)

KDL (Kinematics and Dynamics Library) uses **numerical/iterative IK**. Think of it like gradient descent:

1. Start with a **seed** — some initial guess for joint angles
2. Run FK to see where those angles put the tool tip
3. Compute the **error** between where you are and where you want to be
4. Compute the **Jacobian** — a matrix that says "if I nudge joint 1 by a tiny bit, how does the tool tip move?"
5. Use the Jacobian to figure out which direction to nudge the joints
6. Repeat steps 2-5 until the error is small enough (or you give up)

**Problems with KDL:**
- **Seed-dependent**: Different starting guesses → different solutions (or no solution). It's like hill-climbing — you find the nearest peak, not necessarily the best one.
- **One solution per call**: Each call returns at most 1 solution. For a UR16e, there are up to **8 valid configurations** that all reach the same pose — KDL only finds whichever one is closest to the seed.
- **Can fail to converge**: If the seed is bad or you're near a singularity, it might not find any solution even when one exists.
- **Slow**: Each call does ~100+ iterations of matrix math.
- **7-DOF problem**: MoveIt's `setFromIK()` tries to solve the full 7-DOF (gantry + 6 arm joints) at once. The gantry is prismatic (linear), and KDL treats it as just another joint to iterate on. It doesn't know that the gantry is special.

---

## How Our Analytical Solver Works (The New Way)

Instead of iterating, we use **closed-form equations** — direct formulas that give you the answer in one shot, like using the quadratic formula instead of Newton's method.

This is possible because the UR16e has a specific structure: it's a **6R robot with a spherical wrist** (joints 4, 5, 6 all intersect at a point). This structure allows us to decouple the problem.

### Step 0: DH Parameters

The robot's geometry is encoded in 6 numbers (Denavit-Hartenberg parameters):

```
d1 = 0.1807m    (height from base to shoulder)
a2 = -0.4784m   (upper arm length)
a3 = -0.36m     (forearm length)
d4 = 0.17415m   (shoulder-to-wrist offset in Y)
d5 = 0.11985m   (wrist offset)
d6 = 0.11655m   (tool flange length)
```

These define the link lengths and offsets. `a` values are link lengths (along X), `d` values are offsets (along Z).

### Step 1: Wrist Center (line 44 in ur16e_analytical_ik.hpp)

```cpp
Eigen::Vector3d p05 = p06 - d6 * R06.col(2);
```

The **trick** that makes analytical IK possible: separate position from orientation.

`p06` is where the tool tip is. `R06.col(2)` is the Z-axis direction of the tool. We subtract `d6` (tool length) along that Z-axis to get `p05` — the **wrist center point** (where joints 4, 5, 6 intersect).

Why? Because joints 4, 5, 6 only control **orientation** — they don't move this point. So we can solve for joints 1, 2, 3 using only the wrist center position, then solve 4, 5, 6 from the orientation separately. This is called **kinematic decoupling**.

### Step 2: Theta1 — Shoulder Pan (lines 47-58)

```cpp
double psi = std::atan2(p05.y(), p05.x());
double r_xy = std::hypot(p05.x(), p05.y());
double phi1 = std::acos(d4 / r_xy);
theta1 = psi + phi1 + π/2    // solution A
theta1 = psi - phi1 + π/2    // solution B
```

Looking down at the robot from above (XY plane), the wrist center is at some angle `psi` from the X-axis. But the shoulder has an offset `d4` — the arm doesn't swing directly through the center, it's offset to the side.

Think of it like a door hinge that's not at the edge of the door — there's a `d4` gap. The `acos(d4/r_xy)` accounts for this offset, and the `±` gives you **two solutions**: the "left-shoulder" and "right-shoulder" configurations (like reaching for something with your elbow pointing left vs. right).

The `+ π/2` is because of the UR's DH convention — joint 1's zero position is rotated 90° from what you'd expect.

**2 solutions so far.**

### Step 3: Theta5 — Wrist 2 (lines 64-68)

```cpp
double cos5 = (p06.x() * s1 - p06.y() * c1 - d4) / d6;
theta5 = +acos(cos5)    // wrist up
theta5 = -acos(cos5)    // wrist down
```

For each theta1, we solve for theta5 (the wrist bend). This uses the **orientation** of the target pose combined with the known theta1 to extract what cos(theta5) must be. The `±acos` gives **two solutions**: wrist pointing "up" vs "down" (like flipping your hand over).

**2 × 2 = 4 solutions so far.**

### Step 4: Theta6 — Wrist 3 (lines 74-81)

```cpp
double sin6 = (-R06(0,1) * s1 + R06(1,1) * c1) / s5;
double cos6 = ( R06(0,0) * s1 - R06(1,0) * c1) / s5;
q6 = atan2(sin6, cos6);
```

Given theta1 and theta5, theta6 is **uniquely determined** — no branching here. It comes directly from the rotation matrix elements. If theta5 ≈ 0 (singularity — wrist is straight), theta6 is arbitrary (set to 0).

**Still 4 solutions.**

### Step 5: Theta2 + Theta3 — Shoulder Lift + Elbow (lines 84-116)

This is the **2R planar arm problem** — the classic "given an upper arm and forearm of known lengths, reach a point in the plane."

```cpp
// Project wrist center into the arm's plane
double p1x = c1 * p05.x() + s1 * p05.y();   // horizontal distance in arm plane
double p1z = p05.z() - d1;                    // vertical distance

// Effective second link (accounts for d5 offset)
double Ap = hypot(a3, d5);          // effective forearm length: √(0.36² + 0.12²) = 0.3794m
double phip = atan2(d5, a3);        // angular offset from d5

// Law of cosines for elbow angle
double cos3p = (r² - a2² - Ap²) / (2 * a2 * Ap);
q3p = +acos(cos3p)    // elbow up
q3p = -acos(cos3p)    // elbow down
q3 = q3p - phip;      // correct for d5 offset
```

Imagine a 2D arm with an upper arm (`a2 = 0.4784m`) and forearm (`Ap = 0.3794m`) trying to reach a point. The law of cosines gives you the elbow angle, and `±acos` means **elbow up vs elbow down** (like reaching over a table vs under it).

The `d5` complication: the wrist isn't perfectly aligned with the forearm — it's offset by `d5 = 0.12m`. So the effective forearm length is `Ap = √(a3² + d5²)` and there's an angular correction `phip`.

Then theta2 comes from standard 2R geometry:
```cpp
q2 = atan2(p1z, p1x) - atan2(k2, k1);
```

**4 × 2 = 8 solutions total.**

### Step 6: Theta4 — Wrist 1 (lines 118-136)

```cpp
R03 = [rotation matrix built from q1, q2, q3]
R36 = R03ᵀ × R06     // what joints 4,5,6 must do
q4 = atan2(R36(1,2)/s5, R36(0,2)/s5);
```

We know R06 (the total rotation we want) and R03 (the rotation from joints 1-3 that we just solved). So `R36 = R03ᵀ × R06` tells us what the wrist joints need to do. Theta4 is uniquely determined from the R36 matrix elements.

**Final count: 8 solutions maximum.**

---

## How It's Used in the 7-DOF System (getJointConfigurations)

The UR16e arm is 6-DOF, but our system has a 7th joint — the **gantry** (linear actuator). This makes it **redundant**: there are infinitely many gantry positions that could work.

Our approach:

1. **Sweep the gantry** at 0.1m steps: current position ± 1.0m → ~17 discrete positions
2. At each gantry position:
   - Use MoveIt FK to get where the UR base actually is in the world: `getGlobalLinkTransform("base_link_inertia")`
   - Transform the target pose from world frame into the UR base frame: `target_in_dh0 = dh_frame0⁻¹ × target_in_world`
   - Run the analytical solver → up to 8 solutions
   - Filter by joint limits (`satisfiesBounds`)
   - Score each by cost (joint distance, manipulability, clearance, limit margin)
3. Keep one KDL fallback solve from the current state (belt and suspenders)
4. Sort all solutions by cost, return the ranked list
5. The planner (OMPL/RRT) tries them in order as goal configurations

---

## Side-by-Side Comparison

| | KDL (old) | Analytical (new) |
|---|---|---|
| **Method** | Iterative (Jacobian + Newton-Raphson) | Direct formulas (closed-form) |
| **Solutions per call** | 1 (seed-dependent) | Up to 8 (all of them) |
| **Speed** | ~100+ iterations of matrix ops | ~50 trig operations, one shot |
| **Completeness** | Misses solutions far from seed | Finds ALL solutions (exhaustive) |
| **Singularity handling** | Can fail to converge | Detects and handles gracefully (s5≈0) |
| **7-DOF handling** | Iterates on gantry as another joint | Explicit gantry sweep + 6R solve |
| **Total candidates** | 1 (maybe a few with random seeds) | Up to ~136 (17 gantry × 8 per) |
| **Dependency** | Needs KDL library | Pure math, header-only, no dependencies |

The fundamental win: instead of asking "find *a* solution" you're asking "find *all* solutions and pick the best one." Better goal configurations → easier planning → higher success rate.

---

## Key Files

- **Analytical IK solver**: `src/arpa_control/include/arpa_control/ur16e_analytical_ik.hpp`
- **Integration (getJointConfigurations)**: `src/arpa_control/src/motion_control_node.cpp` (line ~279)
- **DH parameters source**: `src/Universal_Robots_ROS2_Description/config/ur16e/default_kinematics.yaml`
