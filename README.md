# Stewart Platform Class

This Python class, `StewartPlatform`, provides a comprehensive implementation for analyzing a Stewart platform, a type of parallel manipulator used in robotics and automation. This class includes methods for **Inverse Kinematics**, **Forward Kinematics**, **Jacobian matrix computation**, **Kinematic and Force Analysis**, and **Workspace Analysis**. Check the preview of the .ipynb file for more info.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
  - [Initialization](#initialization)
  - [Inverse Kinematics](#inverse-kinematics)
  - [Jacobian Matrix](#jacobian-matrix)
  - [Forward Kinematics](#forward-kinematics)
  - [Kinematic Analysis](#kinematic-analysis)
  - [Force Analysis](#force-analysis)
  - [Workspace Analysis](#workspace-analysis)
- [Methods Overview](#methods-overview)

## Installation

Ensure you have the required dependencies installed:

```bash
pip install numpy scipy matplotlib
```

## Usage

# Initialization
To create an instance of the StewartPlatform class, you need to provide the following parameters:
- r_b: Radius of the base.
- phi_b: Angle between base joints.
- r_p: Radius of the platform.
- phi_p: Angle between platform joints.
```
from stewart_platform import StewartPlatform

r_b = 1.0
phi_b = 80.0
r_p = 0.5
phi_p = 30.0

platform = StewartPlatform(r_b, phi_b, r_p, phi_p)
```

# Inverse Kinematics
Calculate the lengths of the platform legs given a pose (position and orientation).
```
pose = [0.1, 0.2, 0.3, 10, 20, 30]  # [x, y, z, roll, pitch, yaw]
leg_lengths = platform.getIK(pose)
```

# Jacobian Matrix
Compute the Jacobian matrix, which relates joint velocities to end-effector velocities.

```
jacobian = platform.getJacobian()
```

# Forward Kinematics
Determine the pose of the platform given the lengths of the legs and a starting guess.

```
starting_pose = [0, 0, 0.3, 0, 0, 0]
lengths_desired = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
pose = platform.getFK(starting_pose, lengths_desired)

```
# Kinematic Analysis
Calculate various kinematic indices.

```
singular_value_index = platform.getSingularValueIndex()

manipulability_index = platform.getManipulabilityIndex()

condition_number = platform.getConditionNumber()

local_condition_index = platform.getLocalConditionIndex()
```

# Force Analysis
Analyze the forces in the platform and actuators.

```
F_actuators = [10, 20, 30, 40, 50, 60]
F_platform = platform.getPlatformForces(F_actuators)

F_platform_desired = [15, 25, 35, 45, 55, 65]
F_actuators_required = platform.getActuatorForces(F_platform_desired)

force_ellipsoid = platform.getForceEllipsoid()

ldi = platform.getLDI()

```

# Workspace Analysis
Evaluate the platform's workspace with respect to position and orientation.

```
workspace_limits = [-1, 1, -1, 1, -1, 1]  # [x_min, x_max, y_min, y_max, z_min, z_max]
RPY = [0, 0, 0]  # [roll, pitch, yaw]
N = 10  # Discretization parameter
choice = 1  # Index calculation choice

index_workspace_position = platform.getIndexWorkspacePosition(workspace_limits, RPY, N, choice)

position = [0.1, 0.2, 0.3]
orientation_limits = [-30, 30, -30, 30, -30, 30]  # [roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max]

index_workspace_orientation = platform.getIndexWorkspaceOrientation(position, orientation_limits, N, choice)

```

## Methods Overview
- **getIK(pose):** Computes inverse kinematics.
- **getJacobian():** Returns the Jacobian matrix.
- **getFK(starting_pose, lengths_desired):** Computes forward kinematics.
- **getSingularValueIndex():** Calculates the singular value index.
- **getManipulabilityIndex():** Computes the manipulability index.
- **getConditionNumber():** Returns the condition number.
- **getLocalConditionIndex():** Calculates the local condition index.
- **getPlatformForces(F_actuators):** Computes platform forces from actuator forces.
- **getActuatorForces(F_platform):** Determines actuator forces from platform forces.
- **getForceEllipsoid():** Finds the force ellipsoid.
- **getLDI():** Returns the local design index.
- **getIndexWorkspacePosition(workspace_limits, RPY, N, choice):** Analyzes workspace based on position.
- **getIndexWorkspaceOrientation(position, orientation_limits, N, choice):** Analyzes workspace based on orientation.
- **plot():** Plots the Stewart platform configuration.
