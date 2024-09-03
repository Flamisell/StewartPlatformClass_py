# Stewart Platform Class

This Python class, `StewartPlatform`, provides a comprehensive implementation for analyzing a Stewart platform, a type of parallel manipulator used in robotics and automation. This class includes methods for **Inverse Kinematics**, **Forward Kinematics**, **Jacobian matrix computation**, **Kinematic and Force Analysis**, **Workspace Analysis** and **Singularity Finder**. Check the preview of the .ipynb file for more info.

<img src="https://github.com/Flamisell/StewartPlatform_py/blob/main/img/FKplatform3.png" width="400"> <img src="https://github.com/Flamisell/StewartPlatform_py/blob/main/img/newplot.png" width="400"> 
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
  - [Singularity Finder](#singularity-finder)
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
# Define parameters
r_b = 0.5  # Radius of base
phi_b = 50  # Angle between base joints
r_p = 0.3  # Radius of platform
phi_p = 80  # Angle between platform joints

# Create Stewart Platform instance
platform = StewartPlatform(r_b, phi_b, r_p, phi_p)
```

# Inverse Kinematics
Calculate the lengths of the platform legs given a pose (position and orientation).
```
pose = [0.2, 0, 0.6, 10, 20, 0]  # [x, y, z, roll, pitch, yaw]
leg_lengths = platform.getIK(pose)
platform.plot()
```
<img src="https://github.com/Flamisell/StewartPlatform_py/blob/main/img/IK_platform.png" width="500">


# Jacobian Matrix
Compute the Jacobian matrix, which relates joint velocities to end-effector velocities.

```
jacobian_matrix = platform.getJacobian()
```

# Forward Kinematics
Determine the pose of the platform given the lengths of the legs and a starting guess.

```
starting_pose = [0, 0, 0.2, 0, 0, 0]  # Initial guess for the pose
lengths_desired = np.linalg.norm(leg_lengths,axis=1)  # Use the lengths obtained from IK
plot=True
estimated_pose = platform.getFK(starting_pose, lengths_desired, plot)

```
<img src="https://github.com/Flamisell/StewartPlatform_py/blob/main/img/FKplatform1.png" width="330"> <img src="https://github.com/Flamisell/StewartPlatform_py/blob/main/img/FKplatform2.png" width="330"><img src="https://github.com/Flamisell/StewartPlatform_py/blob/main/img/FKplatform3.png" width="330">




# Kinematic Analysis
Calculate various kinematic indices.

```
# Get Singular Value Index
# measures drive capability of the platform, finds max q_dot under unitary x_dot
singular_value_index = platform.getSingularValueIndex()

# Get Manipulability Index
# Measures manipulability of manipulator, can be used to optimize it's configuration
manipulability_index = platform.getManipulabilityIndex()

# Get Condition Number
# Measures closeness to isotropic configuration [1,+ inf)
condition_number = platform.getConditionNumber()

# Get Local Condition Index
# Measures closeness to isotropic configuration (0,1]
local_condition_index = platform.getLocalConditionIndex()


```

# Force Analysis
Analyze the forces in the platform and actuators.

```
# Calculate Platform Forces given Actuator Forces
F_actuators = [10, 10, 10, 10, 10, 10]  # Example actuator forces
F_platform = platform.getPlatformForces(F_actuators)

# Calculate Actuator Forces given Platform Forces
F_platform = [10, 10, 10, 10, 10, 10]  # Example platform forces
F_actuators = platform.getActuatorForces(F_platform)

# Get Force Ellipsoid
force_ellipsoid = platform.getForceEllipsoid()

# Get Local Design Index (LDI)
# Local design index for Force transmittability (actuator design)
ldi = platform.getLDI()

```

# Workspace Analysis
Evaluate the platform's workspace with respect to position and orientation.

```
# Define workspace limits [x_min, x_max, y_min, y_max, z_min, z_max]
workspace_limits = [-0.5, 0.5, -0.5, 0.5, 0.1, 0.6]
RPY = [0, 0, 0]  # Fixed orientation (roll, pitch, yaw)
N = 10  # Number of points in each dimension
choice = 4  # Choice of index calculation (1: Singular Value Index, etc.)
            # self.options = {
            #     1: self.getSingularValueIndex, # measures drive capability of the platform, finds max q_dot under unitary x_dot
            #     2: self.getManipulabilityIndex,# Measures manipulability of manipulator, can be used to optimize it's configuration
            #     3: self.getConditionNumber,# Measures closeness to isotropic configuration [1,+ inf)
            #     4: self.getLocalConditionIndex,# Measures closeness to isotropic configuration (0,1]
            #     5: self.getLDI # Local design index for Force transmittability (actuator design)
            #     6: self.getLocalConditionIndexT # Measures closeness to force isotropic configuration, 0 when joint forces go to infinity.
            # }

workspace_indices_position = platform.getIndexWorkspacePosition(workspace_limits, RPY, N, choice)
print("Workspace Indices (Position):", workspace_indices_position)

# Define orientation limits [roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max]
orientation_limits = [-10, 10, -10, 10, -10, 10]
position = [0, 0, 0.4]  # Fixed position

workspace_indices_orientation = platform.getIndexWorkspaceOrientation(position, orientation_limits, N, choice)
print("Workspace Indices (Orientation):", workspace_indices_orientation)

```
There is also the possibility to use plotly to plot the values in all the defined workspace
<img src="https://github.com/Flamisell/StewartPlatform_py/blob/main/img/newplot.png" width="400"> <img src="https://github.com/Flamisell/StewartPlatform_py/blob/main/img/newplot%20(1).png" width="400">

# Singularity Finder
Evaluate singularities over a range of positions in the workspace.

```
# Define workspace limits 
workspace_limits = [-0.5, 0.5, -0.5, 0.5, 0.1, 0.6]
orientation_limits = [-10, 10, -10, 10, -10, 10]
# Define number of points for dimension
N_pos = 10  # Number of points in each dimension
N_orient = 10  # Number of points in each dimension

# Choosing N_pos and N_orient too high may result in a computational expensive operation, suggested values ( N_pos=10, N_orient=10 )
# for practical usage there is the need to filter the data. Suggestion: filter by local condition index value AND by distance between data points (from scipy.spatial.distance import cdist).

singularities_task_space = spider.getSingularityWorkspace(workspace_limits,orientation_limits,N_pos,N_orient) # find singularities in all space

print("Singularities in task space:", singularities_task_space)
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
- **getSingularityWorkspace(workspace_limits,orientation_limits,N_pos,N_orient):** Evaluate singularities over a range of positions in the workspace.
- **plot():** Plots the Stewart platform configuration.
