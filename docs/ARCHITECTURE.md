# Kinematica Architecture

## High-Level Overview

Kinematica follows a standard rigid body physics engine architecture. It utilizes a semi-implicit Euler integrator and a sequential impulse solver.

### Simulation Loop (`PhysicsWorld::step`)

The simulation step is divided into several phases:

1.  **Substepping (CCD)**:
    The time step is potentially divided into smaller substeps if Continuous Collision Detection (CCD) is enabled and fast-moving objects are detected.

2.  **Integration (Velocity)**:
    - Apply gravity and external forces.
    - Integrate velocity: $v_{new} = v_{old} + a \cdot dt$.
    - Apply damping (linear and angular).

3.  **Broadphase**:
    - Identifies potential collision pairs using a spatial partitioning scheme (Grid-based).
    - Filters out ignored pairs (e.g., connected bodies in a joint).

4.  **Narrowphase**:
    - Generates contact manifolds for potential pairs.
    - Uses **GJK (Gilbert-Johnson-Keerthi)** for collision detection.
    - Uses **EPA (Expanding Polytope Algorithm)** for penetration depth and contact normal calculation.

5.  **Solver**:
    - **Warm Starting**: Applies cached impulses from previous frames to improve stability.
    - **Sequential Impulse**: Iteratively solves constraints (Contacts, Joints, Springs).
        - **Contacts**: Solves for non-penetration and friction.
        - **Joints**: Solves for Ball-Socket and Hinge constraints.
    - **Restitution**: Applies bias for bouncing.

6.  **Integration (Position)**:
    - Integrate position: $p_{new} = p_{old} + v_{new} \cdot dt$.
    - Integrate orientation: $q_{new} = q_{old} + 0.5 \cdot \omega \cdot q_{old} \cdot dt$.

7.  **Sleeping**:
    - Bodies with low energy for a duration are put to sleep to improve performance.
    - Woken up by collisions or external forces.

## Key Components

### RigidBody
Represents a physical object with mass, inertia, position, orientation, and velocity. It holds a `Collider` which defines its shape.

### Collider
A variant structure supporting multiple shape types:
- **Sphere**
- **Box**
- **Capsule**
- **Convex Hull** (Polyhedron)
- **Triangle Mesh** (BVH-accelerated)
- **Compound** (Hierarchy of shapes)

### Constraints
- **BallSocketJoint**: Constrains two points on two bodies to be at the same location.
- **HingeJoint**: Constrains two bodies to rotate around a shared axis.
- **Spring**: Applies Hooke's law force between two points.

### SoftBody
Simulated as a collection of particles (RigidBodies) connected by springs. Used for cloth and blobs.

## Threading
Kinematica uses OpenMP for parallelizing computationally expensive tasks:
- Spring updates.
- Body integration.
- Contact solving (Islands).
