## Introduction

In this final assignment we will finally consider how to model contact between objects. Specifically we will adapt the unconstrained rigid body simulation from the [previous assignment](https://github.com/dilevin/CSC2549-a5-rigid-bodies/) to support contact resolution by solving a Linear Complimentarity Problem. 

### Prerequisite installation

On all platforms, we will assume you have installed cmake and a modern c++
compiler on Mac OS X[¹](#¹macusers), Linux[²](#²linuxusers), or
Windows[³](#³windowsusers).

We also assume that you have cloned this repository using the `--recursive`
flag (if not then issue `git submodule update --init --recursive`). 

**Note:** We only officially support these assignments on Ubuntu Linux 18.04 (the OS the teaching labs are running) and OSX 10.13 (the OS I use on my personal laptop). While they *should* work on other operating systems, we make no guarantees.  

**All grading of assignments is done on Linux 18.04**

### Layout

All assignments will have a similar directory and file layout: 

    README.md
    CMakeLists.txt
    main.cpp
    assignment_setup.h
    include/
      function1.h
      function2.h
      ...
    src/
      function1.cpp
      function2.cpp
      ...
    data/
      ...
    ...

The `README.md` file will describe the background, contents and tasks of the
assignment.

The `CMakeLists.txt` file setups up the cmake build routine for this
assignment.

The `main.cpp` file will include the headers in the `include/` directory and
link to the functions compiled in the `src/` directory. This file contains the
`main` function that is executed when the program is run from the command line.

The `include/` directory contains one file for each function that you will
implement as part of the assignment.

The `src/` directory contains _empty implementations_ of the functions
specified in the `include/` directory. This is where you will implement the
parts of the assignment.

The `data/` directory contains _sample_ input data for your program. Keep in
mind you should create your own test data to verify your program as you write
it. It is not necessarily sufficient that your program _only_ works on the given
sample data.

## Compilation for Debugging

This and all following assignments will follow a typical cmake/make build
routine. Starting in this directory, issue:

    mkdir build
    cd build
    cmake ..

If you are using Mac or Linux, then issue:

    make

## Compilation for Testing

Compiling the code in the above manner will yield working, but very slow executables. To run the code at full speed, you should compile it in release mode. Starting in the **build directory**, do the following:

    cmake .. -DCMAKE_BUILD_TYPE=Release
    
Followed by:

    make 
  
Your code should now run significantly (sometimes as much as ten times) faster. 

If you are using Windows, then running `cmake ..` should have created a Visual Studio solution file
called `a6-rigid-body-contact.sln` that you can open and build from there. Building the project will generate an .exe file.

Why don't you try this right now?

## Execution

Once built, you can execute the assignment from inside the `build/` using 

    ./a6-rigid-body-contact

While running, you can unpause/pause the simulation by pressing 's' and reset the position of the rigid body by pressing `r`. 

## Background 

In this final assignment we will augment our previous rigid body integrator with a contact resolution mechanism based on the basic laws of contact mechanics. We will begin by describing the conditions to resolve single point contact, extend this to multi point contact and then to the rigid body regime. Finally we will explore a popular, iterative scheme for solving the contact equations, the projected Gauss-Seidel method. 

![Fun with contacting rigid bodies](images/rb_contact.gif)

## Resources

This [paper](https://animation.rwth-aachen.de/media/papers/2012-EG-STAR_Rigid_Body_Dynamics.pdf) provides a detailed overview of rigid body simulation with contact.


## Single Point Contact 
![single point contact](images/single_point_contact.gif)

The image above illustrates the most basic setup for a contact problem. It contains a single circular object (blue disk) resting on an infinite plane (black line). There is a single point of contact between these two objects (the contact point $\mathbf{z}$) and associated with this contact point is a shared surface normal ($\mathbf{n}$). Contact mechanics addresses the question of how two such physical objects will behave when they come into contact. 

Imagine the following scenario, our disk is falling under gravity and we've managed to take a snapshot of it just as it makes contact with our plane, but before the contact has any effect. In classical mechanics, no two pieces of matter can occupy the same region of space, our goal is to figure out how we can prevent that from happening -- how we can prevent the disc from falling through the plane. 

Since this is a physics problem we cannot directly manipulate the positions of the disk, rather all interactions happen through forces acting on either the volume (like gravity) or surface of both objects. This is a consequence of Newton's first law (objects in motion remain in motion unless influenced by an external force). We want to choose these forces so that the push the contacting objects away from each other (back into empty space, sometimes called the feasible space). To do this we are going to define a set of rules that will govern such contact mediated interactions. 

To begin, let's consider where our correcting force will come from. It arises as a consequence of Newton's third law, as the disc pushes on the floor, the floor pushes on the disc. From this arises our first rule of contact mechanics -- that the correcting force, actually the **contact force**, is only non-zero when objects are in contact. 

Now that we've defined when the contact force can be applied, we can further define its properties. The most important property of a contact force is that it cannot "pull" the contacting objects back together (its not sticky). This means that locally, the contact force needs to push the objects apart along the normal direction. Finally we need to ensure that the objects are  no longer on a collision course after applying the contact force. 

Let's try to define these conditions mathematically for a single point contacting a plane. Furthermore let's assume the plane is fixed in space and cannot move no matter how hard the disk pushes on it.  First, let's define our contact force. Since the force has to push our disk away from the floor, it has to have a component acting normal to the contact point. In fact, any force pushing in any other direction is doing nothing to resolve the potential contact. This implies that a reasonable definition of the contact force acting on the disk is $\mathbf{f_z} = \mathbf{n}\alpha$, where $\alpha$ is a scalar. By definition this force acts normal to the contact surface. For the force to pull the disk towards the plane, it has to act in the opposite of the normal direction. To prevent this we can add the constraint $\alpha >= 0$.  Next, we need to make sure this force is only applied when the disk is in contact with the object. Let's create a new function $d\left(\mathbf{z}\right)$ which computes the [signed distance](https://en.wikipedia.org/wiki/Signed_distance_function) between the two objects at the contact point, $\mathbf{z}$. If the objects are in contact then $d=0$ if the disk passes into the plane, $d < 0$ and if the disc is not contacting the surface $d>0$. Let's assume we are constraining our problem such that $d >= 0$, then we can only apply the contact force when $d = 0$. This means that for each contact point either $d=0$ or $alpha = 0$. If we compute $d\cdot \alpha$ it will always be 0. This is called a [complimentarity](https://en.wikipedia.org/wiki/Complementarity_theory) condition and it is a kind of orthogonality condition. Accordingly we write this as $d \perp \alpha$.  These collected constraints,

$$\begin{array}{rcl} \mathbf{f_z} &=& \mathbf{n}\alpha \\ d &\perp& \alpha \\ \alpha &>=& 0 \\ d &>=& 0,\end{array}$$

are a form of the [**Signorini Conditions**](https://en.wikipedia.org/wiki/Signorini_problem) which arise in all manner of contact problems. 

## Multi Point Contact and the Equations of Motion 

![multi point contact](images/multi_point_contact.gif)

Now that we have a mathematical model for a single contact point, let's extend it to multiple points. Let's imagine we have a **collison detector** which will check for collisions between all objects in our scene and return a list of collision points, collision normals and a pair of object identifiers for each contact. In the general case, when neither object in the contacting pair is fixed, we must consider the effect of the contact force on both participating bodies.  For this $z^{th}$ contact we get 

$$\begin{array}{rcl} \mathbf{f^A_z} &=& -\mathbf{n}_z\alpha_z \\ \mathbf{f^B_z} &=& \mathbf{n}_z\alpha_z \\ d\left(\mathbf{z^A}, \mathbf{z^B}\right) &\perp& \alpha_z \\ \alpha_z &>=& 0 \\ d\left(\mathbf{z^A}, \mathbf{z^B}\right) &>=& 0,\end{array}$$ 

where $A$ and $B$ index the two objects involved in this collision. $d\left(\mathbf{z_A}, \mathbf{z_B}\right)$ measures the distance, in world space, between the contact point on object $A$ and the contact point on object $B$. As we move object $A$ and $B$ around to try and solve this problem,the contact point $z$ will no longer be a single point, rather it will become two points, one attached to each object. Typically we compute this by mapping the original $z$ to the undeformed space of each object, then mapping it back to the world space as the objects move. We compute $d$ on these new, separate points. Finally, notice that our contact forces are setup to ensure that an equal and opposite force is applied to each object at the contact point.  Each contact introduces a new set of forces on some object $A$ and some object $B$, along with the appropriate complimentarity constraints. 

## The Equations of Motion for a Single Rigid Body with Multiple Contacts 

Now that we have a mathematical model for contact forces, we need to combine them with our equations of motion. Since we are simulating rigid bodies, those equations will be the Newton-Euler Equations. Let's make life a little bit easier by considering only a single rigid body, colliding with fixed objects in the scene. The equations of motion tell us that inertial forces need to balance all other external forces acting on the object. The sum of all contact forces acting on a single rigid body is given by 

$$ \mathbf{f}_c = \sum_{z} \pm\mathbf{n}_z\alpha_z, $$ 

where the $\pm$ depends on whether the object is $A$ or $B$ in the collision pair. Now $\mathbf{f}_c$ is a world space force in $\mathcal{R}^3$. We know that to convert this to a rigid body force (a torque and a center-of-mass force) we need to multiply by the transpose of the rigid body jacobian $G \in \mathcal{R}^{3 \times 6}$, evaluated at the appropriate undeformed point $\mathbf{Z}$. Here $\mathbf{Z}$ is the contact point $\mathbf{z}$ transformed from world into undeformed space (i.e you need to apply the inverse of the rigid body transform). This gives us the following constrained equations of motion


$$ \begin{bmatrix} R\mathcal{I}R^T & 0 \\ 0 & I\end{bmatrix}\begin{bmatrix}\dot{\omega} \\ \ddot{p} \end{bmatrix} = \begin{bmatrix}\omega\times\left(R\mathcal{I}R^T\omega\right)+\tau_{ext} \\ \mathbf{f}_{ext}\end{bmatrix}  + \sum_{z} \pm G\left(\mathbf{Z}\left(\mathbf{z}\right)\right)^T\mathbf{n}_z\alpha_z$$

which we can write in matrix form as 

$$ \begin{bmatrix} R\mathcal{I}R^T & 0 \\ 0 & I\end{bmatrix}\begin{bmatrix}\dot{\omega} \\ \ddot{p} \end{bmatrix} = \begin{bmatrix}\omega\times\left(R\mathcal{I}R^T\omega\right)+\tau_{ext} \\ \mathbf{f}_{ext}\end{bmatrix}  + \mathbf{G}\mathbf{N}\mathbf{\alpha}, $$

where $\mathbf{G}$ is a $6 \times 3n_{contacts}$ matrix of stacked $G^T$ matrices, $\mathbf{N}$ is a $3n_{contacts}\times n_{contacts}$ matrix where each column contains a single contact normal, and $\mathbf{\alpha}$ is a $n_{contacts}$ vector of contact force magnitudes.  Using these matrix variables one can see that the remaining complimentarity conditions for this entire dynamic system become 

$$\begin{array}{rcl} \mathbf{d} &\perp& \mathbf{\alpha} \\ \mathbf{\alpha} &>=& 0 \\ \mathbf{d} &>=& 0\end{array}$$

where $\mathbf{d}$ returns a vector of distances, one for each contact and all inequalities apply component wise to the associated vectors. 

## The Velocity Level Signorini Conditions

## Solving the Contact Problem using Projected Gauss Seidel 

## Assignment Implementation

In this assignment you will adapt your previous, unconstrained rigid body integrator to handle contact using the projected Gauss-Seidel algorithm. 

### rodrigues.cpp

**Use code from previous assignment.**

### inverse_rigid_body.cpp

A method to transform a point from world (deformed) space to body (undeformed) space. 

### rigid_body_jacobian.cpp

**Use code from previous assignment.**

### inertia_matrix.cpp

**Use code from previous assignment.**

### collision_box_floor.cpp

Detect contact between a triangle mesh and an arbitrarily positioned plane.

### dV_spring_particle_particle_dq.cpp

**Use code from previous assignment.**

### exponential_euler_lcp_contact.h

Implement velocity level collision resolution using progressive Gauss-Seidel and exponential Euler time integration. 

### pick_nearest_vertices.cpp

**Use code from previous assignment.**

