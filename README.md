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

The image above illustrates the most basic setup for a contact problem. It contains a single circular object (blue disk) resting on an infinite plane (black line). There is a single point of contact between these two objects (the contact point <img src="/tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode&sanitize=true" align=middle width=8.40178184999999pt height=14.611878600000017pt/>) and associated with this contact point is a shared surface normal (<img src="/tex/b56595d2a30a0af329086562ca12d521.svg?invert_in_darkmode&sanitize=true" align=middle width=10.502226899999991pt height=14.611878600000017pt/>). Contact mechanics addresses the question of how two such physical objects will behave when they come into contact. 

Imagine the following scenario, our disk is falling under gravity and we've managed to take a snapshot of it just as it makes contact with our plane, but before the contact has any effect. In classical mechanics, no two pieces of matter can occupy the same region of space, our goal is to figure out how we can prevent that from happening -- how we can prevent the disc from falling through the plane. 

Since this is a physics problem we cannot directly manipulate the positions of the disk, rather all interactions happen through forces acting on either the volume (like gravity) or surface of both objects. This is a consequence of Newton's first law (objects in motion remain in motion unless influenced by an external force). We want to choose these forces so that the push the contacting objects away from each other (back into empty space, sometimes called the feasible space). To do this we are going to define a set of rules that will govern such contact mediated interactions. 

To begin, let's consider where our correcting force will come from. It arises as a consequence of Newton's third law, as the disc pushes on the floor, the floor pushes on the disc. From this arises our first rule of contact mechanics -- that the correcting force, actually the **contact force**, is only non-zero when objects are in contact. 

Now that we've defined when the contact force can be applied, we can further define its properties. The most important property of a contact force is that it cannot "pull" the contacting objects back together (its not sticky). This means that locally, the contact force needs to push the objects apart along the normal direction. Finally we need to ensure that the objects are  no longer on a collision course after applying the contact force. 

Let's try to define these conditions mathematically for a single point contacting a plane. Furthermore let's assume the plane is fixed in space and cannot move no matter how hard the disk pushes on it.  First, let's define our contact force. Since the force has to push our disk away from the floor, it has to have a component acting normal to the contact point. In fact, any force pushing in any other direction is doing nothing to resolve the potential contact. This implies that a reasonable definition of the contact force acting on the disk is <img src="/tex/03dea39ef18060d787da8c65a8aed874.svg?invert_in_darkmode&sanitize=true" align=middle width=56.22460304999999pt height=22.831056599999986pt/>, where <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> is a scalar. By definition this force acts normal to the contact surface. For the force to pull the disk towards the plane, it has to act in the opposite of the normal direction. To prevent this we can add the constraint <img src="/tex/781400a92cac6b7af3ef9ab783d03712.svg?invert_in_darkmode&sanitize=true" align=middle width=53.49877169999999pt height=21.18721440000001pt/>.  Next, we need to make sure this force is only applied when the disk is in contact with the object. Let's create a new function <img src="/tex/58b92fa639e63c027eb5ec58a62a56d4.svg?invert_in_darkmode&sanitize=true" align=middle width=32.48283554999999pt height=24.65753399999998pt/> which computes the [signed distance](https://en.wikipedia.org/wiki/Signed_distance_function) between the two objects at the contact point, <img src="/tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode&sanitize=true" align=middle width=8.40178184999999pt height=14.611878600000017pt/>. If the objects are in contact then <img src="/tex/a561c9ace81b9b2621c9a73d14b39426.svg?invert_in_darkmode&sanitize=true" align=middle width=38.69280359999998pt height=22.831056599999986pt/> if the disk passes into the plane, <img src="/tex/815fd9129ec271db15b23d2d230f9a67.svg?invert_in_darkmode&sanitize=true" align=middle width=38.69280359999998pt height=22.831056599999986pt/> and if the disc is not contacting the surface <img src="/tex/3e93ba65b33deaea4f4403dd4a07c14f.svg?invert_in_darkmode&sanitize=true" align=middle width=38.69280359999998pt height=22.831056599999986pt/>. Let's assume we are constraining our problem such that <img src="/tex/e139eaa881043a671cb9059709a44663.svg?invert_in_darkmode&sanitize=true" align=middle width=51.47823779999998pt height=22.831056599999986pt/>, then we can only apply the contact force when <img src="/tex/6a1084d56bbb789d6017da77ba2ee276.svg?invert_in_darkmode&sanitize=true" align=middle width=38.69280359999998pt height=22.831056599999986pt/>. This means that for each contact point either <img src="/tex/a561c9ace81b9b2621c9a73d14b39426.svg?invert_in_darkmode&sanitize=true" align=middle width=38.69280359999998pt height=22.831056599999986pt/> or <img src="/tex/2c2244dc5de6df9a1e9ef4a908994852.svg?invert_in_darkmode&sanitize=true" align=middle width=70.48517354999998pt height=22.831056599999986pt/>. If we compute <img src="/tex/a9e7c4cb5b87ff1b4ff005c76146fcac.svg?invert_in_darkmode&sanitize=true" align=middle width=31.00445039999999pt height=22.831056599999986pt/> it will always be 0. This is called a [complimentarity](https://en.wikipedia.org/wiki/Complementarity_theory) condition and it is a kind of orthogonality condition. Accordingly we write this as <img src="/tex/7418c1739ad7e18780764989880a3f1b.svg?invert_in_darkmode&sanitize=true" align=middle width=41.05009919999999pt height=22.831056599999986pt/>.  These collected constraints,

<p align="center"><img src="/tex/e622bf17c5bd5ce90aad0d474bda1815.svg?invert_in_darkmode&sanitize=true" align=middle width=92.75453879999999pt height=73.78996185pt/></p>

are a form of the [**Signorini Conditions**](https://en.wikipedia.org/wiki/Signorini_problem) which arise in all manner of contact problems. 

## Multi Point Contact and the Equations of Motion 

![multi point contact](images/multi_point_contact.gif)

Now that we have a mathematical model for a single contact point, let's extend it to multiple points. Let's imagine we have a **collison detector** which will check for collisions between all objects in our scene and return a list of collision points, collision normals and a pair of object identifiers for each contact. In the general case, when neither object in the contacting pair is fixed, we must consider the effect of the contact force on both participating bodies.  For this <img src="/tex/19957acc3e1071d1174e0b50710e50cb.svg?invert_in_darkmode&sanitize=true" align=middle width=21.02943809999999pt height=27.91243950000002pt/> contact we get 

<p align="center"><img src="/tex/404c18b7a22883735caf687b5285def6.svg?invert_in_darkmode&sanitize=true" align=middle width=180.35016944999998pt height=98.89918334999999pt/></p> 

where <img src="/tex/53d147e7f3fe6e47ee05b88b166bd3f6.svg?invert_in_darkmode&sanitize=true" align=middle width=12.32879834999999pt height=22.465723500000017pt/> and <img src="/tex/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode&sanitize=true" align=middle width=13.29340979999999pt height=22.465723500000017pt/> index the two objects involved in this collision. <img src="/tex/c26f66007cb589124adc6bc0d9305a04.svg?invert_in_darkmode&sanitize=true" align=middle width=71.48954174999999pt height=24.65753399999998pt/> measures the distance, in world space, between the contact point on object <img src="/tex/53d147e7f3fe6e47ee05b88b166bd3f6.svg?invert_in_darkmode&sanitize=true" align=middle width=12.32879834999999pt height=22.465723500000017pt/> and the contact point on object <img src="/tex/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode&sanitize=true" align=middle width=13.29340979999999pt height=22.465723500000017pt/>. As we move object <img src="/tex/53d147e7f3fe6e47ee05b88b166bd3f6.svg?invert_in_darkmode&sanitize=true" align=middle width=12.32879834999999pt height=22.465723500000017pt/> and <img src="/tex/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode&sanitize=true" align=middle width=13.29340979999999pt height=22.465723500000017pt/> around to try and solve this problem,the contact point <img src="/tex/f93ce33e511096ed626b4719d50f17d2.svg?invert_in_darkmode&sanitize=true" align=middle width=8.367621899999993pt height=14.15524440000002pt/> will no longer be a single point, rather it will become two points, one attached to each object. Typically we compute this by mapping the original <img src="/tex/f93ce33e511096ed626b4719d50f17d2.svg?invert_in_darkmode&sanitize=true" align=middle width=8.367621899999993pt height=14.15524440000002pt/> to the undeformed space of each object, then mapping it back to the world space as the objects move. We compute <img src="/tex/2103f85b8b1477f430fc407cad462224.svg?invert_in_darkmode&sanitize=true" align=middle width=8.55596444999999pt height=22.831056599999986pt/> on these new, separate points. Finally, notice that our contact forces are setup to ensure that an equal and opposite force is applied to each object at the contact point.  Each contact introduces a new set of forces on some object <img src="/tex/53d147e7f3fe6e47ee05b88b166bd3f6.svg?invert_in_darkmode&sanitize=true" align=middle width=12.32879834999999pt height=22.465723500000017pt/> and some object <img src="/tex/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode&sanitize=true" align=middle width=13.29340979999999pt height=22.465723500000017pt/>, along with the appropriate complimentarity constraints. 

## The Equations of Motion for a Single Rigid Body with Multiple Contacts 

Now that we have a mathematical model for contact forces, we need to combine them with our equations of motion. Since we are simulating rigid bodies, those equations will be the Newton-Euler Equations. Let's make life a little bit easier by considering only a single rigid body, colliding with fixed objects in the scene. The equations of motion tell us that inertial forces need to balance all other external forces acting on the object. The sum of all contact forces acting on a single rigid body is given by 

<p align="center"><img src="/tex/74c4515ebe7f646c3cf523ad0f8b6ab5.svg?invert_in_darkmode&sanitize=true" align=middle width=114.39249524999998pt height=36.16460595pt/></p> 

where the <img src="/tex/f62db12f95e34116f1f1e827b2c64ce5.svg?invert_in_darkmode&sanitize=true" align=middle width=12.785434199999989pt height=19.1781018pt/> depends on whether the object is <img src="/tex/53d147e7f3fe6e47ee05b88b166bd3f6.svg?invert_in_darkmode&sanitize=true" align=middle width=12.32879834999999pt height=22.465723500000017pt/> or <img src="/tex/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode&sanitize=true" align=middle width=13.29340979999999pt height=22.465723500000017pt/> in the collision pair. Now <img src="/tex/216f21f26c666c9b7407232b4084fa06.svg?invert_in_darkmode&sanitize=true" align=middle width=11.65087769999999pt height=22.831056599999986pt/> is a world space force in <img src="/tex/9e32a55e4e809956cef5fff14c39b80d.svg?invert_in_darkmode&sanitize=true" align=middle width=20.484113099999988pt height=26.76175259999998pt/>. We know that to convert this to a rigid body force (a torque and a center-of-mass force) we need to multiply by the transpose of the rigid body jacobian <img src="/tex/169657c6bf8366cfc815200cb738dc94.svg?invert_in_darkmode&sanitize=true" align=middle width=70.32646334999998pt height=26.76175259999998pt/>, evaluated at the appropriate undeformed point <img src="/tex/f73961fbdccf4a0a52e926e49663cb52.svg?invert_in_darkmode&sanitize=true" align=middle width=11.55245024999999pt height=22.55708729999998pt/>. Here <img src="/tex/f73961fbdccf4a0a52e926e49663cb52.svg?invert_in_darkmode&sanitize=true" align=middle width=11.55245024999999pt height=22.55708729999998pt/> is the contact point <img src="/tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode&sanitize=true" align=middle width=8.40178184999999pt height=14.611878600000017pt/> transformed from world into undeformed space (i.e you need to apply the inverse of the rigid body transform). This gives us the following constrained equations of motion

<p align="center"><img src="/tex/324c14953348465c7d45d5fe3ac1c9cd.svg?invert_in_darkmode&sanitize=true" align=middle width=479.2516047pt height=42.8221167pt/></p>

which we can write in matrix form as 

<p align="center"><img src="/tex/096ee1f1713c5e0e12cc912430781ece.svg?invert_in_darkmode&sanitize=true" align=middle width=372.41348759999994pt height=39.534552749999996pt/></p>

where <img src="/tex/ae44fa1818647ed39ba79b316ce5dd87.svg?invert_in_darkmode&sanitize=true" align=middle width=14.86294424999999pt height=22.55708729999998pt/> is a <img src="/tex/c14e38732950511e6cf4af9a6e34a1d2.svg?invert_in_darkmode&sanitize=true" align=middle width=96.0267528pt height=21.18721440000001pt/> matrix of stacked <img src="/tex/b56beee853be98ce01ab38dba54c009b.svg?invert_in_darkmode&sanitize=true" align=middle width=22.45836284999999pt height=27.6567522pt/> matrices, <img src="/tex/bccab73005d96290c8ef588703533a21.svg?invert_in_darkmode&sanitize=true" align=middle width=14.794451099999991pt height=22.55708729999998pt/> is a <img src="/tex/05c8e25e420a67b0cfc84c601cfddcb6.svg?invert_in_darkmode&sanitize=true" align=middle width=148.12652415pt height=21.18721440000001pt/> matrix where each column contains a single contact normal, and <img src="/tex/0c8edc55152aecfe6d318d888c21e812.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> is a <img src="/tex/0d8c9010b933336238a6fdbe69049339.svg?invert_in_darkmode&sanitize=true" align=middle width=59.497143749999985pt height=14.15524440000002pt/> vector of contact force magnitudes.  Using these matrix variables one can see that the remaining complimentarity conditions for this entire dynamic system become 

<p align="center"><img src="/tex/e77837c91e50df024fa1950dced1bd17.svg?invert_in_darkmode&sanitize=true" align=middle width=79.60056885pt height=51.510287399999996pt/></p>

where <img src="/tex/a1a0237b674867e56447da2abd68e758.svg?invert_in_darkmode&sanitize=true" align=middle width=10.502226899999991pt height=22.831056599999986pt/> returns a vector of distances, one for each contact and all inequalities apply component wise to the associated vectors. 

## The Velocity Level Signorini Conditions

The system of equations and inequalities above is difficult to solve -- there is a lot of nonlinearity hidden in the calculation of the distance function. One way to side step this is to linearize the equations and solve them at the velocity level. Let's look at how this is done for a single contact. 

We begin by computing <img src="/tex/b8fdcd052fc5f1361b8d834d4e1e5c61.svg?invert_in_darkmode&sanitize=true" align=middle width=86.27741265pt height=28.92634470000001pt/> which can be conveniently computed as <img src="/tex/ef394074ea4c0b53feaff655cfb5445c.svg?invert_in_darkmode&sanitize=true" align=middle width=97.58313179999999pt height=27.94539330000001pt/>. Rather than solve the nonlinear complimentarity problem above, we will solve the following [linear complimentarity problem (LCP)](https://en.wikipedia.org/wiki/Linear_complementarity_problem)

<p align="center"><img src="/tex/a9b89f2a3a2c7a4d3987b03f0ba90abd.svg?invert_in_darkmode&sanitize=true" align=middle width=166.60719569999998pt height=59.342801099999996pt/></p>

Now we need a way to relate <img src="/tex/5054c8a0cbac06dcee5cb54477c19a46.svg?invert_in_darkmode&sanitize=true" align=middle width=19.538760449999987pt height=33.089472899999976pt/> and <img src="/tex/dec6e10d5d5f1b585ca37c8ddf3daa5c.svg?invert_in_darkmode&sanitize=true" align=middle width=18.92003519999999pt height=33.089472899999976pt/> to <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>. Fortunately we already have such a set of equations, they are the discretized equations of motion from the [previous assignment](https://github.com/dilevin/CSC2549-a5-rigid-bodies/). We have already seen that, for the rigid body <img src="/tex/53d147e7f3fe6e47ee05b88b166bd3f6.svg?invert_in_darkmode&sanitize=true" align=middle width=12.32879834999999pt height=22.465723500000017pt/>.

<p align="center"><img src="/tex/15baae93e73989d35509a671925800ed.svg?invert_in_darkmode&sanitize=true" align=middle width=646.3435109999999pt height=68.52087660000001pt/></p>

A little bit of rearranging gives 

<p align="center"><img src="/tex/73fe574c8e0c46361055fb72b90329bb.svg?invert_in_darkmode&sanitize=true" align=middle width=268.00199250000003pt height=45.99263295pt/></p> 

where <img src="/tex/b1bfba17d7990fcf3f6f06fb783eaf9c.svg?invert_in_darkmode&sanitize=true" align=middle width=31.749943499999986pt height=26.76175259999998pt/> is the unconstrained velocity at time <img src="/tex/628783099380408a32610228991619a8.svg?invert_in_darkmode&sanitize=true" align=middle width=34.24649744999999pt height=21.18721440000001pt/> (exactly what we computed for the last assignment). So now we see that, for a single contact there's a relatively simple relationship between the contact force magnitude, <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, and the resulting velocity.  Now we can use the rigid body jacobian to relate this to the velocity of the contact point, <img src="/tex/84f39dd90ae59db861a16dcfc66d956d.svg?invert_in_darkmode&sanitize=true" align=middle width=18.28769249999999pt height=27.6567522pt/>. At the end of each time step, the velocity of <img src="/tex/84f39dd90ae59db861a16dcfc66d956d.svg?invert_in_darkmode&sanitize=true" align=middle width=18.28769249999999pt height=27.6567522pt/> will be

<p align="center"><img src="/tex/51bc86176aa9086a17082e7458332555.svg?invert_in_darkmode&sanitize=true" align=middle width=471.49214309999996pt height=24.2029623pt/></p>

Formula's for rigid body <img src="/tex/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode&sanitize=true" align=middle width=13.29340979999999pt height=22.465723500000017pt/> are analogous except rather than subtracting contact forces, we **add** them (equal and opposite :) ). 

Because we can now express <img src="/tex/5054c8a0cbac06dcee5cb54477c19a46.svg?invert_in_darkmode&sanitize=true" align=middle width=19.538760449999987pt height=33.089472899999976pt/> and <img src="/tex/dec6e10d5d5f1b585ca37c8ddf3daa5c.svg?invert_in_darkmode&sanitize=true" align=middle width=18.92003519999999pt height=33.089472899999976pt/> directly as functions of <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, and because these expressions encode the physical behaviour of our rigid body (via the equations of motion) we can now start to solve this complimentarity problem. Let's start by examining the inequality <img src="/tex/5730f5323a6207c98db1eb2306a699d4.svg?invert_in_darkmode&sanitize=true" align=middle width=140.50539855pt height=27.94539330000001pt/>. 

We can now rephrase this inequality completely in terms of <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, which looks like this

<p align="center"><img src="/tex/1054a987266ec13bcfe457b93647e890.svg?invert_in_darkmode&sanitize=true" align=middle width=804.60862185pt height=53.5253565pt/></p>

Let's ignore the inequality and just find <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> such that <img src="/tex/e6b3034a18b1c4911dae98c96be4db52.svg?invert_in_darkmode&sanitize=true" align=middle width=127.71996434999998pt height=27.94539330000001pt/>. This is given by <img src="/tex/ffa93664b733c40892654f378056540a.svg?invert_in_darkmode&sanitize=true" align=middle width=54.86490569999999pt height=24.575218800000012pt/>. That's a very convenient way to find the magnitude of the contact force that will (linearly) pull our objects so that the distance between their contacting points is zero. The problem is we've ignored all the other parts of the LCP. 

One thing that we know is <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> must always be greater than, or equal to <img src="/tex/29632a9bf827ce0200454dd32fc3be82.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=21.18721440000001pt/>, and it can only be non-zero if <img src="/tex/e6b3034a18b1c4911dae98c96be4db52.svg?invert_in_darkmode&sanitize=true" align=middle width=127.71996434999998pt height=27.94539330000001pt/>. One obvious thing to is compute our final contact force magnitude as 

<p align="center"><img src="/tex/a7bdaa2a11a0b72a91e56488d2ea9ca7.svg?invert_in_darkmode&sanitize=true" align=middle width=127.14183075pt height=30.1801401pt/></p>

This ensures <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> is always greater than zero. It also ensures this only happens when the gap between contact points is <img src="/tex/29632a9bf827ce0200454dd32fc3be82.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=21.18721440000001pt/> (because that's wht we solved for). Interestingly, due to the structure of the <img src="/tex/38f1e2a089e53d5c990a82f284948953.svg?invert_in_darkmode&sanitize=true" align=middle width=7.928075099999989pt height=22.831056599999986pt/> term, one only gets a positive <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> when <img src="/tex/a89997741c635097e570db8a18d0bd29.svg?invert_in_darkmode&sanitize=true" align=middle width=140.50539855pt height=27.94539330000001pt/>. This means that contact forces are only applied when the objects are in contact or when one object is inside another (something which we should definitely correct for). Thus, this simple operation solves our single contact point LCP. Next we will use this as the building block of a multi-point contact handling algorithm.

## Solving the Multi-Point Contact Problem using Projected Gauss Seidel 

With multiple points things get a little trickier, we need to somehow satisfy all our complimentarity conditions at once. One way of doing this is to solve a [quadratic program](https://en.wikipedia.org/wiki/Quadratic_programming) at the velocity level. It turns out that certain LCPs (like the one we solve) define the optimality criteria for quadratic programs and solving one is the same as solving the other. For this project we will **not** solve a QP, instead we will use an iterative method ([projected Gauss-Seidel](http://www.optimization-online.org/DB_FILE/2007/06/1675.pdf)) to directly solve the LCP. This approach is extremely popular in physics-based animation and well worth understanding.

Let's imagine we have <img src="/tex/0d8c9010b933336238a6fdbe69049339.svg?invert_in_darkmode&sanitize=true" align=middle width=59.497143749999985pt height=14.15524440000002pt/> contact points. We can augment our contact modified, rigid body update equation with them in the following manner:

<p align="center"><img src="/tex/32d8d160fa450388dcaa7abd150d52d2.svg?invert_in_darkmode&sanitize=true" align=middle width=328.4389119pt height=47.3426514pt/></p> 

We are going to use this equation to solve (and its counterpart for rigid bod <img src="/tex/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode&sanitize=true" align=middle width=13.29340979999999pt height=22.465723500000017pt/>) to solve the contact problem by iteratively updating our <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>'s one-at-a-time. We begin with an initial guess for each <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> (say <img src="/tex/29632a9bf827ce0200454dd32fc3be82.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=21.18721440000001pt/>). The basic algorithm proceeds in the following manner (sorry about the non-processed latex, its a markdown thing that I cannot figure out).

    iterations = 0
    all $\alpha$ = 0

    While i < max iterations
        
        For c = 0 to number of contacts - 1
             
             Compute $\delta_c$ 

             Compute $\gamma_c$

             Compute $alpha^{i}_c = \max(0,-frac{\gamma_c}{\delta_c})$

        End
    
    End

The remaining goal is to come up with formulas for <img src="/tex/a0f4dc8e2aa4dd40aaa873397e83c252.svg?invert_in_darkmode&sanitize=true" align=middle width=14.05829699999999pt height=22.831056599999986pt/> and <img src="/tex/5e607c9771c90017b79d70770f507a75.svg?invert_in_darkmode&sanitize=true" align=middle width=15.262999949999992pt height=14.15524440000002pt/>.  We do this in a Gauss-Seidel fashion, by dividing the contact forces into three groups -- (1) forces that have been updated this iteration, (2) forces that have yet to be updated and (3) the contact we are currently solving for. With this grouping we arrive at a formula for rigid body velocity that looks like this:

<p align="center"><img src="/tex/2d93fd570f2d761909caf506dafed4e0.svg?invert_in_darkmode&sanitize=true" align=middle width=770.1171026999999pt height=73.83572955pt/></p> 

Note that <img src="/tex/c7f539fb243cda00c7bf9f47bc19e3d7.svg?invert_in_darkmode&sanitize=true" align=middle width=42.687501449999985pt height=27.6567522pt/> and <img src="/tex/3fa3feb50e746a701fbb01c9e97b8d60.svg?invert_in_darkmode&sanitize=true" align=middle width=42.50485469999999pt height=27.6567522pt/> can be computed using the values of <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> at this particular point in the contact point iteration. Solving for the updated <img src="/tex/971651e87c9aebb6ec102860c98ae161.svg?invert_in_darkmode&sanitize=true" align=middle width=16.390298099999992pt height=14.15524440000002pt/> is analogous to the single point case. For the contact, <img src="/tex/3e18a4a28fdee1744e5e3f79d13b9ff6.svg?invert_in_darkmode&sanitize=true" align=middle width=7.11380504999999pt height=14.15524440000002pt/>, we are currently visiting, we construct

<p align="center"><img src="/tex/dd8618f5d568c7c6daf5937d96a4f009.svg?invert_in_darkmode&sanitize=true" align=middle width=561.6525667499999pt height=99.78292334999999pt/></p>

We then compute <img src="/tex/1bd40ffd3f22f94d7a047f2d6999dabe.svg?invert_in_darkmode&sanitize=true" align=middle width=137.4183129pt height=37.80850590000001pt/>. The method gets its name due to the Gauss-Seidel like ordering of the <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> updates and the projection of computed <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>'s onto the set of positive, real numbers. 

While this algorithm can be run to convergence, for interactive applications it is best to limit the number of outer iterations. In our case we will set the maximum number of outer iterations to be **10**. 

## Assignment Implementation

In this assignment you will adapt your previous, unconstrained rigid body integrator to handle contact using the projected Gauss-Seidel algorithm. The notes above assume the general case in which contact forces act on two objects which are both dynamic. In the assignment your contacts will happen with a fixed ground plane which cannot move. The ground plane will not be a simulated object. Rather you will modify your projected Gauss-Seidel solver to handle such fixed objects. One way to formulate this modification is to treat static objects as having infinite mass. In this way the inverse mass matrix is zero, meaning applied forces have no effect. If the objects initial velocity is zero, it will always be zero. 

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

