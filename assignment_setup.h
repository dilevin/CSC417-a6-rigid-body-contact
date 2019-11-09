#ifndef ASSIGNMENT_SETUP_H
#define ASSIGNMENT_SETUP_H
//A6 code
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <EigenTypes.h>

#include <visualization.h>
#include <init_state_rigid_bodies.h>
#include <dV_spring_particle_particle_dq.h>
#include <pick_nearest_vertices.h>

#include <inertia_matrix.h>
#include <rigid_body_jacobian.h>
#include <inverse_rigid_body.h>
#include <exponential_euler_lcp_contact.h>

//collision detection/resolution code
#include <collision_box_floor.h>

#define RB_OFFSET 12
#define RB_POS_OFFSET 9

//rigid body geometry
std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>> geometry;

//rigid body mass matrices
std::vector<Eigen::Matrix66d> mass_matrices;

//skinning matrix for rigid body (necesary for the way I interface with the libigl viewer)
Eigen::SparseMatrixd N;

double density = 1.0;

//some forces for the rigid bodies
Eigen::VectorXd forces; 

//floor collision geometry 
Eigen::Vector3d floor_normal;
Eigen::Vector3d floor_pos;

//collision info 
std::vector<Eigen::Vector3d> collision_normals;
std::vector<Eigen::Vector3d> collision_positions; 
std::vector<std::pair<int, int> > collision_objects; //normal points away from the first object

Eigen::VectorXd *q_ptr, *qdot_ptr;

bool simulation_pause = true;

//selection spring
double k_selected = 1e-1;

inline void simulate(Eigen::VectorXd &q, Eigen::VectorXd &qdot, double dt, double t) {  

    //Interaction spring
    Eigen::Vector3d mouse;
    Eigen::Vector6d dV_mouse;
    Eigen::Matrix36d rb_jacobian;

    //simulation step
    collision_normals.clear();
    collision_positions.clear();
    collision_objects.clear();

    Eigen::Vector3d gravity;
    gravity << 0., -0.98, 0.;

    forces.setZero();
    for(unsigned int irb=0; irb < geometry.size(); ++irb) {
        forces.segment<3>(6*irb + 3) = mass_matrices[irb].block(3,3,3,3)*gravity;
    }

    unsigned int irb = 0;
    for(unsigned int pickedi = 0; pickedi < Visualize::picked_vertices().size(); pickedi++) {   
        Eigen::Matrix3d R = Eigen::Map<const Eigen::Matrix3d>(q.segment<9>(12*irb).data());
        Eigen::Vector3d p = Eigen::Map<const Eigen::Vector3d>(q.segment<3>(12*irb + 9).data());
        Eigen::Vector3d x_body; 

        mouse = Visualize::geometry(irb).row(Visualize::picked_vertices()[pickedi]).transpose() + Visualize::mouse_drag_world() + Eigen::Vector3d::Constant(1e-6);
        dV_spring_particle_particle_dq(dV_mouse, mouse, Visualize::geometry(irb).row(Visualize::picked_vertices()[pickedi]).transpose(), 0.0, (Visualize::is_mouse_dragging() ? k_selected : 0.));
        
        //std::cout<<dV_mouse.transpose()<<"\n";
        inverse_rigid_body(x_body, Visualize::geometry(irb).row(Visualize::picked_vertices()[pickedi]).transpose(), R, p);
        rigid_body_jacobian(rb_jacobian, R, p, x_body);
        forces.segment<6>(6*irb) -= rb_jacobian.transpose()*dV_mouse.segment<3>(3);
    }

    for(unsigned int ii=0; ii<geometry.size(); ++ii) {
        //Get the rotation and position of this rigid body 
        Eigen::Matrix3d R = Eigen::Map<const Eigen::Matrix3d>(q.segment<9>(12*ii).data());
        Eigen::Vector3d p = q.segment<3>(12*ii + RB_POS_OFFSET);
        collision_box_floor(collision_positions, collision_normals, collision_objects, R, p, ii, geometry[ii].first, floor_normal, floor_pos);
    }
 
    if(!simulation_pause)
        exponential_euler_lcp_contact(q, qdot, dt, mass_matrices, forces, collision_normals, collision_positions, collision_objects);
    
}

inline void draw(Eigen::Ref<const Eigen::VectorXd> q, Eigen::Ref<const Eigen::VectorXd> qdot, double t) {

    //update vertex positions using simulation
    for(unsigned int irb=0; irb<geometry.size();++irb) {
        Eigen::Map<const Eigen::Matrix3d> R = Eigen::Map<const Eigen::Matrix3d>(q.segment<9>(RB_OFFSET*irb).data());
        Eigen::MatrixXd V_rb = R*(geometry[irb].first.transpose());
        
        for(unsigned int jj=0; jj<V_rb.cols(); ++jj) {
            V_rb.col(jj) += q.segment<3>(RB_OFFSET*irb + RB_POS_OFFSET);
        }
        Visualize::update_vertex_positions(irb, Eigen::Map<const Eigen::VectorXd>(V_rb.data(), 3*V_rb.cols()));
    }

}

bool key_down_callback(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifiers) {

    if(key == 'R') {
        //reset the simulation
        forces.setZero();
        init_state_rigid_bodies(*q_ptr, *qdot_ptr,geometry.size());
        qdot_ptr->segment<3>(0) << 0.1, 0.1, 0.0;
    } 

    if(key == 'S') {
        simulation_pause = !simulation_pause;
    }

    return false;
}

inline void assignment_setup(int argc, char **argv, Eigen::VectorXd &q, Eigen::VectorXd &qdot) {

    q_ptr = &q;
    qdot_ptr = &qdot; 

    //setup the floor 
    floor_normal << 0.7, 0.7, 0.;
    floor_pos << 0. , -0.1, 0.;    

    //load geometric data 
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOBJ("../data/torus.obj", V, F);

    std::cout<<F.cols()<<"\n";

    geometry.push_back(std::make_pair(V,F));

    //some forces to show the integrator works 
    forces.resize(6*geometry.size());
    forces.setZero();

    //setup simulation 
    init_state_rigid_bodies(q,qdot,geometry.size());

    Eigen::Matrix3d inertia;
    Eigen::Matrix66d mass_matrix;
    double mass = 0;
    Eigen::Vector3d com;
    Eigen::Vector3d gravity;
    gravity << 0., -0.98, 0.;

    for(unsigned int irb=0; irb < geometry.size(); ++irb) {
        inertia_matrix(inertia, com, mass, geometry[irb].first, geometry[irb].second, density);
        
        mass_matrix.setZero();
        mass_matrix.block(0,0,3,3) = inertia;
        mass_matrix(3,3) = mass;
        mass_matrix(4,4) = mass;
        mass_matrix(5,5) = mass;

        mass_matrices.push_back(mass_matrix);
        
        //inital angular velocity
        qdot.segment<3>(6*irb) << 0.1, 0.1, 0.0;

        //setup rigid bodies initial position in space 
        q.segment<3>(RB_OFFSET*irb + RB_POS_OFFSET) = com;

        forces.segment<3>(6*irb + 3) = mass_matrices[irb].block(3,3,3,3)*gravity;
        N.resize(geometry[irb].first.rows(), geometry[irb].first.rows());
        N.setIdentity();
        Visualize::add_object_to_scene(geometry[irb].first, geometry[irb].second, geometry[irb].first, geometry[irb].second, N,Eigen::RowVector3d(244,165,130)/255.);

        //fix up mesh
        for(unsigned int jj=0; jj<geometry[irb].first.rows(); ++jj) {
            geometry[irb].first.row(jj) -= com.transpose();
        }
    }

    //add floor
    Eigen::MatrixXd V_floor;
    Eigen::MatrixXi F_floor;
    igl::readOBJ("../data/plane.obj", V_floor, F_floor);

    N.resize(V_floor.rows(), V_floor.rows());
    N.setIdentity();

    //rotate plane
    Eigen::Vector3d n0;
    n0 << 0, 1, 0;
    floor_normal.normalize();
    float angle = std::acos(n0.dot(floor_normal)) + 0.15;
    Eigen::Vector3d axis = n0.cross(floor_normal);

    Eigen::Matrix3d floor_R = Eigen::AngleAxisd(angle, axis).matrix();

    //translate plane
    for(unsigned int iv=0; iv<V_floor.rows(); ++iv) {
        Eigen::Vector3d rotated = floor_R*V_floor.row(iv).transpose();
        V_floor.row(iv) = (rotated + floor_pos + Eigen::Vector3d(0., -0.01, 0.)).transpose();
    }    

    Visualize::add_object_to_scene(V_floor, F_floor, V_floor, F_floor, N,Eigen::RowVector3d(64,165,130)/255.);

    Visualize::viewer().callback_key_down = key_down_callback;
}

#endif

