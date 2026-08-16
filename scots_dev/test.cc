/*
 * test/cpp (alteration of unicycle.cc)
 *
 *  created on: 21.01.2016
 *      author: rungger
 * 
 *  edited on: 03.04.2026
 *      edited by: suhit 
 */

/*
 * information about this example is given in the readme file
 *
 */

#include <array>
#include <iostream>

#include "cuddObj.hh"

#include "SymbolicSet.hh"
#include "SymbolicModelGrowthBound.hh"

#include "TicToc.hh"
#include "RungeKutta4.hh"
#include "FixedPoint.hh"



/* state space dim */
// 3 space state dimensions (technically): x, y, yaw
#define statespace_dim 3

// 3 input dimensions: vx, vy, vyaw
#define input_dim 3

/* data types for the ode solver */
typedef std::array<double,3> state_t;
typedef std::array<double,3> input_t;

/* sampling time */
const double tau = 0.3;
/* number of intermediate steps in the ode solver */
const int nint=5;

// From Runge Kutta 4
OdeSolver ode_solver(statespace_dim,nint,tau);

/* we integrate the dog ode by 0.3 sec (the result is stored in x)  */
// lambda expression function that redefines the states and inputs you pass in by a step
auto  go2_post = [](state_t &x, input_t &u) -> void {

  /* the ode describing the go2's response to controller inputs */
  /* first order, autonomous */
  auto rhs =[](state_t &xx,  const state_t &x, input_t &u) -> void {


    // xx[0] = u[0];
    // xx[1] = u[1];
    // xx[2] = u[2];


    //dx/dt (positive x towards computing server)
    xx[0] = u[0]*std::cos(x[2]) - u[1]*std::sin(x[2]);
    // u[1]*std::cos(x[2]+(3.14/2));

    //dy/dt (positive y towards other room, right handed coord plane)
    xx[1] = u[0]*std::sin(x[2]) + u[1]*std::cos(x[2]);
    // u[1]*std::sin(x[2]+(3.14/2));
    
    //dyaw/dt (0 at positive x; counterclockwise -pi to pi)
    xx[2] = u[2];
  };

  // Runge Kutta solve the diffeq above a single step
  ode_solver(rhs,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_t &r, input_t &u) -> void {
    //x
    r[0] = r[0];
    // +r[2]*std::abs(u[0])*0.3;
    //y
    r[1] = r[1];
    // +r[2]*std::abs(u[0])*0.3;
    //yaw
    r[2] = r[2];

};


/* forward declaration of the functions to setup the state space 
 * and input space of the unicycle example */
// scots::SymbolicSet unicycleCreateStateSpace(Cudd &mgr);
// scots::SymbolicSet unicycleCreateInputSpace(Cudd &mgr);

scots::SymbolicSet arenaCreateStateSpace(Cudd &mgr);
scots::SymbolicSet go2CreateInputSpace(Cudd &mgr);


int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
   std::cout << "2026-5-22" << std::endl;
   scots::SymbolicSet ss=arenaCreateStateSpace(mgr);
   ss.writeToFile("go2_ss.bdd");
  /* write SymbolicSet of obstacles to unicycle_obst.bdd */
   ss.complement();
   ss.writeToFile("go2_obst.bdd");
   ss.complement();
   std::cout << "Uniform grid details:" << std::endl;
   ss.printInfo(1);

  /****************************************************************************/
  /* the target set */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet ts = ss;
  /* define the target set as a symbolic set */
  /* The target set being an ellipsoid means there's wiggle room and each dimension is mutually dependent on one another. Ellipsoids can be more than 3 dimensions!*/


  /* Each half axis of the ellipsoid is defined as 1/H[i,i] */
  double H[9]={ 2, 0, 0, // x wiggle room
                0, 2, 0, // y wiggle room
                0, 0, .001}; // yaw wiggle room (very small, almost negligible)
  /* compute inner approximation of P={ x | H x<= h1 }  */

  /* target center is going to be at center of the room, facing 0 degrees */
  double c[3] = {-1.25, 0, 0};
  ts.addEllipsoid(H,c, scots::INNER);
  ts.writeToFile("go2_target.bdd");   


  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  scots::SymbolicSet is=go2CreateInputSpace(mgr);
  std::cout << std::endl << "Input space details:" << std::endl;
  is.printInfo(1);

  /****************************************************************************/
  /* setup class for symbolic model computation */
  /****************************************************************************/
  /* first create SymbolicSet of post variables 
   * by copying the SymbolicSet of the state space and assigning new BDD IDs */
  scots::SymbolicSet sspost(ss,1);  
  /* instantiate the SymbolicModel */
  scots::SymbolicModelGrowthBound<state_t,input_t> abstraction(&ss, &is, &sspost);
  /* compute the transition relation */
  tt.tic();
  abstraction.computeTransitionRelation(go2_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;


  /****************************************************************************/
  /* we continue with the controller synthesis */
  /****************************************************************************/
  int verbose=1;
  /* we setup a fixed point object to compute reachabilty controller */
  scots::FixedPoint fp(&abstraction);
  /* the fixed point algorithm operates on the BDD directly */
  BDD T = ts.getSymbolicSet();
  tt.tic();
  BDD C = fp.reach(T,verbose);
  tt.toc();

  /****************************************************************************/
  /* last we store the controller as a SymbolicSet 
   * the underlying uniform grid is given by the Cartesian product of 
   * the uniform gird of the space and uniform gird of the input space */
  /****************************************************************************/
  scots::SymbolicSet controller(ss,is);
  controller.setSymbolicSet(C);
  controller.writeToFile("go2_controller.bdd");

  scots::SymbolicSet tr = abstraction.getTransitionRelation();
  tr.writeToFile("go2_relation.bdd");


  
  return 1;
}

scots::SymbolicSet arenaCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[statespace_dim]={-1.5,-1.5,-3.14-0.4};  
  /* upper bounds of the hyper rectangle */
  double ub[statespace_dim]={1.5,1.5,3.14+0.4}; 
  /* grid node distance diameter */
  // should be 100 nodes for each interval
  
//finest
double eta[statespace_dim]={0.06, 0.06, 0.1256};
// double eta[statespace_dim]={0.12, 0.12, 0.2512};
// double eta[statespace_dim]={0.15, 0.15, 0.314};
// double eta[statespace_dim]={0.30, 0.30, 0.628};
//coarsest



  /* eta is added to the bound so as to ensure that the whole
   * [0,10]x[0,10]x[-pi-eta,pi+eta] is covered by the cells */

  scots::SymbolicSet state_space(mgr,statespace_dim, lb,ub,eta);

  /* add the grid points to the SymbolicSet ss */
  state_space.addGridPoints();
  /* remove the obstacles from the state space */
  /* the obstacles are defined as polytopes */
  /* define H* x <= h */
  double H[4*statespace_dim]={
    -1, 0, 0,
     1, 0, 0,
     0,-1, 0,
     0, 1, 0};
//   /* remove outer approximation of P={ x | H x<= h1 } form state space */
  double h1[4] = {-.25, .3, 0.8,-0.3};
  state_space.remPolytope(4,H,h1, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h2 } form state space */
  double h2[4] = {.3, -.25, -0.3,0.8};
  state_space.remPolytope(4,H,h2, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h3 } form state space */
//   double h3[4] = {-2.2,2.4,-6,10};
//   state_space.remPolytope(4,H,h3, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h4 } form state space */
//   double h4[4] = {-3.4,3.6,-0,9};
//   state_space.remPolytope(4,H,h4, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h5 } form state space */
//   double h5[4] = {-4.6 ,4.8,-1,10};
//   state_space.remPolytope(4,H,h5, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h6 } form state space */
//   double h6[4] = {-5.8,6,-0,6};
//   state_space.remPolytope(4,H,h6, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h7 } form state space */
//   double h7[4] = {-5.8,6,-7,10};
//   state_space.remPolytope(4,H,h7, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h8 } form state space */
//   double h8[4] = {-7,7.2,-1,10};
//   state_space.remPolytope(4,H,h8, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h9 } form state space */
//   double h9[4] = {-8.2,8.4,-0,8.5};
//   state_space.remPolytope(4,H,h9, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h10 } form state space */
//   double h10[4] = {-8.4,9.3,-8.3,8.5};
//   state_space.remPolytope(4,H,h10, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h11 } form state space */
//   double h11[4] = {-9.3,10,-7.1,7.3};
//   state_space.remPolytope(4,H,h11, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h12 } form state space */
//   double h12[4] = {-8.4,9.3,-5.9,6.1};
//   state_space.remPolytope(4,H,h12, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h13 } form state space */
//   double h13[4] = {-9.3,10 ,-4.7,4.9};
//   state_space.remPolytope(4,H,h13, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h14 } form state space */
//   double h14[4] = {-8.4,9.3,-3.5,3.7};
//   state_space.remPolytope(4,H,h14, scots::OUTER);
//   /* remove outer approximation of P={ x | H x<= h15 } form state space */
//   double h15[4] = {-9.3,10 ,-2.3,2.5};
//   state_space.remPolytope(4,H,h15, scots::OUTER);


 return state_space;
}

scots::SymbolicSet go2CreateInputSpace(Cudd &mgr) {
// vx: Range [-2.5~3.8] (m/s); vy: Range [-1.0~1.0] (m/s); vyaw: Range [-4~4] (rad/s).

  /* lower bounds of the hyper rectangle */
  double lb[input_dim]={-2.5/5,-1.0/5,-4.0/5};  
  /* upper bounds of the hyper rectangle */
  double ub[input_dim]={3.8/5, 1.0/5 ,4.0/5}; 
  /* grid node distance diameter */
 

  // finest
  // double eta[input_dim] = {0.063, 0.02, 0.08};
  // double eta[input_dim] = {0.126, 0.04, 0.16};
  // double eta[input_dim] = {0.252, 0.08, 0.32};
  double eta[input_dim] = {0.63, 0.2, 0.8};
  // coarsest


  scots::SymbolicSet input_space(mgr,input_dim, lb,ub,eta);
  input_space.addGridPoints();

  return input_space;
}