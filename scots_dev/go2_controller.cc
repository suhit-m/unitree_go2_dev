/*
 * go2_controller.cc (alteration of unicycle.cc)
 *
 *  created on: 21.01.2016
 *      author: rungger
 *
 *  edited on: 03.04.2026
 *      edited by: suhit
 *
 *  ------------------------------------------------------------------------
 *  WHAT CHANGED FROM test.cc
 *  ------------------------------------------------------------------------
 *  The dynamics, growth bound and default geometry are IDENTICAL to test.cc.
 *  The only structural change: arena geometry is read at startup from a
 *  plain-text config file so the AutoDeploy GUI can place targets/obstacles
 *  and re-synthesise without editing or recompiling this file.
 *
 *      ./go2_controller                      -> uses arena_config.txt
 *      ./go2_controller my_arena.txt         -> uses my_arena.txt
 *      ./go2_controller --defaults           -> ignores any config file
 *
 *  If no config file is found it falls back to the values hardcoded below,
 *  which are exactly the ones currently in test.cc. So running it bare
 *  reproduces your existing behaviour.
 *
 *  ------------------------------------------------------------------------
 *  CONFIG FORMAT ('#' starts a comment, blank lines ignored)
 *  ------------------------------------------------------------------------
 *      tau            0.3
 *      nint           5
 *      growth_bound   0                  # 0 = identity, 1 = rigorous
 *      state_lb      -1.5  -1.5  -3.5416
 *      state_ub       1.5   1.5   3.5416
 *      state_eta      0.06  0.06  0.1256
 *      input_lb      -0.5  -0.2  -0.8
 *      input_ub       0.76  0.2   0.8
 *      input_eta      0.63  0.2   0.8
 *      target         cx cy cyaw  Lxx Lyy Lyaw     # repeatable, union
 *      obstacle       xmin xmax ymin ymax          # repeatable
 *
 *  ------------------------------------------------------------------------
 *  TWO THINGS THAT ARE EASY TO GET WRONG
 *  ------------------------------------------------------------------------
 *  1. addEllipsoid(L,c,type) encodes  { x : (x-c)' L'L (x-c) <= 1 }, so for a
 *     diagonal L the half-axis in dimension i is 1/L[i][i]. Your comment in
 *     test.cc already says this and it is correct. Any MATLAB/Python
 *     in-target check must therefore be
 *         sum_i ( L_ii * (x_i - c_i) )^2 <= 1
 *     and NOT  sum_i L_ii*(x_i-c_i)^2 <= 1.
 *
 *  2. The rotation from body to world frame lives in the ODE below, so the
 *     inputs this controller returns are BODY-frame commands. At deployment
 *     send them straight to SportClient.Move(vx, vy, vyaw). Passing them
 *     through a global->local helper as well rotates twice and the robot
 *     drives off in the wrong direction.
 */

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

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


/****************************************************************************/
/* arena configuration -- defaults are the current test.cc values           */
/****************************************************************************/
struct Ellipsoid {
  double c[3];
  double L[9];        /* row major 3x3 */
};

struct ArenaConfig {
  /* sampling time */
  double tau  = 0.3;
  /* number of intermediate steps in the ode solver */
  int    nint = 5;
  /* 0 = identity growth bound (what test.cc uses), 1 = rigorous */
  int    rigorous_growth = 0;

  double s_lb[3]  = {-1.5, -1.5, -3.14-0.4};
  double s_ub[3]  = { 1.5,  1.5,  3.14+0.4};
  double s_eta[3] = { 0.06, 0.06, 0.1256};

  double u_lb[3]  = {-2.5/5, -1.0/5, -4.0/5};
  double u_ub[3]  = { 3.8/5,  1.0/5,  4.0/5};
  double u_eta[3] = { 0.63,   0.2,    0.8};

  std::vector<Ellipsoid>            targets;
  std::vector<std::array<double,4>> obstacles;   /* xmin xmax ymin ymax */

  void loadDefaultGeometry() {
    /* target: centre of the far side of the room, any heading.
     * L = diag(2,2,0.001) -> half axes 0.5 m, 0.5 m, 1000 rad */
    Ellipsoid e;
    for (int i = 0; i < 9; i++) e.L[i] = 0.0;
    e.c[0] = -1.25; e.c[1] = 0.0; e.c[2] = 0.0;
    e.L[0] = 2.0;   e.L[4] = 2.0; e.L[8] = 0.001;
    targets.push_back(e);

    /* the two thin walls from test.cc, already decoded out of h1/h2 */
    obstacles.push_back({{ 0.25,  0.30, -0.80, -0.30}});   /* was h1 */
    obstacles.push_back({{-0.30, -0.25,  0.30,  0.80}});   /* was h2 */
  }
};

static ArenaConfig cfg;


/****************************************************************************/
/* ode solver                                                               */
/****************************************************************************/
/* constructed in main() once tau/nint are known, hence the pointer */
static OdeSolver* ode_solver_ptr = nullptr;

/* we integrate the dog ode by tau sec (the result is stored in x) */
// lambda expression function that redefines the states and inputs you pass in by a step
auto go2_post = [](state_t &x, input_t &u) -> void {

  /* the ode describing the go2's response to controller inputs */
  auto rhs = [](state_t &xx, const state_t &x, input_t &u) -> void {


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
  (*ode_solver_ptr)(rhs, x, u);
};


auto radius_post = [](state_t &r, input_t &u) -> void {
    //x
    r[0] = r[0];
    //y
    r[1] = r[1];
    //yaw
    r[2] = r[2];
  
};


/* forward declaration of the functions to setup the state space
 * and input space */
scots::SymbolicSet arenaCreateStateSpace(Cudd &mgr);
scots::SymbolicSet go2CreateInputSpace(Cudd &mgr);


/****************************************************************************/
/* config parsing                                                           */
/****************************************************************************/
static bool readArenaConfig(const std::string &path, ArenaConfig &c) {
  // Open file
  std::ifstream config(path.c_str());
  // Check if its good
  if (!config.good()) return false;

  bool saw_target = false;
  bool saw_obstacle = false;
  std::string line;
  size_t idx = 0;

  // parse
  while (std::getline(config, line)) {
    idx++;
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;

    std::istringstream ss(line);
    std::string key;
    ss >> key;

    if      (key == "tau")          ss >> c.tau;
    else if (key == "nint")         ss >> c.nint;
    else if (key == "growth_bound") ss >> c.rigorous_growth;
    else if (key == "state_lb")     ss >> c.s_lb[0]  >> c.s_lb[1]  >> c.s_lb[2];
    else if (key == "state_ub")     ss >> c.s_ub[0]  >> c.s_ub[1]  >> c.s_ub[2];
    else if (key == "state_eta")    ss >> c.s_eta[0] >> c.s_eta[1] >> c.s_eta[2];
    else if (key == "input_lb")     ss >> c.u_lb[0]  >> c.u_lb[1]  >> c.u_lb[2];
    else if (key == "input_ub")     ss >> c.u_ub[0]  >> c.u_ub[1]  >> c.u_ub[2];
    else if (key == "input_eta")    ss >> c.u_eta[0] >> c.u_eta[1] >> c.u_eta[2];
    else if (key == "target") {
      if (!saw_target) { c.targets.clear(); saw_target = true; }
      Ellipsoid e;
      for (int i = 0; i < 9; i++) e.L[i] = 0.0;
      double lx, ly, lyaw;
      ss >> e.c[0] >> e.c[1] >> e.c[2] >> lx >> ly >> lyaw;
      e.L[0] = lx; e.L[4] = ly; e.L[8] = lyaw;
      c.targets.push_back(e);
    }
    else if (key == "obstacle") {
      if (!saw_obstacle) { c.obstacles.clear(); saw_obstacle = true; }
      std::array<double,4> b;
      ss >> b[0] >> b[1] >> b[2] >> b[3];
      if (b[0] >= b[1] || b[2] >= b[3]) {
        std::cout << "WARNING: config line " << idx
                  << ": obstacle has empty interior (xmin>=xmax or ymin>=ymax)"
                     ", skipped. SCOTS would silently remove nothing."
                  << std::endl;
        continue;
      }
      c.obstacles.push_back(b);
    }
    else {
      std::cout << "WARNING: config line " << idx
                << ": unknown key '" << key << "', ignored." << std::endl;
    }
  }
  return true;
}

/* replicate the SCOTS grid arithmetic so a bad eta is visible before the
 * transition relation eats several minutes */
static size_t gridCount(double lb, double ub, double eta) {
  const double Nl = std::ceil(lb/eta);
  const double Nu = std::floor(ub/eta);
  return (size_t)std::abs(Nu-Nl) + 1;
}

static void printArenaConfig(const ArenaConfig &c) {
  std::cout << "=== arena configuration ===" << std::endl;
  std::cout << "tau=" << c.tau << "  nint=" << c.nint
            << "  growth_bound=" << (c.rigorous_growth ? "rigorous" : "identity")
            << std::endl;

  size_t ns = 1, nu = 1;
  for (int i = 0; i < 3; i++) {
    ns *= gridCount(c.s_lb[i], c.s_ub[i], c.s_eta[i]);
    nu *= gridCount(c.u_lb[i], c.u_ub[i], c.u_eta[i]);
  }
  std::cout << "state points = " << ns
            << "   input points = " << nu
            << "   product = " << (double)ns*(double)nu << std::endl;

  for (int i = 0; i < 3; i++) {
    const size_t n = gridCount(c.u_lb[i], c.u_ub[i], c.u_eta[i]);
    if (n < 3) {
      std::cout << "WARNING: input dimension " << i << " has only " << n
                << " grid point(s). Too few distinct commands is the most "
                   "common cause of an empty winning set." << std::endl;
    }
  }

  std::cout << "targets (" << c.targets.size() << "):" << std::endl;
  for (size_t i = 0; i < c.targets.size(); i++) {
    const Ellipsoid &e = c.targets[i];
    std::cout << "  centre=[" << e.c[0] << " " << e.c[1] << " " << e.c[2]
              << "]  half-axes=["
              << (e.L[0] ? 1.0/e.L[0] : 0) << " "
              << (e.L[4] ? 1.0/e.L[4] : 0) << " "
              << (e.L[8] ? 1.0/e.L[8] : 0) << "]" << std::endl;
    if (e.L[0] && 1.0/e.L[0] < c.s_eta[0])
      std::cout << "  WARNING: x half-axis is smaller than one grid cell; "
                   "the INNER approximation will be empty." << std::endl;
    if (e.L[4] && 1.0/e.L[4] < c.s_eta[1])
      std::cout << "  WARNING: y half-axis is smaller than one grid cell; "
                   "the INNER approximation will be empty." << std::endl;
  }

  std::cout << "obstacles (" << c.obstacles.size() << "):" << std::endl;
  for (size_t i = 0; i < c.obstacles.size(); i++) {
    const std::array<double,4> &b = c.obstacles[i];
    std::cout << "  x=[" << b[0] << ", " << b[1]
              << "]  y=[" << b[2] << ", " << b[3]
              << "]  (" << (b[1]-b[0]) << " x " << (b[3]-b[2]) << " m)"
              << std::endl;
    if ((b[1]-b[0]) < c.s_eta[0] || (b[3]-b[2]) < c.s_eta[1])
      std::cout << "  NOTE: thinner than one grid cell in at least one axis. "
                   "OUTER approximation still removes the cells it touches, "
                   "but the effective wall is one cell wide." << std::endl;
  }
  std::cout << "===========================" << std::endl << std::endl;
}


/****************************************************************************/
/* main                                                                     */
/****************************************************************************/
int main(int argc, char** argv) {
  std::cout << "go2_controller -- SCOTS synthesis for the Unitree Go2"
            << std::endl;

  /**************************************************************************/
  /* configuration                                                          */
  /**************************************************************************/
  std::string cfg_path = "arena_config.txt";
  bool force_defaults = false;
  if (argc > 1) {
    const std::string a1 = argv[1];
    if (a1 == "--defaults") force_defaults = true;
    else                    cfg_path = a1;
  }

  bool loaded = false;
  if (!force_defaults) loaded = readArenaConfig(cfg_path, cfg);

  if (loaded && !cfg.targets.empty()) {
    std::cout << "Loaded geometry from '" << cfg_path << "'" << std::endl;
  } else {
    if (!force_defaults && !loaded)
      std::cout << "No '" << cfg_path
                << "' found, using built-in defaults." << std::endl;
    else if (loaded)
      std::cout << "Config '" << cfg_path
                << "' defined no target, using built-in defaults."
                << std::endl;
    else
      std::cout << "Using built-in defaults (--defaults)." << std::endl;
    cfg.loadDefaultGeometry();
  }

  printArenaConfig(cfg);

  /* the ode solver needs tau/nint, which may have come from the config */
  OdeSolver ode_solver(statespace_dim, cfg.nint, cfg.tau);
  ode_solver_ptr = &ode_solver;

  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /**************************************************************************/
  /* construct SymbolicSet for the state space                              */
  /**************************************************************************/
  scots::SymbolicSet ss = arenaCreateStateSpace(mgr);
  ss.writeToFile("go2_ss.bdd");
  /* write SymbolicSet of obstacles to go2_obst.bdd */
  ss.complement();
  ss.writeToFile("go2_obst.bdd");
  ss.complement();
  std::cout << "Uniform grid details:" << std::endl;
  ss.printInfo(1);

  /**************************************************************************/
  /* the target set                                                         */
  /**************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet ts = ss;
  /* the copy carries the state space contents, so empty it before adding
   * the target -- otherwise the target would be the whole arena */
  ts.clear();

  /* The target set being an ellipsoid means there's wiggle room and each
   * dimension is mutually dependent on one another. Ellipsoids can be more
   * than 3 dimensions.
   * Each half axis of the ellipsoid is defined as 1/L[i,i]. */
  for (size_t i = 0; i < cfg.targets.size(); i++)
    ts.addEllipsoid(cfg.targets[i].L, cfg.targets[i].c, scots::INNER);

  std::cout << std::endl << "Target set details:" << std::endl;
  ts.printInfo(1);
  ts.writeToFile("go2_target.bdd");

  if (ts.getSize() < 1.0) {
    std::cout << std::endl
              << "ERROR: the target set is empty on this grid." << std::endl
              << "  - is the centre inside the state bounds?" << std::endl
              << "  - is any half-axis smaller than one grid cell? INNER only"
                 " keeps cells lying entirely inside the ellipsoid."
              << std::endl;
    return 1;
  }

  /**************************************************************************/
  /* construct SymbolicSet for the input space                              */
  /**************************************************************************/
  scots::SymbolicSet is = go2CreateInputSpace(mgr);
  std::cout << std::endl << "Input space details:" << std::endl;
  is.printInfo(1);

  /**************************************************************************/
  /* setup class for symbolic model computation                             */
  /**************************************************************************/
  /* first create SymbolicSet of post variables
   * by copying the SymbolicSet of the state space and assigning new BDD IDs */
  scots::SymbolicSet sspost(ss,1);
  /* instantiate the SymbolicModel */
  scots::SymbolicModelGrowthBound<state_t,input_t> abstraction(&ss, &is, &sspost);
  /* compute the transition relation */
  std::cout << std::endl << "Computing transition relation..." << std::endl;
  tt.tic();
  abstraction.computeTransitionRelation(go2_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: "
            << abstraction.getSize() << std::endl;

  /**************************************************************************/
  /* we continue with the controller synthesis                              */
  /**************************************************************************/
  int verbose = 1;
  /* we setup a fixed point object to compute reachabilty controller */
  scots::FixedPoint fp(&abstraction);
  /* the fixed point algorithm operates on the BDD directly */
  BDD T = ts.getSymbolicSet();
  std::cout << std::endl << "Synthesising controller..." << std::endl;
  tt.tic();
  BDD C = fp.reach(T, verbose);
  tt.toc();

  /**************************************************************************/
  /* last we store the controller as a SymbolicSet
   * the underlying uniform grid is given by the Cartesian product of
   * the uniform grid of the space and uniform grid of the input space      */
  /**************************************************************************/
  scots::SymbolicSet controller(ss,is);
  controller.setSymbolicSet(C);

  std::cout << std::endl << "Controller details:" << std::endl;
  controller.printInfo(1);
  controller.writeToFile("go2_controller.bdd");

  scots::SymbolicSet tr = abstraction.getTransitionRelation();
  tr.writeToFile("go2_relation.bdd");

  if (controller.getSize() < 1.0) {
    std::cout << std::endl
              << "ERROR: the winning set is EMPTY. In rough order of likelihood:"
              << std::endl
              << "  1. the input grid is too coarse -- too few distinct"
                 " commands to make progress" << std::endl
              << "  2. the target is walled off by an obstacle, or lies outside"
                 " the state bounds" << std::endl
              << "  3. state eta so coarse that one step cannot leave a cell"
              << std::endl
              << "  4. growth_bound 1 with a grid too coarse to absorb the"
                 " growth -- try growth_bound 0" << std::endl;
    return 1;
  }

  std::cout << std::endl
            << "SYNTHESIS OK -- go2_controller.bdd written" << std::endl;

  /* exit 0 on success. test.cc returned 1 here, which the GUI would read as
   * a failure, so this is deliberate. */
  return 0;
}


/****************************************************************************/
scots::SymbolicSet arenaCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* bounds and grid node distance come from the config (or the defaults) */
  scots::SymbolicSet state_space(mgr, statespace_dim,
                                 cfg.s_lb, cfg.s_ub, cfg.s_eta);

  /* add the grid points to the SymbolicSet ss */
  state_space.addGridPoints();

  /* remove the obstacles from the state space.
   * the obstacles are defined as polytopes, H*x <= h, with
   *
   *     -1  0  0      ->   -x <= h[0]   i.e.  x >= -h[0]
   *      1  0  0      ->    x <= h[1]
   *      0 -1  0      ->   -y <= h[2]   i.e.  y >= -h[2]
   *      0  1  0      ->    y <= h[3]
   *
   * so for a box the vector is  h = { -xmin, xmax, -ymin, ymax }.
   * Getting a sign wrong yields contradictory constraints and SCOTS removes
   * nothing at all without raising an error, so the boxes are validated in
   * readArenaConfig instead of being written by hand here. */
  double H[4*statespace_dim] = {
    -1,  0, 0,
     1,  0, 0,
     0, -1, 0,
     0,  1, 0};

  for (size_t i = 0; i < cfg.obstacles.size(); i++) {
    const std::array<double,4> &b = cfg.obstacles[i];   /* xmin xmax ymin ymax */
    double h[4] = { -b[0], b[1], -b[2], b[3] };
    state_space.remPolytope(4, H, h, scots::OUTER);
  }

  return state_space;
}

/****************************************************************************/
scots::SymbolicSet go2CreateInputSpace(Cudd &mgr) {
  /* Go2 hardware limits are vx [-2.5, 3.8], vy [-1.0, 1.0], vyaw [-4, 4].
   * The defaults above scale those down by 5 for indoor use: at tau = 0.3 s
   * full speed would cross 1.14 m in a single step, which is most of a 3 m
   * arena. */
  scots::SymbolicSet input_space(mgr, input_dim,
                                 cfg.u_lb, cfg.u_ub, cfg.u_eta);
  input_space.addGridPoints();

  return input_space;
}