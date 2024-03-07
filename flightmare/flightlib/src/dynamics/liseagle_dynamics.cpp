#include "flightlib/dynamics/liseagle_dynamics.hpp"

namespace flightlib {

LiseagleDynamics::LiseagleDynamics(const YAML::Node &cfg_node):cfg_(cfg_node) {
  // all the parameters are hard coded according to the agilicious drone
  // motor drag coefficient
  mass_ = 0.7;

  kappa_ = 0.016;
  S_ = 0.224;
  c_ = 0.23;
  b_ = 1.24;

  x_shift_ = 0.02; //0.01; 

  // translational matrix. from motor to body [x0, x1, x2, x3, y0, y1, y2, y3,
  // z0, z1, z2, z3]
  t_BM_ << 0.075, -0.075, -0.075, 0.075, -0.10, 0.10, -0.10, 0.10, 0.0, 0.0,
    0.0, 0.0;

  // motor speed limits
  motor_omega_min_ = 0.0;
  motor_omega_max_ = 500.0;

  // motor dynamics (how quickly do they react)
  motor_tau_inv_ = 4.250825892169585;
  Scalar motor_delay = 0.33390702538338135;

  // thrust mapping coefficients;
  thrust_map_ << 2.59045362e-05, 1.22321444e-05, 7.44440460e-06;

  // thrust limit
  thrust_min_ = 0.0;
  thrust_max_ = 5.9456;
  elevator_min_ = -20.0;
  elevator_max_ = 10.0;
  rudder_min_ = -20.0;
  rudder_max_ = 20.0;
  tail_sweep_min_ = 15.0; //10.0;
  tail_sweep_max_ =  60.0;
  left_sweep_min_ = 50.0; // 45.0;
  left_sweep_max_ = 125; // 130.0;
  right_sweep_min_ = 50; // 45.0;
  right_sweep_max_ = 125; //130.0;
  left_twist_min_ = -8.0;
  left_twist_max_ = 10.0;
  right_twist_min_ = -8.0;
  right_twist_max_ = 10.0;

  //
  collective_thrust_min_ = thrust_min_ / mass_;
  collective_thrust_max_ = thrust_max_ / mass_;

  // Offsets of the actuators are suppost to be zero
  motor_o_ = 0;
  servo_o_ele_ = 0;
  servo_o_rud_ = 0;  
  servo_o_tswe_ = 0;
  servo_o_lswe_ = 0.0;
  servo_o_rswe_ = 0.0;
  servo_o_ltwi_ = 0.0;
  servo_o_rtwi_ = 0.0;

  // Following data is fit to the KST X08 V6.0 Servo 8mm HV servo
  servo_c_ele_ = 0.9192529270938236;
  servo_c_rud_ = 0.948077074121983;
  servo_c_tswe_ = 4.773505913801708;
  servo_c_lswe_ = 1.2112645433848217;
  servo_c_rswe_ = 1.2112645433848217;
  servo_c_ltwi_ = 0.9192529271611521;
  servo_c_rtwi_ = 0.9192529271611521;

  // body rate max
  omega_max_ << 2.1, 1.5, 1.5;
  velu_max_ = 16.0;

  // allocation matrix (mapping from motor speeds to body torque)
  B_allocation_ =
    (Matrix<4, 4>() << Vector<4>::Ones().transpose(), t_BM_.row(1),
     -t_BM_.row(0), kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose())
      .finished();

  // reading in all coefficients from csv files
  std::string path = getenv("FLIGHTMARE_PATH") +
                     std::string("/flightlib/src/dynamics/liseagle_csv/");

  // aerodynamic constants
  readCsv(path + "xdlw.csv", xdlw_);
  readCsv(path + "ydlw.csv", ydlw_);
  readCsv(path + "zdlw.csv", zdlw_);
  readCsv(path + "ldlw.csv", ldlw_);
  readCsv(path + "mdlw.csv", mdlw_);
  readCsv(path + "ndlw.csv", ndlw_);
  readCsv(path + "xdrw.csv", xdrw_);
  readCsv(path + "ydrw.csv", ydrw_);
  readCsv(path + "zdrw.csv", zdrw_);
  readCsv(path + "ldrw.csv", ldrw_);
  readCsv(path + "mdrw.csv", mdrw_);
  readCsv(path + "ndrw.csv", ndrw_);
  
  readCsv(path + "xdt.csv", xdt_);
  readCsv(path + "zdt.csv", zdt_);
  readCsv(path + "mdt.csv", mdt_);
  readCsv(path + "xa.csv", xa_);
  readCsv(path + "xb.csv", xb_);
  readCsv(path + "xdr.csv", xdr_);
  readCsv(path + "yb.csv", yb_);
  readCsv(path + "ydr.csv", ydr_);
  readCsv(path + "za.csv", za_);
  readCsv(path + "lb.csv", lb_);
  readCsv(path + "ldr.csv", ldr_);
  readCsv(path + "ma.csv", ma_);
  readCsv(path + "nb.csv", nb_);
  readCsv(path + "ndr.csv", ndr_);

  //readCsv(path + "xp.csv", xp_);
  readCsv(path + "yp.csv", yp_);
  readCsv(path + "lp.csv", lp_);
  readCsv(path + "np.csv", np_);
  //readCsv(path + "xq.csv", xq_);
  readCsv(path + "zq.csv", zq_);
  readCsv(path + "mq.csv", mq_);
  //readCsv(path + "xr.csv", xr_);
  readCsv(path + "yr.csv", yr_);
  readCsv(path + "lr.csv", lr_);
  readCsv(path + "nr.csv", nr_);

  readCsv(path + "aoa.csv", aoa_);
  //readCsv(path + "aoa_dyn.csv", aoa_dyn_);
  readCsv(path + "aos.csv", aos_);

  readCsv(path + "elevator_deflection.csv", elevator_deflection_);
  readCsv(path + "rudder_deflection.csv", rudder_deflection_);
  readCsv(path + "tail_sweep.csv", tail_sweep_);
  readCsv(path + "wing_sweep.csv", wing_sweep_);
  readCsv(path + "wing_twist.csv", wing_twist_);

  readCsv(path + "lplw.csv", lplw_);
  readCsv(path + "lprw.csv", lprw_);
  readCsv(path + "zqt.csv", zqt_);
  readCsv(path + "zqlw.csv", zqlw_);
  readCsv(path + "zqrw.csv", zqrw_);
  readCsv(path + "mqt.csv", mqt_);
  readCsv(path + "mqlw.csv", mqlw_);
  readCsv(path + "mqrw.csv", mqrw_);
  readCsv(path + "lrlw.csv", lrlw_);
  readCsv(path + "lrrw.csv", lrrw_);
  readCsv(path + "nrlw.csv", nrlw_);
  readCsv(path + "nrrw.csv", nrrw_);

  readCsv(path + "I_base.csv", J_base_);
  readCsv(path + "I_dlw.csv", J_dlw_);
  readCsv(path + "I_drw.csv", J_drw_);

  readCsv(path + "cg_dlw.csv", cg_dlw_);
  readCsv(path + "cg_drw.csv", cg_drw_);

  // To correct dynamic coefficients
  scale_star_(0) = 1.0/4.3;
  scale_star_(1) = 1.0/1.7;
  scale_star_(2) = 1.0/10.5;

  // Helper
  J_change_ << 1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -1.0, -1.0, 1.0;
  J_base_ = J_base_.cwiseProduct(J_change_);

  // actuators
  motor_ptr_ = std::make_shared<Motor>(thrust_map_r_, motor_o_r_, motor_tau_inv_, motor_delay, thrust_min_, thrust_max_, motor_omega_min_, motor_omega_max_);
  servo_ptr_ele_ = std::make_shared<Servo>(servo_c_ele_r_, servo_o_ele_r_, elevator_min_, elevator_max_);
  servo_ptr_rud_ = std::make_shared<Servo>(servo_c_rud_r_, servo_o_rud_r_, rudder_min_, rudder_max_);
  servo_ptr_tswe_ = std::make_shared<Servo>(servo_c_tswe_r_, servo_o_tswe_r_, tail_sweep_min_, tail_sweep_max_);
  servo_ptr_lswe_ = std::make_shared<Servo>(servo_c_lswe_r_, servo_o_lswe_r_, left_sweep_min_, left_sweep_max_);
  servo_ptr_rswe_ = std::make_shared<Servo>(servo_c_rswe_r_, servo_o_rswe_r_, right_sweep_min_, right_sweep_max_);
  servo_ptr_ltwi_ = std::make_shared<Servo>(servo_c_ltwi_r_, servo_o_ltwi_r_, left_twist_min_, left_twist_max_);
  servo_ptr_rtwi_ = std::make_shared<Servo>(servo_c_rtwi_r_, servo_o_rtwi_r_, right_twist_min_, right_twist_max_);

  // Adapt parameters based on fit_model
  if (cfg_["robot"]["fit_model"].as<bool>()){
    readCsv(path + "fitmodel.csv", params_);
    loadModelParams(params_);
  }

  // Update the *_r parameters and reset actuators
  reset();
}

LiseagleDynamics::~LiseagleDynamics() {}

bool LiseagleDynamics::dState(const Ref<const Vector<RobotState::SIZE>> state,
                              Ref<Vector<RobotState::SIZE>> dstate) const {
  if (!state.segment<STATE::NDYM>(0).allFinite()) return false;

  dstate.setZero();
  //
  const Vector<3> omega(state(STATE::OMEX), state(STATE::OMEY),
                        state(STATE::OMEZ));
  const Quaternion q_omega(0, omega.x(), omega.y(), omega.z());
  const Quaternion qx(state(STATE::ATTW), state(STATE::ATTX),
                      state(STATE::ATTY), state(STATE::ATTZ));

  // linear velocity = dx / dt
  dstate.segment<STATE::NPOS>(STATE::POS) =
    qx.toRotationMatrix() * state.segment<STATE::NVEL>(STATE::VEL);

  // differentiate quaternion = dq / dt
  dstate.segment<STATE::NATT>(STATE::ATT) =
    0.5 * Q_right(q_omega) * state.segment<STATE::NATT>(STATE::ATT);

  // linear acceleration = dv / dt
  dstate.segment<STATE::NVEL>(STATE::VEL) =
    state.segment<STATE::NACC>(STATE::ACC);

  // angular accleration = domega / dt
  dstate.segment<STATE::NOME>(STATE::OME) =
    state.segment<STATE::NAAC>(STATE::AAC);
  
  return true;
}

LiseagleDynamics::DynamicsFunction LiseagleDynamics::getDynamicsFunction()
  const {
  return std::bind(
    static_cast<bool (LiseagleDynamics::*)(const Ref<const Vector<STATE::SIZE>>,
                                           Ref<Vector<STATE::SIZE>>) const>(
      &LiseagleDynamics::dState),
    this, std::placeholders::_1, std::placeholders::_2);
}

Vector<7> LiseagleDynamics::getAccelerations(RobotState state,
                                             const Vector<Dynamic>& cmd_ll,
                                             const Scalar sim_dt,
                                             Vector<6> wind_curl, bool direct) {
  //Vector<3> pos = state.rotEuclWorldToNEDWorldFrame(state.p);  // position in inertial frame North East Down (NED)
  Vector<3> wind = wind_curl.segment<3>(0);  // wind in world frame
  Vector<3> curl = wind_curl.segment<3>(3);  // curl in world frame

  Vector<3> vel =
    state.rotEuclBodyToFRDBodyFrame(state.v) -
    state.rotEuclWorldToFRDBodyFrame(wind);  // velocity in body frame
  Scalar u = vel(0);                         // forward
  Scalar v = vel(1);                         // right
  Scalar w = vel(2);                         // down

  // Matrix<3, 3> R_ib = state.rotEul(M_PI, 0, 0) * state.R() * state.rotEul(M_PI, 0, 0); //the following is nummerically better
  Matrix<3, 3> R_rot;
  R_rot << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  Matrix<3, 3> R_ib = R_rot * state.R() * R_rot;

  Vector<3> omega = state.rotEuclBodyToFRDBodyFrame(
    state.w) - state.rotEuclWorldToFRDBodyFrame(curl);           // angular velocity in body frame (FRD)
  
  Scalar p = rad2deg(omega(0));  // around x
  Scalar q = rad2deg(omega(1));  // around y
  Scalar r = rad2deg(omega(2));  // around z

  // Aerodynamic parameters
  Scalar V = sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2));  // velocity norm
  Scalar alpha = rad2deg(atan(w / u)); // angle of attack
  Scalar beta = rad2deg(asin(v / V)); // angle of sideslip

  // input
  Scalar thrust;
  Scalar elevator;
  Scalar rudder;
  Scalar tail_sweep;
  Scalar left_sweep;
  Scalar right_sweep;
  Scalar left_twist;
  Scalar right_twist;
  if (direct) {
    thrust = cmd_ll(0);
    elevator = cmd_ll(1);
    rudder = cmd_ll(2);
    tail_sweep = cmd_ll(3);
    left_sweep = cmd_ll(4);
    right_sweep = cmd_ll(5);
    left_twist = cmd_ll(6);
    right_twist = cmd_ll(7);
  } else{
    thrust = motor_ptr_->run(sim_dt,cmd_ll(0),u);
    elevator = servo_ptr_ele_->run(sim_dt,cmd_ll(1)); //negative: upward deflection, positive: downward deflection
    rudder = servo_ptr_rud_->run(sim_dt,cmd_ll(2)); //positive: left deflection, negative: right deflection
    tail_sweep = servo_ptr_tswe_->run(sim_dt,cmd_ll(3)); //fully extended
    left_sweep = servo_ptr_lswe_->run(sim_dt,cmd_ll(4)); //fully extended
    right_sweep = servo_ptr_rswe_->run(sim_dt,cmd_ll(5)); //fully extended
    left_twist = servo_ptr_ltwi_->run(sim_dt,cmd_ll(6)); //positive: leading edge up
    right_twist = servo_ptr_rtwi_->run(sim_dt,cmd_ll(7)); //positive: leading edge up
  }

  // Adjusting moment of inertia
  Matrix<3, 3> J_lw =
    interp1D(wing_sweep_, left_sweep, J_dlw_r_).cwiseProduct(J_change_);
  Matrix<3, 3> J_rw =
    interp1D(wing_sweep_, right_sweep, J_drw_r_).cwiseProduct(J_change_);

  Matrix<3, 3> J = J_base_r_ + J_lw + J_rw;

  // adjust moment of inertia to new weight (initally measured at 0.6kg)
  J = J * mass_r_ / 0.6;
  
  // Aerodynamic forces calculations
  // Non-dimensionalized rates
  Scalar p_star = 2 * b_r_ / V * p;
  Scalar q_star = 2 * c_r_ / V * q;
  Scalar r_star = 2 * b_r_ / V * r;

  // quickfix because the dynamic values are too big
  p_star *= scale_star_r_(0);
  q_star *= scale_star_r_(1);
  r_star *= scale_star_r_(2);

  // Aerodynamic coefficients
  // Coefficient in X
  // Define parameters
  Scalar Cx_a = interp1D(aoa_, alpha, xa_);
  Scalar Cx_b;

  if (abs(beta) <= 10.0) {
    Cx_b = interp2D(aoa_, aos_, alpha, beta, xb_);
  } else {
    Cx_b = interp2D(aoa_, aos_, alpha, sign(beta) * 10.0, xb_);
  }

  Scalar Cx_tail =
    interp3D(aoa_, tail_sweep_, elevator_deflection_, alpha, tail_sweep, elevator, xdt_);
  Scalar Cx_rud = interp2D(aoa_, rudder_deflection_, alpha, rudder, xdr_);
  Scalar Cx_lw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, left_sweep, left_twist, xdlw_);
  Scalar Cx_rw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, right_sweep, right_twist, xdrw_);

  // X  superposition
  // Scalar Cx = Cx_a + Cx_b + Cx_tail + Cx_rud + Cx_lw + Cx_rw;
  Scalar Cx = Cx_a + -abs(Cx_b) + Cx_tail + Cx_rud + (Cx_lw + Cx_rw); //the -abs() is nescessary for symmetry //the brackets are there for computational symmetricity

  // Coefficient in Y
  // static coefficients
  Scalar Cy_b = interp2D(aoa_, aos_, alpha, beta, yb_);
  Scalar Cy_rud = interp2D(aoa_, rudder_deflection_, alpha, rudder, ydr_);
  Scalar Cy_lw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, left_sweep, left_twist, ydlw_);
  Scalar Cy_rw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, right_sweep, right_twist, ydrw_);

  // dynamic coefficients
  Scalar Cy_p = yp_ * p_star;
  Scalar Cy_r = yr_ * r_star; 

  // Y  superposition
  Scalar Cy = Cy_b + Cy_p + Cy_r + Cy_rud + (Cy_lw + Cy_rw); //the brackets are there for computational symmetricity

  // Coefficient in Z
  // static coefficients
  Scalar Cz_a = interp1D(aoa_, alpha, za_);
  Scalar Cz_tail =
    interp3D(aoa_, tail_sweep_, elevator_deflection_, alpha, tail_sweep, elevator, zdt_);
  Scalar Cz_lw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, left_sweep, left_twist, zdlw_);
  Scalar Cz_rw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, right_sweep, right_twist, zdrw_);

  // dynamic coefficients
  Scalar Cz_q = zq_ * q_star;
  Scalar Cz_qt = interp1D(tail_sweep_, tail_sweep, zqt_) *
                q_star; 
  Scalar Cz_qlw = interp1D(wing_sweep_, left_sweep, zqlw_) *
                  q_star; 
  Scalar Cz_qrw = interp1D(wing_sweep_, right_sweep, zqrw_) *
                  q_star; 

  // Z  superposition
  Scalar Cz = Cz_a + Cz_q + Cz_tail + Cz_qt + (Cz_lw + Cz_rw) + (Cz_qlw + Cz_qrw); //the brackets are there for computational symmetricity

  // Coefficient in L
  // static coefficients
  Scalar Cl_b;
  if (abs(beta) <= 10.0) {
    Cl_b = interp2D(aoa_, aos_, alpha, beta, lb_);
  } else {
    Cl_b = interp2D(aoa_, aos_, alpha, sign(beta) * 10, lb_);
  }

  Scalar Cl_rud = interp2D(aoa_, rudder_deflection_, alpha, rudder, ldr_);
  Scalar Cl_lw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, left_sweep, left_twist, ldlw_);
  Scalar Cl_rw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, right_sweep, right_twist, ldrw_);
  
  Scalar Cl_p = lp_ * p_star;
  Scalar Cl_r = lr_ * r_star;
  Scalar Cl_plw = interp1D(wing_sweep_, left_sweep, lplw_) * p_star; 
  Scalar Cl_prw = interp1D(wing_sweep_, right_sweep, lprw_) * p_star; 
  Scalar Cl_rlw = interp1D(wing_sweep_, left_sweep, lrlw_) * r_star; 
  Scalar Cl_rrw = interp1D(wing_sweep_, right_sweep, lrrw_) * r_star;

  // L  superposition
  Scalar Cl = Cl_b + Cl_p + Cl_r + Cl_rud + (Cl_lw + Cl_rw) + (Cl_plw + Cl_prw) +
              (Cl_rlw + Cl_rrw); //the brackets are there for computational symmetricity

  // Coefficient in M
  // static coefficients
  Scalar Cm_a = interp1D(aoa_, alpha, ma_) + 0.03; 
  Scalar Cm_tail =
    interp3D(aoa_, tail_sweep_, elevator_deflection_, alpha, tail_sweep, elevator, mdt_);
  Scalar Cm_lw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, left_sweep, left_twist, mdlw_);
  Scalar Cm_rw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, right_sweep, right_twist, mdrw_);

  // Dynamic coefficients
  Scalar Cm_q = mq_ * q_star; 
  Scalar Cm_qt = interp1D(tail_sweep_, tail_sweep, mqt_) *
                q_star; 
  Scalar Cm_qlw = interp1D(wing_sweep_, left_sweep, mqlw_) *
                  q_star; 
  Scalar Cm_qrw = interp1D(wing_sweep_, right_sweep, mqrw_) *
                  q_star; 

  // M  superposition
  Scalar Cm = Cm_a + Cm_q + Cm_tail + (Cm_lw + Cm_rw) + Cm_qt + (Cm_qlw + Cm_qrw); //the brackets are there for computational symmetricity

  // Coefficient in N
  // Define parameters
  Scalar Cn_b;
  if (abs(beta) <= 10.0) {
    Cn_b = interp2D(aoa_, aos_, alpha, beta, nb_);
  } else {
    // Find min and argmin
    Scalar aoa_now = std::numeric_limits<Scalar>::max();
    int aoa_idx = 0;
    for (int i = 0; i < aoa_.size(); i++) {
      if (abs(aoa_(i) - alpha) < aoa_now) {
        aoa_now = abs(aoa_(i) - alpha);
        aoa_idx = i;
      }
    }
    // Cn_b = fitLinEval(aos_.block(2, 0, 3, 1),
    //                   nb_.block(aoa_idx, 0, 1, 3).transpose(), beta);
    //the abs() and this sign() are nescessary for symmetricity
    Cn_b = fitLinEval(aos_.block(2, 0, 3, 1),
                      nb_.block(aoa_idx, 0, 1, 3).transpose(), abs(beta));
    Cn_b *= sign(beta);
  }

  Scalar Cn_rud = interp2D(aoa_, rudder_deflection_, alpha, rudder, ndr_);
  Scalar Cn_lw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, left_sweep, left_twist, ndlw_) * 0.05;
  Scalar Cn_rw =
    interp3D(aoa_, wing_sweep_, wing_twist_, alpha, right_sweep, right_twist, ndrw_) * 0.05;

  // Dynamic coefficients
  Scalar Cn_p = -np_ * p_star;
  Scalar Cn_r = nr_ * r_star * 5;

  Scalar Cn_rlw = interp1D(wing_sweep_, left_sweep, nrlw_) *
                  r_star; 
  Scalar Cn_rrw = interp1D(wing_sweep_, right_sweep, nrrw_) *
                  r_star; 

  // N  superposition
  Scalar Cn = Cn_b + Cn_p + Cn_r + Cn_rud + (Cn_lw + Cn_rw) + (Cn_rlw + Cn_rrw); //the brackets are there for computational symmetricity

  // Aerodynamic forces
  Scalar X = 0.5 * Rho * S_r_ * pow(V, 2) * Cx;  // N, from cg towards the nose
  Scalar Y =
    0.5 * Rho * S_r_ * pow(V, 2) * Cy;  // N, from cg towards the right wing tip
  Scalar Z = 0.5 * Rho * S_r_ * pow(V, 2) * Cz;  // N, from cg downward

  Scalar L = 0.5 * Rho * S_r_ * pow(V, 2) * b_r_ *
            Cl;  // Nm, around x
  Scalar M = 0.5 * Rho * S_r_ * pow(V, 2) * c_r_ *
            Cm;  // Nm, around y
  Scalar N = 0.5 * Rho * S_r_ * pow(V, 2) * b_r_ *
            Cn;  // Nm, around z

  // cg shift in x direction
  Scalar cg_xlw = interp1D(wing_sweep_, left_sweep, cg_dlw_.col(0));
  Scalar cg_xrw = interp1D(wing_sweep_, right_sweep, cg_drw_.col(0));
  Scalar cg_x = cg_xlw + cg_xrw + x_shift_r_;

  // cg shift in y direction
  Scalar cg_ylw = interp1D(wing_sweep_, left_sweep, cg_dlw_.col(1));
  Scalar cg_yrw = interp1D(wing_sweep_, right_sweep, cg_drw_.col(1));
  Scalar cg_y = cg_ylw + cg_yrw;

  // cg shift in z direction
  Scalar cg_zlw = interp1D(wing_sweep_, left_sweep, cg_dlw_.col(2));
  Scalar cg_zrw = interp1D(wing_sweep_, right_sweep, cg_drw_.col(2));
  Scalar cg_z = cg_zlw + cg_zrw;

  // if consider cg shift in x y z direction
  L = L - Z * cg_y + Y * cg_z;
  M = M + Z * cg_x - X * cg_z;
  N = N - Y * cg_x + X * cg_y;

  // resulting forces (aerodynamic, weight, and propulsive)
  // - the weight force is given in the inertial frame and must be translated
  //   into the body fixed frame by the R_inertia_to_body matrix.
  // - thrust acts along the x-axis
  Vector<3> f_xyz{X + thrust, Y, Z};
  Vector<3> gravity_vec = {0.0, 0.0,
                          mass_r_ * Gz * -1.0};  // because this is FRD frame
  f_xyz += R_ib.transpose() * gravity_vec;

  // moment vector in the body frame
  Vector<3> LMN{L, M, N};

  // Body fixed accelerations
  // see Small Unmanned Aircraft, Beard et al., 2012, p.36
  Vector<3> vel_dot = (1 / mass_r_) * f_xyz - omega.cross(vel);

  // Pitch acceleration
  // Euler's equation (rigid body dynamics)
  // https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
  Vector<3> vec_omega;
  vec_omega = J * omega;
  Vector<3> omega_dot =
    J.colPivHouseholderQr().solve(LMN - omega.cross(J * omega));

  Vector<7> accelerations = Vector<7>::Zero();
  accelerations.segment<3>(0) = state.rotFRDBodyToEuclBodyFrame(vel_dot);
  accelerations.segment<3>(3) = state.rotFRDBodyToEuclBodyFrame(omega_dot);

  // check if still within valid range of model
  Scalar limit = 1000.0;
  
  // if (abs(alpha) > 32.0 || abs(beta) > 50.0 || V < 6.0 || V > 20.0 || !accelerations.allFinite() || accelerations.minCoeff() < -limit || accelerations.maxCoeff() > limit){
  if (!accelerations.allFinite() || accelerations.minCoeff() < -limit || accelerations.maxCoeff() > limit){
    accelerations(6) = 1;
  }
  else{
    accelerations(6) = 0;
  }

  return accelerations;
}

bool LiseagleDynamics::valid() const {
  bool check = true;

  check &= mass_r_ > 0.0;
  check &= mass_r_ < 100.0;  // limit maximum mass
  check &= t_BM_.allFinite();
  check &= J_base_.allFinite();

  check &= motor_omega_min_ >= 0.0;
  check &= (motor_omega_max_ > motor_omega_min_);
  check &= motor_tau_inv_ > 0.0;

  check &= thrust_map_r_.allFinite();
  check &= kappa_ > 0.0;
  check &= thrust_min_ >= 0.0;
  check &= (thrust_max_ > thrust_min_);

  check &= (omega_max_.array() > 0).all();

  return check;
}

Vector<3> LiseagleDynamics::clampTorque(const Vector<3>& torque) const {
  return torque.cwiseMax(force_torque_min_.segment<3>(1))
    .cwiseMin(force_torque_max_.segment<3>(1));
}

Scalar LiseagleDynamics::clampCollectiveThrust(const Scalar thrust) const {
  return std::clamp(thrust, collective_thrust_min_, collective_thrust_max_);
}

Vector<Dynamic> LiseagleDynamics::clampRaw(const Vector<Dynamic> raw) const {
  Vector<8> thrust_c;
  thrust_c(0) = std::clamp(raw(0), thrust_min_, thrust_max_);
  thrust_c(1) = std::clamp(raw(1), elevator_min_, elevator_max_);
  thrust_c(2) = std::clamp(raw(2), rudder_min_, rudder_max_);
  thrust_c(3) = std::clamp(raw(3), tail_sweep_min_, tail_sweep_max_);
  thrust_c(4) = std::clamp(raw(4), left_sweep_min_, left_sweep_max_);
  thrust_c(5) = std::clamp(raw(5), right_sweep_min_, right_sweep_max_);
  thrust_c(6) = std::clamp(raw(6), left_twist_min_, left_twist_max_);
  thrust_c(7) = std::clamp(raw(7), right_twist_min_, right_twist_max_);

  return thrust_c;
}

Vector<Dynamic> LiseagleDynamics::clampMotorOmega(
  const Vector<Dynamic>& omega) const {
  return omega.cwiseMax(motor_omega_min_).cwiseMin(motor_omega_max_);
}

Vector<3> LiseagleDynamics::clampBodyrates(const Vector<3>& omega) const {
  return omega.cwiseMax(-omega_max_).cwiseMin(omega_max_);
}

Scalar LiseagleDynamics::clampVelu(const Scalar velu) const {
    return std::clamp(velu,0.0, velu_max_);
}

Matrix<4, 4> LiseagleDynamics::getAllocationMatrix(void) const {
  // compute column-wise cross product
  // tau_i = t_BM_i x F_i
  return B_allocation_;
}

Vector<Dynamic> LiseagleDynamics::getRawMean(void) const {
  Vector<8> mean;
  mean(0) = (thrust_max_ + thrust_min_) / 2;
  mean(1) = (elevator_max_ + elevator_min_) / 2;
  mean(2) = (rudder_max_ + rudder_min_) / 2;
  mean(3) = (tail_sweep_max_ + tail_sweep_min_) / 2;
  mean(4) = (left_sweep_max_ + left_sweep_min_) / 2;
  mean(5) = (right_sweep_max_ + right_sweep_min_) / 2;
  mean(6) = (left_twist_max_ + left_twist_min_) / 2;
  mean(7) = (right_twist_max_ + right_twist_min_) / 2;
  return mean;
}

Vector<Dynamic> LiseagleDynamics::getRawStd(void) const {
  Vector<8> std;
  std(0) = (thrust_max_ - thrust_min_) / 2;
  std(1) = (elevator_max_ - elevator_min_) / 2;
  std(2) = (rudder_max_ - rudder_min_) / 2;
  std(3) = (tail_sweep_max_ - tail_sweep_min_) / 2;
  std(4) = (left_sweep_max_ - left_sweep_min_) / 2;
  std(5) = (right_sweep_max_ - right_sweep_min_) / 2;
  std(6) = (left_twist_max_ - left_twist_min_) / 2;
  std(7) = (right_twist_max_ - right_twist_min_) / 2;
  return std;
}

Scalar LiseagleDynamics::getConsumption(void) const{
  Scalar consumption = 0;
  consumption += motor_ptr_->getConsumption();
  consumption += servo_ptr_ele_->getConsumption();
  consumption += servo_ptr_rud_->getConsumption();
  consumption += servo_ptr_tswe_->getConsumption();
  consumption += servo_ptr_lswe_->getConsumption();
  consumption += servo_ptr_rswe_->getConsumption();
  consumption += servo_ptr_ltwi_->getConsumption();
  consumption += servo_ptr_rtwi_->getConsumption();
  return consumption;
}

Vector<Dynamic> LiseagleDynamics::getActuator(void) const{
  // unity works in degrees
  Vector<8> actuators;
  actuators(0) = motor_ptr_->getState();
  actuators(1) = servo_ptr_ele_->getState();
  actuators(2) = servo_ptr_rud_->getState();
  actuators(3) = servo_ptr_tswe_->getState();
  actuators(4) = servo_ptr_lswe_->getState();
  actuators(5) = servo_ptr_rswe_->getState();
  actuators(6) = servo_ptr_ltwi_->getState();
  actuators(7) = servo_ptr_rtwi_->getState();
  return actuators;
}

bool LiseagleDynamics::setActuator(const Vector<Dynamic>& actuator){
  // getActuator works in degrees
  motor_ptr_->setState(actuator(0));
  servo_ptr_ele_->setState(actuator(1));
  servo_ptr_rud_->setState(actuator(2));
  servo_ptr_tswe_->setState(actuator(3));
  servo_ptr_lswe_->setState(actuator(4));
  servo_ptr_rswe_->setState(actuator(5));
  servo_ptr_ltwi_->setState(actuator(6));
  servo_ptr_rtwi_->setState(actuator(7));

  return true;
}

bool LiseagleDynamics::reset() {
  randomizeParams();

  motor_ptr_->reset(thrust_map_r_, motor_o_r_);
  
  servo_ptr_ele_->reset(servo_c_ele_r_, servo_o_ele_r_);
  servo_ptr_rud_->reset(servo_c_rud_r_, servo_o_rud_r_);
  servo_ptr_tswe_->reset(servo_c_tswe_r_, servo_o_tswe_r_);
  servo_ptr_lswe_->reset(servo_c_lswe_r_, servo_o_lswe_r_);
  servo_ptr_rswe_->reset(servo_c_rswe_r_, servo_o_rswe_r_);
  servo_ptr_ltwi_->reset(servo_c_ltwi_r_, servo_o_ltwi_r_);
  servo_ptr_rtwi_->reset(servo_c_rtwi_r_, servo_o_rtwi_r_);

  //If start_actuator is defined in the config file, set it accordingly
  try {motor_ptr_->setState(cfg_["start_actuator"]["actu_0"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_ele_->setState(cfg_["start_actuator"]["actu_1"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_rud_->setState(cfg_["start_actuator"]["actu_2"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_tswe_->setState(cfg_["start_actuator"]["actu_3"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_lswe_->setState(cfg_["start_actuator"]["actu_4"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_rswe_->setState(cfg_["start_actuator"]["actu_5"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_ltwi_->setState(cfg_["start_actuator"]["actu_6"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_rtwi_->setState(cfg_["start_actuator"]["actu_7"].as<Scalar>());}
  catch (...) {}

  return true;
}

bool LiseagleDynamics::randomizeParams(){
  thrust_map_r_ = randomize(thrust_map_);
  x_shift_r_ = randomize(x_shift_);
  mass_r_ = randomize(mass_);

  motor_o_r_ = randomizeOffset(motor_o_);
  servo_o_ele_r_ = randomizeOffset(servo_o_ele_);
  servo_o_rud_r_ = randomizeOffset(servo_o_rud_);
  servo_o_tswe_r_ = randomizeOffset(servo_o_tswe_);
  servo_o_lswe_r_ = randomizeOffset(servo_o_lswe_);
  servo_o_rswe_r_ = randomizeOffset(servo_o_rswe_);
  servo_o_ltwi_r_ = randomizeOffset(servo_o_ltwi_);
  servo_o_rtwi_r_ = randomizeOffset(servo_o_rtwi_);

  servo_c_ele_r_ = randomize(servo_c_ele_);
  servo_c_rud_r_ = randomize(servo_c_rud_);
  servo_c_tswe_r_ = randomize(servo_c_tswe_);
  servo_c_lswe_r_ = randomize(servo_c_lswe_);
  servo_c_rswe_r_ = randomize(servo_c_rswe_);
  servo_c_ltwi_r_ = randomize(servo_c_ltwi_);
  servo_c_rtwi_r_ = randomize(servo_c_rtwi_);

  scale_star_r_ = randomize(scale_star_);

  S_r_ = randomize(S_);
  c_r_ = randomize(c_);
  b_r_ = randomize(b_);
  J_base_r_ = randomize(J_base_);
  J_dlw_r_ = randomize(J_dlw_);
  J_drw_r_ = randomize(J_drw_);

  scale_star_r_ = randomize(scale_star_);

  return true;
}

bool LiseagleDynamics::setModelParams(const Vector<Dynamic>& params){
  param_idx_ = 0;

  x_shift_r_ = scale(x_shift_, params);
  mass_r_ = scale(mass_, params);

  motor_o_r_ = offset(motor_o_, params);
  servo_o_ele_r_ = offset(servo_o_ele_, params);
  servo_o_rud_r_ = offset(servo_o_rud_, params);
  servo_o_tswe_r_ = offset(servo_o_tswe_, params);
  servo_o_lswe_r_ = offset(servo_o_lswe_, params);
  servo_o_rswe_r_ = offset(servo_o_rswe_, params);
  servo_o_ltwi_r_ = offset(servo_o_ltwi_, params);
  servo_o_rtwi_r_ = offset(servo_o_rtwi_, params);

  scale_star_r_ = scale(scale_star_, params);

  S_r_ = scale(S_, params);
  c_r_ = scale(c_, params);
  b_r_ = scale(b_, params);
  J_base_r_ = scale(J_base_, params);

  if (param_idx_ != params.rows()){
    std::cout << "ERROR: setModelParams expected " << param_idx_ << " elements, but got " << params.rows() << " instead. \n";
    return false;
  }

  // needed to update the actuators
  motor_ptr_->reset(thrust_map_r_, motor_o_r_);
  servo_ptr_ele_->reset(servo_c_ele_r_, servo_o_ele_r_);
  servo_ptr_rud_->reset(servo_c_rud_r_, servo_o_rud_r_);
  servo_ptr_tswe_->reset(servo_c_tswe_r_, servo_o_tswe_r_);
  servo_ptr_lswe_->reset(servo_c_lswe_r_, servo_o_lswe_r_);
  servo_ptr_rswe_->reset(servo_c_rswe_r_, servo_o_rswe_r_);
  servo_ptr_ltwi_->reset(servo_c_ltwi_r_, servo_o_ltwi_r_);
  servo_ptr_rtwi_->reset(servo_c_rtwi_r_, servo_o_rtwi_r_);

  return true;
}

bool LiseagleDynamics::loadModelParams(const Vector<Dynamic>& params){
  param_idx_ = 0;

  x_shift_ = scale(x_shift_, params);
  mass_ = scale(mass_, params);

  motor_o_ = offset(motor_o_, params);
  servo_o_ele_ = offset(servo_o_ele_, params);
  servo_o_rud_ = offset(servo_o_rud_, params);
  servo_o_tswe_ = offset(servo_o_tswe_, params);
  servo_o_lswe_ = offset(servo_o_lswe_, params);
  servo_o_rswe_ = offset(servo_o_rswe_, params);
  servo_o_ltwi_ = offset(servo_o_ltwi_, params);
  servo_o_rtwi_ = offset(servo_o_rtwi_, params);

  scale_star_ = scale(scale_star_, params);

  S_ = scale(S_, params);
  c_ = scale(c_, params);
  b_ = scale(b_, params);
  J_base_ = scale(J_base_, params);

  if (param_idx_ != params.rows()){
    std::cout << "ERROR: loadModelParams expected " << param_idx_ << " elements, but got " << params.rows() << " instead. \n";
    return false;
  }

  return true;
}

Scalar LiseagleDynamics::interp1D(Vector<Dynamic> X, Scalar x_des,
                                  Vector<Dynamic> Y, std::string extrap) {
  if (extrap=="nn"){
    x_des = std::clamp(x_des,X(0),X(X.size()-1));
  }
  else if (extrap=="linear"){
    // in that case x_des remains untouched
  }
  else{
    std::cout << "Invalid extrapolation method in interpolation function \n";
  }

  int idx0 = closestIdxLow(X, x_des);
  int idx1 = idx0 + 1;
  
  Scalar x0 = X(idx0);
  Scalar x1 = X(idx1);
  Scalar y0;
  Scalar y1;

  y0 = Y(idx0);
  y1 = Y(idx1);

  Scalar result;
  result = (y0 * (x1 - x_des) + y1 * (x_des - x0)) / (x1 - x0);

  return result;
}

Matrix<Dynamic, Dynamic> LiseagleDynamics::interp1D(
  Vector<3> X, Scalar x_des,
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> Y, std::string extrap) {
  if (extrap=="nn"){
    x_des = std::clamp(x_des,X(0),X(X.size()-1));
  }
  else if (extrap=="linear"){
    // in that case x_des remains untouched
  }
  else{
    std::cout << "Invalid extrapolation method in interpolation function \n";
  }

  int idx0 = closestIdxLow(X, x_des);
  int idx1 = idx0 + 1;

  Scalar x0 = X(idx0);
  Scalar x1 = X(idx1);
  Scalar y0;
  Scalar y1;

  Matrix<3, 3> result;
  for (int i = 0; i < 9; i++) {
    y0 = Y(i / 3, i % 3, idx0);
    y1 = Y(i / 3, i % 3, idx1);
    result(i / 3, i % 3) = (y0 * (x1 - x_des) + y1 * (x_des - x0)) / (x1 - x0);
  }

  return result;
}

Scalar LiseagleDynamics::interp2D(Vector<Dynamic> X0, Vector<Dynamic> X1,
                                  Scalar x0_des, Scalar x1_des,
                                  Matrix<Dynamic, Dynamic> Y, std::string extrap) {
  
  if (extrap=="nn"){
    x0_des = std::clamp(x0_des,X0(0),X0(X0.size()-1));
    x1_des = std::clamp(x1_des,X1(0),X1(X1.size()-1));
  }
  else if (extrap=="linear"){
    // in that case x_des remains untouched
  }
  else{
    std::cout << "Invalid extrapolation method in interpolation function \n";
  }

  int idx00 = closestIdxLow(X0, x0_des);
  int idx01 = idx00 + 1;
  int idx10 = closestIdxLow(X1, x1_des);
  int idx11 = idx10 + 1;

  Scalar x00 = X0(idx00);
  Scalar x01 = X0(idx01);
  Scalar x10 = X1(idx10);
  Scalar x11 = X1(idx11);

  Scalar y00 = Y(idx00, idx10);
  Scalar y01 = Y(idx00, idx11);
  Scalar y10 = Y(idx01, idx10);
  Scalar y11 = Y(idx01, idx11);

  Matrix<1, 2> m_x0;
  m_x0 << x01 - x0_des, x0_des - x00;
  Matrix<2, 2> m_y;
  m_y << y00, y01, y10, y11;
  Matrix<2, 1> m_x1;
  m_x1 << x11 - x1_des, x1_des - x10;

  Scalar result;
  result = 1 / ((x01 - x00) * (x11 - x10)) * m_x0 * m_y * m_x1;

  return result;
}

Scalar LiseagleDynamics::interp3D(
  Vector<Dynamic> X0, Vector<Dynamic> X1, Vector<Dynamic> X2, Scalar x0_des,
  Scalar x1_des, Scalar x2_des,
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> Y, std::string extrap) {
  if (extrap=="nn"){
    x0_des = std::clamp(x0_des,X0(0),X0(X0.size()-1));
    x1_des = std::clamp(x1_des,X1(0),X1(X1.size()-1));
    x2_des = std::clamp(x2_des,X2(0),X2(X2.size()-1));
  }
  else if (extrap=="linear"){
    // in that case x_des remains untouched
  }
  else{
    std::cout << "Invalid extrapolation method in interpolation function \n";
  }

  int idx00 = closestIdxLow(X0, x0_des);
  int idx01 = idx00 + 1;
  int idx10 = closestIdxLow(X1, x1_des);
  int idx11 = idx10 + 1;
  int idx20 = closestIdxLow(X2, x2_des);
  int idx21 = idx20 + 1;

  Scalar x00 = X0(idx00);
  Scalar x01 = X0(idx01);
  Scalar x10 = X1(idx10);
  Scalar x11 = X1(idx11);
  Scalar x20 = X2(idx20);
  Scalar x21 = X2(idx21);

  Scalar y000 = Y(idx00, idx10, idx20);
  Scalar y001 = Y(idx00, idx10, idx21);
  Scalar y010 = Y(idx00, idx11, idx20);
  Scalar y011 = Y(idx00, idx11, idx21);
  Scalar y100 = Y(idx01, idx10, idx20);
  Scalar y101 = Y(idx01, idx10, idx21);
  Scalar y110 = Y(idx01, idx11, idx20);
  Scalar y111 = Y(idx01, idx11, idx21);

  Scalar x0_d = (x0_des - x00) / (x01 - x00);
  Scalar x1_d = (x1_des - x10) / (x11 - x10);
  Scalar x2_d = (x2_des - x20) / (x21 - x20);

  Scalar y00 = y000 * (1 - x0_d) + y100 * x0_d;
  Scalar y01 = y001 * (1 - x0_d) + y101 * x0_d;
  Scalar y10 = y010 * (1 - x0_d) + y110 * x0_d;
  Scalar y11 = y011 * (1 - x0_d) + y111 * x0_d;

  Scalar y0 = y00 * (1 - x1_d) + y10 * x1_d;
  Scalar y1 = y01 * (1 - x1_d) + y11 * x1_d;

  Scalar result = y0 * (1 - x2_d) + y1 * x2_d;

  return result;
}

Scalar LiseagleDynamics::interp3D(
  Vector<Dynamic> X0, Vector<Dynamic> X1, Vector<Dynamic> X2, Scalar x0_des,
  Scalar x1_des, Scalar x2_des,
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> Y, std::string extrap) {
  if (extrap=="nn"){
    x0_des = std::clamp(x0_des,X0(0),X0(X0.size()-1));
    x1_des = std::clamp(x1_des,X1(0),X1(X1.size()-1));
    x2_des = std::clamp(x2_des,X2(0),X2(X2.size()-1));
  }
  else if (extrap=="linear"){
    // in that case x_des remains untouched
  }
  else{
    std::cout << "Invalid extrapolation method in interpolation function \n";
  }

  int idx00 = closestIdxLow(X0, x0_des);
  int idx01 = idx00 + 1;
  int idx10 = closestIdxLow(X1, x1_des);
  int idx11 = idx10 + 1;
  int idx20 = closestIdxLow(X2, x2_des);
  int idx21 = idx20 + 1;

  Scalar x00 = X0(idx00);
  Scalar x01 = X0(idx01);
  Scalar x10 = X1(idx10);
  Scalar x11 = X1(idx11);
  Scalar x20 = X2(idx20);
  Scalar x21 = X2(idx21);

  Scalar y000 = Y(idx00, idx10, idx20);
  Scalar y001 = Y(idx00, idx10, idx21);
  Scalar y010 = Y(idx00, idx11, idx20);
  Scalar y011 = Y(idx00, idx11, idx21);
  Scalar y100 = Y(idx01, idx10, idx20);
  Scalar y101 = Y(idx01, idx10, idx21);
  Scalar y110 = Y(idx01, idx11, idx20);
  Scalar y111 = Y(idx01, idx11, idx21);

  Scalar x0_d = (x0_des - x00) / (x01 - x00);
  Scalar x1_d = (x1_des - x10) / (x11 - x10);
  Scalar x2_d = (x2_des - x20) / (x21 - x20);

  Scalar y00 = y000 * (1 - x0_d) + y100 * x0_d;
  Scalar y01 = y001 * (1 - x0_d) + y101 * x0_d;
  Scalar y10 = y010 * (1 - x0_d) + y110 * x0_d;
  Scalar y11 = y011 * (1 - x0_d) + y111 * x0_d;

  Scalar y0 = y00 * (1 - x1_d) + y10 * x1_d;
  Scalar y1 = y01 * (1 - x1_d) + y11 * x1_d;

  Scalar result = y0 * (1 - x2_d) + y1 * x2_d;

  return result;
}

int LiseagleDynamics::closestIdxLow(Vector<Dynamic> X, Scalar x) {
  Scalar min = std::numeric_limits<Scalar>::max();
  int x_low = 0;
  int N = X.size();
  for (int i = 0; i < N; i++) {
    if (abs(X(i) - x) < min) {
      min = abs(X(i) - x);
      if (X(i) < x) {
        x_low = i;
      } else {
        x_low = i - 1;
      }
    }
  }
  x_low = std::clamp(x_low, 0, N - 2);
  return x_low;
}

Scalar LiseagleDynamics::fitLinEval(Vector<Dynamic> X, Vector<Dynamic> Y,
                                    Scalar x) {
  int n = X.size();
  Scalar a, b;
  Scalar xsum = 0, x2sum = 0, ysum = 0,
         xysum = 0;  // variables for sums/sigma of xi,yi,xi^2,xiyi etc
  for (int i = 0; i < n; i++) {
    xsum = xsum + X(i);            // calculate sigma(xi)
    ysum = ysum + Y(i);            // calculate sigma(yi)
    x2sum = x2sum + pow(X(i), 2);  // calculate sigma(x^2i)
    xysum = xysum + X(i) * Y(i);   // calculate sigma(xi*yi)
  }
  a = (n * xysum - xsum * ysum) / (n * x2sum - xsum * xsum);  // calculate slope
  b = (x2sum * ysum - xsum * xysum) /
      (x2sum * n - xsum * xsum);  // calculate intercept
  return a * x + b;               // to calculate y(fitted) at given x point
}

Scalar LiseagleDynamics::randomizeOffset(Scalar x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + MatrixXd::Random(1,1)(0,0)*var;
}

Matrix<Dynamic,Dynamic> LiseagleDynamics::randomizeOffset(Matrix<Dynamic,Dynamic> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + MatrixXd::Random(x.rows(),x.cols())*var;
}

Scalar LiseagleDynamics::randomize(){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return MatrixXd::Random(1,1)(0,0)*var;
}

Scalar LiseagleDynamics::randomize(Scalar x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x*MatrixXd::Random(1,1)(0,0)*var;
}

Matrix<Dynamic,Dynamic> LiseagleDynamics::randomize(Matrix<Dynamic,Dynamic> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x.cwiseProduct(MatrixXd::Random(x.rows(),x.cols()))*var;
}

Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> LiseagleDynamics::randomize(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> rand_tensor;
  return x + x*rand_tensor.setRandom()*var;
}

Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> LiseagleDynamics::randomize(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> rand_tensor;
  return x + x*rand_tensor.setRandom()*var;
}

Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> LiseagleDynamics::randomize(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> rand_tensor;
  return x + x*rand_tensor.setRandom()*var;
}

Scalar LiseagleDynamics::offset(Scalar x, const Vector<Dynamic>& params){
  Scalar var = params(param_idx_);
  param_idx_ += 1;
  return x + var;
}

Matrix<Dynamic,Dynamic> LiseagleDynamics::offset(Matrix<Dynamic,Dynamic> x, const Vector<Dynamic>& params){
  int N = x.rows()*x.cols();
  Vector<Dynamic> var = params.segment(param_idx_, N);
  Matrix<Dynamic, Dynamic> m = Eigen::MatrixXd::Map(var.data(),x.rows(),x.cols());
  param_idx_ += N;
  return x + m;
}

Scalar LiseagleDynamics::scale(Scalar x, const Vector<Dynamic>& params){
  Scalar var = params(param_idx_);
  param_idx_ += 1;
  return x + x*var;
}

Matrix<Dynamic,Dynamic> LiseagleDynamics::scale(Matrix<Dynamic,Dynamic> x, const Vector<Dynamic>& params){
  int N = x.rows()*x.cols();
  Vector<Dynamic> var = params.segment(param_idx_, N);
  Matrix<Dynamic, Dynamic> m = Eigen::MatrixXd::Map(var.data(),x.rows(),x.cols());
  param_idx_ += N;
  return x + x.cwiseProduct(m);
}

Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> LiseagleDynamics::scale(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> x, const Vector<Dynamic>& params){
  int N = 11*3*3;
  Vector<Dynamic> var = params.segment(param_idx_, N);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> m;

  for (int i = 0; i < 11; i++) {
      for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
          m(i, j, k) = var(i + j*11 + k*11*3);
        }
      }
    }

  param_idx_ += N;
  return x + x*m;
}

Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> LiseagleDynamics::scale(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> x, const Vector<Dynamic>& params){
  int N = 11*2*4;
  Vector<Dynamic> var = params.segment(param_idx_, N);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> m;

  for (int i = 0; i < 11; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 4; k++) {
          m(i, j, k) = var(i + j*11 + k*11*2);
        }
      }
    }

  param_idx_ += N;
  return x + x*m;
}

Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> LiseagleDynamics::scale(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> x, const Vector<Dynamic>& params){
  int N = 3*3*3;
  Vector<Dynamic> var = params.segment(param_idx_, N);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> m;

  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
          m(i, j, k) = var(i + j*3 + k*3*3);
        }
      }
    }

  param_idx_ += N;
  return x + x*m;
}

}  // namespace flightlib