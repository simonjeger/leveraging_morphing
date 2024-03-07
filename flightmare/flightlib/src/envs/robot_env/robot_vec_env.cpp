#include "flightlib/envs/robot_env/robot_vec_env.hpp"

namespace flightlib {

template<typename EnvBaseName>
RobotVecEnv<EnvBaseName>::RobotVecEnv() {
  std::string config_path =
    getenv("FLIGHTMARE_PATH") +
    std::string("/flightpy/configs/control/config.yaml");
  cfg_ = YAML::LoadFile(config_path);
  // yaml configurations
  configEnv(cfg_);

  if (cfg_["turbulence"]["on"].as<bool>()){
    turbulence_dir_ = getenv("FLIGHTMARE_PATH") + std::string("/flightpy/turbulence/train/maps/"); //no option for testing (but so far that case is never called)
    initTurbulence();
  }
}

template<typename EnvBaseName>
RobotVecEnv<EnvBaseName>::RobotVecEnv(const std::string& cfg,
                                      const bool from_file,
                                      const bool train) {
  // load environment configuration
  if (from_file) {
    // load directly from a yaml file
    cfg_ = YAML::LoadFile(cfg);
  } else {
    // load from a string or dictionary
    cfg_ = YAML::Load(cfg);
  }
  configEnv(cfg_);

  if (cfg_["turbulence"]["on"].as<bool>()){
    if (train){
      turbulence_dir_ = getenv("FLIGHTMARE_PATH") + std::string("/flightpy/turbulence/train/maps/"); //no option for testing (but so far that case is never called)
    }
    else{
      turbulence_dir_ = getenv("FLIGHTMARE_PATH") + std::string("/flightpy/turbulence/test/maps/"); //no option for testing (but so far that case is never called)
    }
    initTurbulence();
  }
}

template<typename EnvBaseName>
RobotVecEnv<EnvBaseName>::RobotVecEnv(const YAML::Node& cfg_node) {
  cfg_ = cfg_node;
  configEnv(cfg_);

  if (cfg_["turbulence"]["on"].as<bool>()){
    turbulence_dir_ = getenv("FLIGHTMARE_PATH") + std::string("/flightpy/turbulence/train/maps/");
    initTurbulence();
  }
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::reset(Ref<MatrixRowMajor<>> obs) {
  if (obs.rows() != this->num_envs_ || obs.cols() != this->obs_dim_) {
    this->logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    return false;
  }

  this->receive_id_ = 0;
  for (int i = 0; i < this->num_envs_; i++) {
    this->envs_[i]->reset(obs.row(i));
  }

  // if enabled, load the same turbulence map for all environments
  if (cfg_["turbulence"]["on"].as<bool>()){
    loadTurbulence();
  }

  return true;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::step(Ref<MatrixRowMajor<>> act,
                                    Ref<Vector<>> dt,
                                    Ref<MatrixRowMajor<>> obs,
                                    Ref<MatrixRowMajor<>> reward,
                                    Ref<BoolVector<>> done,
                                    Ref<MatrixRowMajor<>> extra_info) {
  if (act.rows() != this->num_envs_ || act.cols() != this->act_dim_ ||
      dt.rows() != this->num_envs_ || dt.cols() != 1 ||
      obs.rows() != this->num_envs_ || obs.cols() != this->obs_dim_ ||
      reward.rows() != this->num_envs_ || reward.cols() != this->rew_dim_ ||
      done.rows() != this->num_envs_ || done.cols() != 1 ||
      extra_info.rows() != this->num_envs_ ||
      extra_info.cols() != (long int)this->extra_info_names_.size()) {
    this->logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    std::cout << "act dim  =       [" << act.rows() << "x" << act.cols()
              << "]\n"
              << "dt dim  =       [" << dt.rows() << "x" << dt.cols()
              << "]\n"
              << "obs dim  =       [" << obs.rows() << "x" << obs.cols()
              << "]\n"
              << "rew dim  =       [" << reward.rows() << "x" << reward.cols()
              << "]\n"
              << "done dim  =       [" << done.rows() << "x" << done.cols()
              << "]\n"
              << "extra info dim  =       [" << extra_info.rows() << "x"
              << extra_info.cols() << "]\n"
              << std::endl;
    return false;
  }

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentStep(i, act, dt, obs, reward, done, extra_info);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentStep(int agent_id,
                                            Ref<MatrixRowMajor<>> act,
                                            Ref<Vector<>> dt,
                                            Ref<MatrixRowMajor<>> obs,
                                            Ref<MatrixRowMajor<>> reward,
                                            Ref<BoolVector<>> done,
                                            Ref<MatrixRowMajor<>> extra_info) {
  // get individual rewards
  this->envs_[agent_id]->step(act.row(agent_id), dt(agent_id), obs.row(agent_id),
                              reward.row(agent_id));

  Scalar terminal_reward = 0;
  done[agent_id] = this->envs_[agent_id]->isTerminalState(terminal_reward);

  this->envs_[agent_id]->updateExtraInfo();
  for (int j = 0; j < extra_info.cols(); j++)
    extra_info(agent_id, j) =
      this->envs_[agent_id]->extra_info_[this->extra_info_names_[j]];

  if (done[agent_id]) {
    this->envs_[agent_id]->reset(obs.row(agent_id));
    reward(agent_id, reward.cols() - 1) = terminal_reward;
  }
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::setRobotState(Ref<MatrixRowMajor<>> state, 
                                            Ref<MatrixRowMajor<>> obs,
                                            Ref<MatrixRowMajor<>> reward, 
                                            Ref<BoolVector<>> done, 
                                            Ref<MatrixRowMajor<>> extra_info) {
  if (obs.rows() != this->num_envs_ || obs.cols() != this->obs_dim_ ||
      done.rows() != this->num_envs_ || done.cols() != 1 ||
      extra_info.rows() != this->num_envs_ ||
      extra_info.cols() != (long int)this->extra_info_names_.size()) {
    this->logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    std::cout << "obs dim  =       [" << obs.rows() << "x" << obs.cols()
              << "]\n"
              << "done dim  =       [" << done.rows() << "x" << done.cols()
              << "]\n"
              << "extra info dim  =       [" << extra_info.rows() << "x"
              << extra_info.cols() << "]\n"
              << std::endl;
    return false;
  }

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentSetRobotState(i, state, obs, reward, done, extra_info);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentSetRobotState(int agent_id,
                                                    Ref<MatrixRowMajor<>> state,
                                                    Ref<MatrixRowMajor<>> obs,
                                                    Ref<MatrixRowMajor<>> reward,
                                                    Ref<BoolVector<>> done, 
                                                    Ref<MatrixRowMajor<>> extra_info) {
  // get individual rewards
  this->envs_[agent_id]->setRobotState(state.row(agent_id), obs.row(agent_id), 
                              reward.row(agent_id));

  Scalar terminal_reward = 0;
  done[agent_id] = this->envs_[agent_id]->isTerminalState(terminal_reward);

  this->envs_[agent_id]->updateExtraInfo();
  for (int j = 0; j < extra_info.cols(); j++)
    extra_info(agent_id, j) =
      this->envs_[agent_id]->extra_info_[this->extra_info_names_[j]];

  // setRobotState does not reset the environment
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::setModelParams(Ref<MatrixRowMajor<>> params) {

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentSetModelParams(i, params);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentSetModelParams(int agent_id, 
                                                    Ref<MatrixRowMajor<>> params) {

  // set individual params
  this->envs_[agent_id]->setModelParams(params.row(agent_id));

}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::setLLDynamics(Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> cmd) {

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentSetLLDynamics(i, state, cmd);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentSetLLDynamics(int agent_id, 
                                                    Ref<MatrixRowMajor<>> state, 
                                                    Ref<MatrixRowMajor<>> cmd) {

  // set individual params
  this->envs_[agent_id]->setLLDynamics(state.row(agent_id), cmd.row(agent_id));

}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::setLLOffset(Ref<MatrixRowMajor<>> offset) {

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentSetLLLastOutput(i, offset);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentSetLLLastOutput(int agent_id, 
                                                    Ref<MatrixRowMajor<>> offset) {

  // set individual params
  this->envs_[agent_id]->setLLOffset(offset.row(agent_id));

}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::callLLRun(Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> act, 
  Ref<MatrixRowMajor<>> gt, Ref<MatrixRowMajor<>> ref, Ref<MatrixRowMajor<>> proj_matrix) {

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentCallLLRun(i, state, act, gt, ref, proj_matrix);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentCallLLRun(int agent_id, 
  Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> act, 
  Ref<MatrixRowMajor<>> gt, Ref<MatrixRowMajor<>> ref, Ref<MatrixRowMajor<>> proj_matrix) {

  // set individual params
  this->envs_[agent_id]->callLLRun(state.row(agent_id), act.row(agent_id), 
    gt.row(agent_id), ref.row(agent_id), proj_matrix.row(agent_id));

}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::setLLGains(Ref<MatrixRowMajor<>> gains, const int idx) {

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentSetLLGains(i, gains, idx);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentSetLLGains(int agent_id, 
                                                    Ref<MatrixRowMajor<>> gains, const int idx) {

  // set individual params
  this->envs_[agent_id]->setLLGains(gains.row(agent_id), idx);

}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::storeLLReservoir() {

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentStoreLLReservoir(i);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentStoreLLReservoir(int agent_id) {

  // set individual params
  this->envs_[agent_id]->storeLLReservoir();

}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::restoreLLReservoir() {

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentRestoreLLReservoir(i);
  }

  return true;
}

template<typename EnvBaseName>
void RobotVecEnv<EnvBaseName>::perAgentRestoreLLReservoir(int agent_id) {

  // set individual params
  this->envs_[agent_id]->restoreLLReservoir();

}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::setRobotActuator(Ref<MatrixRowMajor<>> robotactuator) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->setRobotActuator(robotactuator.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getActMean(Ref<MatrixRowMajor<>> robotmean) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getActMean(robotmean.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getActStd(Ref<MatrixRowMajor<>> robotstd) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getActStd(robotstd.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getRobotAct(Ref<MatrixRowMajor<>> robotact) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getRobotAct(robotact.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getRobotActuator(Ref<MatrixRowMajor<>> robotactuator) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getRobotActuator(robotactuator.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getRobotState(Ref<MatrixRowMajor<>> robotstate) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getRobotState(robotstate.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getRobotTimestamp(Ref<Vector<>> robotts) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    robotts(i) = this->envs_[i]->getRobotTimestamp();
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getRobotEnergy(Ref<Vector<>> robotenergy) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    robotenergy(i) = this->envs_[i]->getRobotEnergy();
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getWindCurl(Ref<MatrixRowMajor<>> wc) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getWindCurl(wc.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getWindCurlEst(Ref<MatrixRowMajor<>> wce) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getWindCurlEst(wce.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::getGoalState(Ref<MatrixRowMajor<>> goal) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getGoalState(goal.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::initTurbulence(void) {
  // only first time to resize matrix
  Matrix<3, 2> world_box = Matrix<3, 2>((cfg_["environment"]["world_box"].as<std::vector<Scalar>>()).data());
  Scalar res = cfg_["turbulence"]["res"].as<Scalar>();
  Scalar vel_max = cfg_["wind"]["mag_max"].as<Scalar>();
  Scalar dt_max = cfg_["simulation"]["max_t"].as<Scalar>();
  Scalar curl_dist = cfg_["turbulence"]["curl_dist"].as<Scalar>();

  int nx = (int)std::ceil(((world_box(3)-world_box(0))*std::pow(2.0,0.5) + vel_max*dt_max + 2*curl_dist)/res);
  int ny = (int)std::ceil(((world_box(4)-world_box(1))*std::pow(2.0,0.5) + 2*curl_dist)/res);
  int nz = (int)std::ceil(((world_box(5)-world_box(2))*std::pow(2.0,0.5) + 2*curl_dist)/res);

  int rows = nx*ny*nz;
  turbulence_.resize(rows,3);

  // Populate the vector with file paths from the folder
  for (const auto& entry : std::filesystem::directory_iterator(turbulence_dir_)) {
      if (entry.is_regular_file()) {
          files_.push_back(entry.path());
      }
  }

  return true;
}


template<typename EnvBaseName>
bool RobotVecEnv<EnvBaseName>::loadTurbulence(void) {
  // Select random file from folder
  std::uniform_int_distribution<size_t> dist(0, files_.size() - 1);
  size_t randomIndex = dist(rng_);
  std::filesystem::path randomFilePath = files_[randomIndex];

  // reading in all coefficients from csv files
  readCsv(randomFilePath, turbulence_);
  
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentSetTurbulence(i);
  }

  return true;
}

template<typename EnvBaseName>

void RobotVecEnv<EnvBaseName>::perAgentSetTurbulence(int agent_id) {
  // set individual params
  this->envs_[agent_id]->setTurbulence(turbulence_);
}

template<typename EnvBaseName>
RobotVecEnv<EnvBaseName>::~RobotVecEnv() {}

// IMPORTANT. Otherwise:
// Linker errors because of the separation between the
// declaration and definition of the template class.
// Segmentation fault (core dumped)
template class RobotVecEnv<RobotEnv>;

}  // namespace flightlib