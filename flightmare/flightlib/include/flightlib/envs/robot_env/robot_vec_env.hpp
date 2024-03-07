#pragma once

// yaml cpp
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "flightlib/envs/robot_env/robot_env.hpp"
#include "flightlib/envs/vec_env_base.hpp"

namespace flightlib {

template<typename EnvBaseName>
class RobotVecEnv : public VecEnvBase<EnvBaseName> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotVecEnv();
  RobotVecEnv(const std::string& cfg, const bool from_file = true, const bool train = true);
  RobotVecEnv(const YAML::Node& cfg_node);
  ~RobotVecEnv();

  using VecEnvBase<EnvBaseName>::configEnv;

  bool reset(Ref<MatrixRowMajor<>> obs) override;
  bool step(Ref<MatrixRowMajor<>> act, Ref<Vector<>> dt, Ref<MatrixRowMajor<>> obs,
            Ref<MatrixRowMajor<>> reward, Ref<BoolVector<>> done,
            Ref<MatrixRowMajor<>> extra_info) override;
  bool setRobotState(Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> obs,
            Ref<MatrixRowMajor<>> reward, Ref<BoolVector<>> done, 
            Ref<MatrixRowMajor<>> extra_info);
  bool setModelParams(Ref<MatrixRowMajor<>> params);
  bool setLLDynamics(Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> cmd);
	bool setLLOffset(Ref<MatrixRowMajor<>> offset);
  bool callLLRun(Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> act, 
    Ref<MatrixRowMajor<>> gt, Ref<MatrixRowMajor<>> ref, Ref<MatrixRowMajor<>> proj_matrix);
  bool setLLGains(Ref<MatrixRowMajor<>> gains, const int idx);
  bool storeLLReservoir();
  bool restoreLLReservoir();
  bool setRobotActuator(Ref<MatrixRowMajor<>> robotactuator);
  bool getActMean(Ref<MatrixRowMajor<>> actmean);
  bool getActStd(Ref<MatrixRowMajor<>> actstd);
  bool getRobotAct(Ref<MatrixRowMajor<>> robotact);
  bool getRobotActuator(Ref<MatrixRowMajor<>> robotactuator);
  bool getRobotState(Ref<MatrixRowMajor<>> robotstate);
  bool getRobotTimestamp(Ref<Vector<>> robotts);
  bool getRobotEnergy(Ref<Vector<>> robotenergy);
  bool getWindCurl(Ref<MatrixRowMajor<>> wc);
  bool getWindCurlEst(Ref<MatrixRowMajor<>> wce);
  bool getGoalState(Ref<MatrixRowMajor<>> goal);
  inline std::vector<std::string> getRewardNames(void) {
    return this->envs_[0]->getRewardNames();
  };

 private:
  void perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act, Ref<Vector<>> dt,
                    Ref<MatrixRowMajor<>> obs,
                    Ref<MatrixRowMajor<>> reward_units, Ref<BoolVector<>> done,
                    Ref<MatrixRowMajor<>> extra_info) override;
  void perAgentSetRobotState(int agent_id, Ref<MatrixRowMajor<>> state,
                    Ref<MatrixRowMajor<>> obs, 
                    Ref<MatrixRowMajor<>> reward_units, Ref<BoolVector<>> done,
                    Ref<MatrixRowMajor<>> extra_info);
  void perAgentVelPred(int agent_id, Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> state,
                    Ref<MatrixRowMajor<>> velpred, Ref<BoolVector<>> done,
                    Ref<MatrixRowMajor<>> extra_info);
  void perAgentSetModelParams(int agent_id, Ref<MatrixRowMajor<>> params);
  void perAgentSetLLDynamics(int agent_id, Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> cmd);
  void perAgentSetLLLastOutput(int agent_id, Ref<MatrixRowMajor<>> offset);
  void perAgentCallLLRun(int agent_id, Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> act,
    Ref<MatrixRowMajor<>> gt, Ref<MatrixRowMajor<>> ref, Ref<MatrixRowMajor<>> proj_matrix);
  void perAgentSetLLGains(int agent_id, Ref<MatrixRowMajor<>> gains, const int idx);
  void perAgentStoreLLReservoir(int agent_id);
  void perAgentRestoreLLReservoir(int agent_id);
  
  bool initTurbulence(void);
  bool loadTurbulence(void);
  void perAgentSetTurbulence(int agent_id);

  //
  YAML::Node cfg_;
  std::vector<std::string> reward_names_;

  // turbulence
  std::string turbulence_dir_;
  std::vector<std::filesystem::path> files_;
	std::default_random_engine rd_;
  std::mt19937 rng_{rd_()};
  Matrix<Dynamic, 3> turbulence_;
};

}  // namespace flightlib