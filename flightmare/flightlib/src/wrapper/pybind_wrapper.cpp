
// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// flightlib
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/robot_env/robot_env.hpp"
#include "flightlib/envs/robot_env/robot_vec_env.hpp"
#include "flightlib/envs/vec_env_base.hpp"

namespace py = pybind11;
using namespace flightlib;

PYBIND11_MODULE(flightgym, m) {
	py::class_<RobotVecEnv<RobotEnv>>(m, "RobotEnv_v1")
	    .def(py::init<>())
	    .def(py::init<const std::string&>())
	    .def(py::init<const std::string&, const bool, const bool>())
	    .def("reset",
	         static_cast<bool (RobotVecEnv<RobotEnv>::*)(Ref<MatrixRowMajor<>>)>(&RobotVecEnv<RobotEnv>::reset),
	         "reset")
	    .def("step", &RobotVecEnv<RobotEnv>::step)
		.def("setRobotState", &RobotVecEnv<RobotEnv>::setRobotState)
		.def("setModelParams", &RobotVecEnv<RobotEnv>::setModelParams)
		.def("setLLDynamics", &RobotVecEnv<RobotEnv>::setLLDynamics)
		.def("setLLOffset", &RobotVecEnv<RobotEnv>::setLLOffset)
		.def("callLLRun", &RobotVecEnv<RobotEnv>::callLLRun)
		.def("setLLGains", &RobotVecEnv<RobotEnv>::setLLGains)
		.def("storeLLReservoir", &RobotVecEnv<RobotEnv>::storeLLReservoir)
		.def("restoreLLReservoir", &RobotVecEnv<RobotEnv>::restoreLLReservoir)
		.def("setRobotActuator", &RobotVecEnv<RobotEnv>::setRobotActuator)
	    .def("setSeed", &RobotVecEnv<RobotEnv>::setSeed)
	    .def("close", &RobotVecEnv<RobotEnv>::close)
	    .def("isTerminalState", &RobotVecEnv<RobotEnv>::isTerminalState)
	    .def("curriculumUpdate", &RobotVecEnv<RobotEnv>::curriculumUpdate)
	    .def("connectUnity", &RobotVecEnv<RobotEnv>::connectUnity)
	    .def("disconnectUnity", &RobotVecEnv<RobotEnv>::disconnectUnity)
	    .def("updateUnity", &RobotVecEnv<RobotEnv>::updateUnity)
	    .def("getObs", &RobotVecEnv<RobotEnv>::getObs)
		.def("getActMean", &RobotVecEnv<RobotEnv>::getActMean)
		.def("getActStd", &RobotVecEnv<RobotEnv>::getActStd)
	    .def("getRobotAct", &RobotVecEnv<RobotEnv>::getRobotAct)
	    .def("getRobotActuator", &RobotVecEnv<RobotEnv>::getRobotActuator)
	    .def("getRobotState", &RobotVecEnv<RobotEnv>::getRobotState)
	    .def("getRobotTimestamp", &RobotVecEnv<RobotEnv>::getRobotTimestamp)
	    .def("getRobotEnergy", &RobotVecEnv<RobotEnv>::getRobotEnergy)
	    .def("getWindCurl", &RobotVecEnv<RobotEnv>::getWindCurl)
	    .def("getWindCurlEst", &RobotVecEnv<RobotEnv>::getWindCurlEst)
	    .def("getGoalState", &RobotVecEnv<RobotEnv>::getGoalState)
	    .def("getImage", &RobotVecEnv<RobotEnv>::getImage)
	    .def("getDepthImage", &RobotVecEnv<RobotEnv>::getDepthImage)
	    .def("getNumOfEnvs", &RobotVecEnv<RobotEnv>::getNumOfEnvs)
	    .def("getObsDim", &RobotVecEnv<RobotEnv>::getObsDim)
	    .def("getActDim", &RobotVecEnv<RobotEnv>::getActDim)
	    .def("getRewDim", &RobotVecEnv<RobotEnv>::getRewDim)
	    .def("getImgHeight", &RobotVecEnv<RobotEnv>::getImgHeight)
	    .def("getImgWidth", &RobotVecEnv<RobotEnv>::getImgWidth)
	    .def("getRewardNames", &RobotVecEnv<RobotEnv>::getRewardNames)
	    .def("getExtraInfoNames", &RobotVecEnv<RobotEnv>::getExtraInfoNames)
	    .def("__repr__", [](const RobotVecEnv<RobotEnv>& a) { return "RPG Drone Control Environment"; });
}