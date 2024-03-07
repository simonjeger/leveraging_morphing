# LIS Flightmare

This is an adaptation from the [original Flightmare](https://github.com/uzh-rpg/flightmare) developed by the RPG. 

This publication only uses a small part of the provided framework, but to ensure reproducibility, we upload the whole stack.
Note that this code enables real-world experiments on the Avian Informed Drone LisEagle and does not provide a simulation framework.

The files of interest are:

in **flightpy**
- config.yaml (config file as used in the experiments)
- run_controller.py (runs the control method)
- run_bo.py (runs Bayesian Optimization)

in **flightlib**
- liseagle_dynamics.hpp, liseagle_dynamics.cpp (dynamics model of the avian-informed drone LisEagle)
- motor.hpp, motor.cpp (motor model)
- servo.hpp, servo.cpp (servo model)

in **controller**
- high_level_pid (high-level PID controller that takes in position and gives out thrust and body rates)

## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details.

# Installation guide for Ubuntu 18.04 / 20.04 (recommended)

## Get Compilers

Flightmare requires CMake and GCC compilers.

```
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   build-essential \
   cmake \
   libzmqpp-dev \
   libopencv-dev
source ~/.bashrc
```

## Conda

If not done already, install Conda, download [here](https://www.anaconda.com/products/distribution)

```
cd Downloads
chmod +x anaconda_file_name
./anaconda_file_name
```

Add environment variable to .bashrc file (change ```path/to/flightmare``` to your specific path)

```
echo "export FLIGHTMARE_PATH=~path/to/flightmare" >> ~/.bashrc
source ~/.bashrc
```

Create and activate new conda environment called pubflight

```
cd flightmare
conda env create -f environment.yml
conda activate pubflight
```

## Install flightmare packages

Installing flightgym (option 1)

```
cd flightmare/flightlib
pip install .
```

Alternatively you can also compile flightgym in this way (option 2), which is faster since it only recompiles the files you changed. The -j4 specifies how many cores you are using to compile (in this case 4).

```
cd flightmare/flightlib/build
clear;cmake ..;make -j4;pip install .
```

## In case of ERROR: Failed building wheel for flightgym

Check that you've installed the right version of gcc (7.5.0) and g++ (7.5.0). If not, [this](https://askubuntu.com/questions/26498/how-to-choose-the-default-gcc-and-g-version/26518#26518) might be helpfull.
If that wasn't the problem, it might be that pybind doesn't have access to the python packages. Try the following steps:
```
cd flightlib/build
cmake .. \
-DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")  \
-DPYTHON_LIBRARY=$(python -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))") \
-DPYTHON_EXECUTABLE:FILEPATH=`which python`
make -j3
pip install .
```

# Run an example

**Stable Hover Flight against Airflow**

In the config file, adapt the airspeed by changing **wind, mag_min & mag_max**.
```
cd flightpy
python3 run_controller.py
```

**Using morphing to optimize for energy efficiency**

In the config file, change **experiments, energy** to **True**
```
cd flightpy
python3 run_controller.py
python3 bo.py (in a seperate thread)
```
