


#include <python_interface_go1/real_robot_interface_go1.hpp>
// #include <python_interface_go1/real_robot_interface_go1_highlevel.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> // This header includes #include <Eigen/Core>
#include <pybind11/stl.h>



/* -------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Python Wrapper ---------------------------------------------------- */
/* -------------------------------------------------------------------------------------------------------------- */


namespace py = pybind11;

PYBIND11_MODULE(real_robot_interface_go1, m) {
    m.doc() = R"pbdoc(
          Go1 Robot Interface Python Bindings
          -----------------------
          .. currentmodule:: real_robot_interface_go1
          .. autosummary::
             :toctree: _generate
      )pbdoc";

    py::class_<Cartesian>(m, "Cartesian")
        .def(py::init<>())
        .def_readwrite("x", &Cartesian::x)
        .def_readwrite("y", &Cartesian::y)
        .def_readwrite("z", &Cartesian::z);

    py::class_<IMU>(m, "IMU")
        .def(py::init<>())
        .def_readwrite("quaternion", &IMU::quaternion)
        .def_readwrite("gyroscope", &IMU::gyroscope)
        .def_readwrite("accelerometer", &IMU::accelerometer)
        .def_readwrite("rpy", &IMU::rpy)
        .def_readwrite("temperature", &IMU::temperature);

    py::class_<LED>(m, "LED")
        .def(py::init<>())
        .def_readwrite("r", &LED::r)
        .def_readwrite("g", &LED::g)
        .def_readwrite("b", &LED::b);

    py::class_<MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("mode", &MotorState::mode)
        .def_readwrite("q", &MotorState::q)
        .def_readwrite("dq", &MotorState::dq)
        .def_readwrite("ddq", &MotorState::ddq)
        .def_readwrite("tauEst", &MotorState::tauEst)
        .def_readwrite("q_raw", &MotorState::q_raw)
        .def_readwrite("dq_raw", &MotorState::dq_raw)
        .def_readwrite("ddq_raw", &MotorState::ddq_raw)
        .def_readwrite("temperature", &MotorState::temperature)
        .def_readwrite("reserve", &MotorState::reserve);

    py::class_<MotorCmd>(m, "MotorCmd")
        .def(py::init<>())
        .def_readwrite("mode", &MotorCmd::mode)
        .def_readwrite("q", &MotorCmd::q)
        .def_readwrite("dq", &MotorCmd::dq)
        .def_readwrite("tau", &MotorCmd::tau)
        .def_readwrite("Kp", &MotorCmd::Kp)
        .def_readwrite("Kd", &MotorCmd::Kd)
        .def_readwrite("reserve", &MotorCmd::reserve);

    py::class_<LowState>(m, "LowState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &LowState::levelFlag)
        .def_readwrite("commVersion", &LowState::commVersion)
        .def_readwrite("robotID", &LowState::robotID)
        .def_readwrite("SN", &LowState::SN)
        .def_readwrite("bandWidth", &LowState::bandWidth)
        .def_readwrite("imu", &LowState::imu)
        .def_readwrite("motorState", &LowState::motorState)
        .def_readwrite("footForce", &LowState::footForce)
        .def_readwrite("footForceEst", &LowState::footForceEst)
        .def_readwrite("tick", &LowState::tick)
        .def_readwrite("wirelessRemote", &LowState::wirelessRemote)
        .def_readwrite("reserve", &LowState::reserve)
        .def_readwrite("crc", &LowState::crc);

    py::class_<LowCmd>(m, "LowCmd")
        .def(py::init<>())
        .def_readwrite("levelFlag", &LowCmd::levelFlag)
        .def_readwrite("commVersion", &LowCmd::commVersion)
        .def_readwrite("robotID", &LowCmd::robotID)
        .def_readwrite("SN", &LowCmd::SN)
        .def_readwrite("bandWidth", &LowCmd::bandWidth)
        .def_readwrite("motorCmd", &LowCmd::motorCmd)
        // .def_readwrite("led", &LowCmd::led)
        .def_readwrite("wirelessRemote", &LowCmd::wirelessRemote)
        .def_readwrite("reserve", &LowCmd::reserve)
        .def_readwrite("crc", &LowCmd::crc);

    py::class_<HighState>(m, "HighState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighState::levelFlag)
        .def_readwrite("commVersion", &HighState::commVersion)
        .def_readwrite("robotID", &HighState::robotID)
        .def_readwrite("SN", &HighState::SN)
        .def_readwrite("bandWidth", &HighState::bandWidth)
        .def_readwrite("mode", &HighState::mode)
        .def_readwrite("imu", &HighState::imu)
        .def_readwrite("position", &HighState::position)
        .def_readwrite("velocity", &HighState::velocity)
        // .def_readwrite("forwardSpeed", &HighState::forwardSpeed)
        // .def_readwrite("sideSpeed", &HighState::sideSpeed)
        // .def_readwrite("rotateSpeed", &HighState::rotateSpeed)
        .def_readwrite("bodyHeight", &HighState::bodyHeight)
        // .def_readwrite("updownSpeed", &HighState::updownSpeed)
        // .def_readwrite("forwardPosition", &HighState::forwardPosition)
        // .def_readwrite("sidePosition", &HighState::sidePosition)
        .def_readwrite("footPosition2Body", &HighState::footPosition2Body)
        .def_readwrite("footSpeed2Body", &HighState::footSpeed2Body)
        .def_readwrite("footForce", &HighState::footForce)
        .def_readwrite("footForceEst", &HighState::footForceEst)
        // .def_readwrite("tick", &HighState::tick)
        .def_readwrite("wirelessRemote", &HighState::wirelessRemote)
        .def_readwrite("reserve", &HighState::reserve)
        .def_readwrite("crc", &HighState::crc);

    py::class_<HighCmd>(m, "HighCmd")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighCmd::levelFlag)
        .def_readwrite("commVersion", &HighCmd::commVersion)
        .def_readwrite("robotID", &HighCmd::robotID)
        .def_readwrite("SN", &HighCmd::SN)
        .def_readwrite("bandWidth", &HighCmd::bandWidth)
        .def_readwrite("mode", &HighCmd::mode)
        .def_readwrite("position", &HighCmd::postion)
        .def_readwrite("velocity", &HighCmd::velocity)
        .def_readwrite("gaitType", &HighCmd::gaitType)
        // .def_readwrite("forwardSpeed", &HighCmd::forwardSpeed)
        // .def_readwrite("sideSpeed", &HighCmd::sideSpeed)
        // .def_readwrite("rotateSpeed", &HighCmd::rotateSpeed)
        .def_readwrite("bodyHeight", &HighCmd::bodyHeight)
        .def_readwrite("footRaiseHeight", &HighCmd::footRaiseHeight)
        // .def_readwrite("yaw", &HighCmd::yaw)
        // .def_readwrite("pitch", &HighCmd::pitch)
        // .def_readwrite("roll", &HighCmd::roll)
        .def_readwrite("euler", &HighCmd::euler)
        .def_readwrite("yawSpeed", &HighCmd::yawSpeed)
        .def_readwrite("led", &HighCmd::led)
        .def_readwrite("wirelessRemote", &HighCmd::wirelessRemote)
        // .def_readwrite("AppRemote", &HighCmd::AppRemote)
        .def_readwrite("reserve", &HighCmd::reserve)
        .def_readwrite("crc", &HighCmd::crc);

    py::class_<UDPState>(m, "UDPState")
        .def(py::init<>())
        .def_readwrite("TotalCount", &UDPState::TotalCount)
        .def_readwrite("SendCount", &UDPState::SendCount)
        .def_readwrite("RecvCount", &UDPState::RecvCount)
        .def_readwrite("SendError", &UDPState::SendError)
        .def_readwrite("FlagError", &UDPState::FlagError)
        .def_readwrite("RecvCRCError", &UDPState::RecvCRCError)
        .def_readwrite("RecvLoseError", &UDPState::RecvLoseError);

    py::class_<RealRobotInterfaceGo1>(m, "RealRobotInterfaceGo1")
        // .def(py::init<const std::array<float, 12>, const std::array<float, 12>>())
        .def(py::init<>())
        .def("collect_observations", &RealRobotInterfaceGo1::CollectObservations)
        .def("update_all_observations", &RealRobotInterfaceGo1::update_all_observations)
        .def("set_deltaT", &RealRobotInterfaceGo1::set_deltaT)
        .def("get_deltaT", &RealRobotInterfaceGo1::get_deltaT)
        .def("send_command", &RealRobotInterfaceGo1::SendCommand)
        .def("send_desired_position", &RealRobotInterfaceGo1::send_desired_position)
        .def("set_PD_gains", &RealRobotInterfaceGo1::set_PD_gains)
        .def("get_joint_pos_curr", &RealRobotInterfaceGo1::get_joint_pos_curr)
        .def("update_joint_pos_curr", &RealRobotInterfaceGo1::update_joint_pos_curr)
        .def("update_joint_vel_curr", &RealRobotInterfaceGo1::update_joint_vel_curr)
        .def("update_body_linear_velocity", &RealRobotInterfaceGo1::update_body_linear_velocity)
        .def("update_body_angular_velocity", &RealRobotInterfaceGo1::update_body_angular_velocity)
        .def("update_body_orientation", &RealRobotInterfaceGo1::update_body_orientation);
        // .def("main", &RealRobotInterfaceGo1::main)
        // .def("stand_up", &RealRobotInterfaceGo1::stand_up);

    // py::class_<RealRobotInterfaceGo1HighLevel>(m, "RealRobotInterfaceGo1HighLevel")
    //     // .def(py::init<const std::array<float, 12>, const std::array<float, 12>>())
    //     .def(py::init<>())
    //     .def("get_observations", &RealRobotInterfaceGo1HighLevel::get_observations)
    //     .def("get_joint_pos_curr", &RealRobotInterfaceGo1HighLevel::get_joint_pos_curr)
    //     .def("get_joint_vel_curr", &RealRobotInterfaceGo1HighLevel::get_joint_vel_curr)
    //     .def("get_body_linear_velocity", &RealRobotInterfaceGo1HighLevel::get_body_linear_velocity)
    //     .def("get_body_angular_velocity", &RealRobotInterfaceGo1HighLevel::get_body_angular_velocity)
    //     .def("get_body_orientation", &RealRobotInterfaceGo1HighLevel::get_body_orientation)
    //     .def("update_mode_behavior", &RealRobotInterfaceGo1HighLevel::update_mode_behavior)
    //     .def("update_gait_type", &RealRobotInterfaceGo1HighLevel::update_gait_type)
    //     .def("send_linear_and_angular_velocity_commands", &RealRobotInterfaceGo1HighLevel::send_linear_and_angular_velocity_commands)
    //     .def("send_desired_body_height", &RealRobotInterfaceGo1HighLevel::send_desired_body_height)
    //     .def("set_deltaT", &RealRobotInterfaceGo1HighLevel::set_deltaT)
    //     .def("get_deltaT", &RealRobotInterfaceGo1HighLevel::get_deltaT)
    //     .def("print_joint_info", &RealRobotInterfaceGo1HighLevel::print_joint_info)
    //     .def("set_IP_and_port_udp", &RealRobotInterfaceGo1HighLevel::set_IP_and_port_udp)
    //     .def("read_IPs_and_ports", &RealRobotInterfaceGo1HighLevel::read_IPs_and_ports);

    py::class_<GymEnvironmentRealGo1>(m, "GymEnvironmentRealGo1")
        .def(py::init<>())
        .def("init", &GymEnvironmentRealGo1::init)
        .def("update_env_observation", &GymEnvironmentRealGo1::update_env_observation)
        .def("step", &GymEnvironmentRealGo1::step)
        .def("set_PD_gains", &GymEnvironmentRealGo1::set_PD_gains)
        .def("set_deltaT", &GymEnvironmentRealGo1::set_deltaT)
        .def("get_deltaT", &GymEnvironmentRealGo1::get_deltaT)
        // .def("stand_up", &GymEnvironmentRealGo1::stand_up)
        .def("set_action_std", &GymEnvironmentRealGo1::set_action_std)
        .def("set_action_mean", &GymEnvironmentRealGo1::set_action_mean)
        .def("get_joint_pos_curr", &GymEnvironmentRealGo1::get_joint_pos_curr)
        .def_readonly("low_state", &GymEnvironmentRealGo1::state) // DBG
        // .def_read("low_state", &GymEnvironmentRealGo1::state) // DBG
        .def("get_env_observation", &GymEnvironmentRealGo1::get_env_observation);

    #ifdef VERSION_INFO
      m.attr("__version__") = VERSION_INFO;
    #else
      m.attr("__version__") = "dev";
    #endif

      m.attr("TEST") = py::int_(int(42));

}
