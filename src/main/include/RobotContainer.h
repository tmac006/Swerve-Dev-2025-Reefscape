// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/NetworkButton.h>

#include <functional>
#include <memory>

#include "subsystems/Drive.h"


class RobotContainer {
 public:
  RobotContainer();
  Drive& GetDrive();
 private:
  void ConfigureBindings();
  void ConfigureSysIdBinds();
  frc2::CommandPtr SteerVoltsSysIdCommands(std::function<bool()> fwd,
                                           std::function<bool()> quasistatic);
  frc2::CommandPtr SteerTorqueCurrentSysIdCommands(
      std::function<bool()> fwd, std::function<bool()> quasistatic);
  frc2::CommandPtr DriveSysIdCommands(std::function<bool()> fwd,
                                      std::function<bool()> quasistatic);
  frc2::CommandPtr WheelRadiusSysIdCommands(std::function<bool()> fwd);

  frc2::CommandXboxController driverJoystick{0};

  Drive driveSub{};

  std::shared_ptr<nt::NetworkTable> tuningTable{
      nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};
  frc2::NetworkButton steerTuneBtn{tuningTable, "SteerPidTuning"};
  frc2::NetworkButton driveTuneBtn{tuningTable, "DrivePidTuning"};
  frc2::NetworkButton steerSysIdVoltsBtn{tuningTable, "SteerSysIdVolts"};
  frc2::NetworkButton steerSysIdTorqueCurrentBtn{tuningTable,
                                                 "SteerSysIdTorqueCurrent"};
  frc2::NetworkButton driveSysIdBtn{tuningTable, "DriveSysId"};
  frc2::NetworkButton wheelRadiusBtn{tuningTable, "WheelRadius"};
};
