#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "str/DriverStationUtils.h"
#include "subsystems/Drive.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
  ConfigureSysIdBinds();
}

void RobotContainer::ConfigureBindings() {
  driveSub.SetDefaultCommand(driveSub.DriveTeleop(
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftY(), .1) *
            consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
      },
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftX(), .1) *
            consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
      },
      [this] {
        return frc::ApplyDeadband<double>(-driverJoystick.GetRightX(), .1) *
               consts::swerve::physical::MAX_ROT_SPEED;
      }));


}

void RobotContainer::ConfigureSysIdBinds() {
  tuningTable->PutBoolean("SteerPidTuning", false);
  tuningTable->PutBoolean("DrivePidTuning", false);
  tuningTable->PutBoolean("SteerSysIdVolts", false);
  tuningTable->PutBoolean("SteerSysIdTorqueCurrent", false);
  tuningTable->PutBoolean("DriveSysId", false);
  tuningTable->PutBoolean("WheelRadius", false);
  tuningTable->PutBoolean("Quasistatic", true);
  tuningTable->PutBoolean("Forward", true);

  steerTuneBtn.OnTrue(
      driveSub.TuneSteerPID([this] { return !steerTuneBtn.Get(); }));
  driveTuneBtn.OnTrue(
      driveSub.TuneDrivePID([this] { return !driveTuneBtn.Get(); }));

  steerSysIdVoltsBtn.WhileTrue(SteerVoltsSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  steerSysIdTorqueCurrentBtn.WhileTrue(SteerTorqueCurrentSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  driveSysIdBtn.WhileTrue(DriveSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  wheelRadiusBtn.WhileTrue(WheelRadiusSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); }));

}

frc2::CommandPtr RobotContainer::SteerVoltsSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(
          driveSub.SysIdSteerQuasistaticVoltage(
              frc2::sysid::Direction::kForward),
          driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kForward),
          quasistatic),
      frc2::cmd::Either(
          driveSub.SysIdSteerQuasistaticVoltage(
              frc2::sysid::Direction::kReverse),
          driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kReverse),
          quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::SteerTorqueCurrentSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(driveSub.SysIdSteerQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        driveSub.SysIdSteerDynamicTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        quasistatic),
      frc2::cmd::Either(driveSub.SysIdSteerQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        driveSub.SysIdSteerDynamicTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        quasistatic),
      fwd);
}


frc2::CommandPtr RobotContainer::DriveSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(driveSub.SysIdDriveQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        driveSub.SysIdDriveDynamicTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        quasistatic),
      frc2::cmd::Either(driveSub.SysIdDriveQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        driveSub.SysIdDriveDynamicTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::WheelRadiusSysIdCommands(
    std::function<bool()> fwd) {
  return frc2::cmd::Either(
      driveSub.WheelRadius(frc2::sysid::Direction::kForward),
      driveSub.WheelRadius(frc2::sysid::Direction::kReverse), fwd);
}

Drive& RobotContainer::GetDrive() {
  return driveSub;
}


