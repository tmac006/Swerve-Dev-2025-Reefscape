#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/Threads.h>
#include <frc2/command/CommandScheduler.h>
#include <wpinet/WebServer.h>

#include <ctre/phoenix6/SignalLogger.hpp>

#include "Constants/Constants.h"
#include "Constants/SwerveConstants.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "photon/PhotonPoseEstimator.h"

Robot::Robot() {
  // DANGEROUS MAKE SURE CODE DOESN'T BLOCK!!!
  frc::SetCurrentThreadPriority(true, 15);
  ctre::phoenix6::SignalLogger::EnableAutoLogging(true);
  ctre::phoenix6::SignalLogger::Start();
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  AddPeriodic([this] { m_container.GetDrive().UpdateOdom(); },
              1 / consts::swerve::ODOM_UPDATE_RATE, 2_ms);
  wpi::WebServer::GetInstance().Start(5800,
                                      frc::filesystem::GetDeployDirectory());
}

void Robot::RobotPeriodic() {
  units::second_t now = frc::Timer::GetFPGATimestamp();
  units::second_t loopTime = now - lastTotalLoopTime;
  loopTimePub.Set((1 / loopTime).value());

  frc2::CommandScheduler::GetInstance().Run();
  UpdateVision();

  lastTotalLoopTime = now;
  matchTimePub.Set(frc::DriverStation::GetMatchTime().value());
  battVoltagePub.Set(frc::RobotController::GetBatteryVoltage().value());
}

void Robot::SimulationPeriodic() {
  // m_container.GetVision().SimulationPeriodic(
  //     m_container.GetDrive().GetOdomPose());
}

void Robot::UpdateVision() {
  // m_container.GetVision().UpdateYaws(m_container.GetDrive().GetGyroYaw(),
  //                                    frc::Timer::GetFPGATimestamp());
  // auto visionEstimates = m_container.GetVision().GetCameraEstimatedPoses(
  //     frc::Pose3d{m_container.GetDrive().GetRobotPose()});

  // auto stdDevs = m_container.GetVision().GetPoseStdDevs(visionEstimates);

  // frc::Pose3d pose = frc::Pose3d{m_container.GetDrive().GetRobotPose()};

  // m_container.GetVision().UpdateCameraPositionVis(pose);

  // int i = 0;
  // for (const auto& est : visionEstimates) {
  //   if (est.has_value()) {
  //     // TRICKING PHOTON STRATS
  //     // WE ONLY WANT SINGLE TAG RESULTS TO BE ADDED IF THEY ARE OUR TRIG
  //     METHOD if (est->targetsUsed.size() == 1) {
  //       if (est->strategy == photon::PoseStrategy::CLOSEST_TO_CAMERA_HEIGHT)
  //       {
  //         m_container.GetDrive().AddVisionMeasurement(
  //             est.value().estimatedPose.ToPose2d(), est.value().timestamp,
  //             stdDevs[i].value());
  //       }
  //     } else {
  //       m_container.GetDrive().AddVisionMeasurement(
  //           est.value().estimatedPose.ToPose2d(), est.value().timestamp,
  //           stdDevs[i].value());
  //     }
  //   }
  //   i++;
  // }
}

void Robot::DisabledInit() {
  // m_container.GetPivot().SetToStartingPosition();
  // m_container.GetElevator().SetToZeroHeight();
}

void Robot::DisabledPeriodic() {
  // m_container.GetPivot().SetToStartingPosition();
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  // m_container.GetManipulator().OverrideHasCoral(true);
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
