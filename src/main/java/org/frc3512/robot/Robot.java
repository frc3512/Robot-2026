package org.frc3512.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.Path;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {

    // Set up data receivers & replay source
    switch (Constants.GeneralConstants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    robotContainer.checkHubStateChange();
  }

  @Override
  public void disabledInit() {
    Constants.GeneralConstants.hubTimer.stop();
    Constants.GeneralConstants.hubTimer.reset();
  }

  @Override
  public void disabledPeriodic() {
    // Pre-match module orientation using BLine
    // Keep modules pointed at the first target while the robot is disabled on the field
    try {
      String pathName = robotContainer.getWantedPath();
      if (pathName != null && robotContainer.getDrive() != null) {
        Path path = new Path(pathName);
        // Get the initial module direction based on current robot pose
        Rotation2d moduleDirection = 
            path.getInitialModuleDirection(robotContainer.getDrive()::getPose);
        // Set all modules to face this direction
        robotContainer.getDrive().setModuleOrientations(moduleDirection);
      }
    } catch (Exception e) {
      // Silently fail - module orientation is not critical
      // This prevents errors from breaking disabled mode
    }
  }

  @Override
  public void autonomousInit() {
    try {
      autonomousCommand = robotContainer.getAutonomousCommand();

      if (autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(autonomousCommand);
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to initialize autonomous command", e.getStackTrace());
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    Constants.GeneralConstants.hubTimer.reset();
    Constants.GeneralConstants.hubTimer.start();
  }

  @Override
  public void teleopPeriodic() {}
}
