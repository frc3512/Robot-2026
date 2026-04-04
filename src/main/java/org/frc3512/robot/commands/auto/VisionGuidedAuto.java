package org.frc3512.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Simplified vision-guided auto command that provides continuous pose correction
 * during path following. Uses vision measurements to correct robot position
 * when reliable data is available.
 */
public class VisionGuidedAuto extends Command {
  private final Drive drive;
  private final Vision vision;
  private final Command pathCommand;
  
  // Vision correction parameters
  private static final double VISION_TIMEOUT = 0.5; // 500ms timeout
  private static final double MAX_POSITION_ERROR = 0.3; // 30cm max error
  private static final double MAX_ANGLE_ERROR = 15.0; // 15 degrees max error
  private static final double UPDATE_INTERVAL = 0.1; // 100ms between updates
  
  private double lastUpdateTime = 0.0;
  private int totalCorrections = 0;
  private int rejectedCorrections = 0;

  public VisionGuidedAuto(Drive drive, Vision vision, Command pathCommand) {
    this.drive = drive;
    this.vision = vision;
    this.pathCommand = pathCommand;
    
    addRequirements(drive); // Require drive to prevent conflicts
  }

  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(pathCommand);
    lastUpdateTime = 0.0;
    totalCorrections = 0;
    rejectedCorrections = 0;
    
    Logger.recordOutput("VisionGuidedAuto/Active", true);
    Logger.recordOutput("VisionGuidedAuto/TotalCorrections", 0);
    Logger.recordOutput("VisionGuidedAuto/RejectedCorrections", 0);
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    
    // Check if we should apply vision correction
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      if (shouldApplyCorrection()) {
        if (applyVisionCorrection()) {
          totalCorrections++;
        } else {
          rejectedCorrections++;
        }
        lastUpdateTime = currentTime;
      }
    }
    
    // Log status
    Logger.recordOutput("VisionGuidedAuto/PathActive", pathCommand.isScheduled());
    Logger.recordOutput("VisionGuidedAuto/TotalCorrections", totalCorrections);
    Logger.recordOutput("VisionGuidedAuto/RejectedCorrections", rejectedCorrections);
    Logger.recordOutput("VisionGuidedAuto/VisionAvailable", vision.hasRecentVisionData(VISION_TIMEOUT));
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand.isScheduled()) {
      pathCommand.cancel();
    }
    
    Logger.recordOutput("VisionGuidedAuto/Active", false);
    Logger.recordOutput("VisionGuidedAuto/FinalCorrections", totalCorrections);
    Logger.recordOutput("VisionGuidedAuto/FinalRejections", rejectedCorrections);
  }

  @Override
  public boolean isFinished() {
    return !pathCommand.isScheduled();
  }

  private boolean shouldApplyCorrection() {
    // Only apply correction if we have recent vision data
    return vision.hasRecentVisionData(VISION_TIMEOUT);
  }

  private boolean applyVisionCorrection() {
    Pose2d currentPose = drive.getPose();
    Pose2d visionPose = vision.getLatestVisionPose();
    
    if (visionPose == null) {
      Logger.recordOutput("VisionGuidedAuto/State", "NoVisionPose");
      return false;
    }
    
    // Calculate position and angle errors
    double positionError = currentPose.getTranslation().getDistance(visionPose.getTranslation());
    double angleError = Math.abs(currentPose.getRotation().minus(visionPose.getRotation()).getDegrees());
    
    // Reject corrections that are too large (likely bad measurements)
    if (positionError > MAX_POSITION_ERROR || angleError > MAX_ANGLE_ERROR) {
      Logger.recordOutput("VisionGuidedAuto/State", "Rejected");
      Logger.recordOutput("VisionGuidedAuto/PositionError", positionError);
      Logger.recordOutput("VisionGuidedAuto/AngleError", angleError);
      return false;
    }
    
    // Apply the vision correction
    drive.setPose(visionPose);
    
    Logger.recordOutput("VisionGuidedAuto/State", "Corrected");
    Logger.recordOutput("VisionGuidedAuto/PositionError", positionError);
    Logger.recordOutput("VisionGuidedAuto/AngleError", angleError);
    Logger.recordOutput("VisionGuidedAuto/OdometryPose", currentPose);
    Logger.recordOutput("VisionGuidedAuto/VisionPose", visionPose);
    
    return true;
  }
}
