package org.frc3512.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc3512.robot.RobotContainer;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.vision.Vision;

/**
 * A wrapper command that adds vision correction to any PathPlanner autonomous command. This
 * demonstrates how to integrate vision-based pose correction into existing autos.
 */
public class CorrectedAuto extends SequentialCommandGroup {

  /**
   * Creates a vision-corrected version of a PathPlanner auto command. Adds vision correction at key
   * waypoints and provides continuous fusion during path following.
   *
   * @param originalAuto The original PathPlanner auto command
   * @param robotContainer Access to robot subsystems and vision correction methods
   */
  public CorrectedAuto(Command originalAuto, RobotContainer robotContainer) {
    // Get the drive and vision subsystems
    Drive drive = robotContainer.getDrive();
    Vision vision = robotContainer.getVision();

    // Wrap the original auto with vision assistance for continuous correction
    Command visionAssistedAuto = new AssistedAuto(drive, vision, originalAuto, 0.3);

    // Add the vision-assisted auto as the main command
    addCommands(visionAssistedAuto);
  }

  /**
   * Creates a vision-corrected auto with specific waypoint corrections. This is useful for autos
   * that have known critical positions.
   *
   * @param originalAuto The original PathPlanner auto command
   * @param robotContainer Access to robot subsystems and vision correction methods
   * @param correctAtStart Whether to correct pose at the start
   * @param correctAtNZ Whether to correct at NZ (Note Zone) positions
   * @param correctAtHP Whether to correct at HP (Human Player) positions
   */
  public CorrectedAuto(
      Command originalAuto,
      RobotContainer robotContainer,
      boolean correctAtStart,
      boolean correctAtNZ,
      boolean correctAtHP) {

    Drive drive = robotContainer.getDrive();
    Vision vision = robotContainer.getVision();

    // Build the sequence with vision corrections at key points
    if (correctAtStart) {
      addCommands(new PoseCorrector(drive, vision, drive.getPose(), 0.15, 10.0));
    }

    // Wrap the main auto with vision assistance
    Command visionAssistedAuto = new AssistedAuto(drive, vision, originalAuto, 0.3);
    addCommands(visionAssistedAuto);

    // Note: For specific waypoint corrections during the auto, you would need
    // to modify the .auto files to include the named commands like:
    // - "CorrectAtNZLeft" for left note zone
    // - "CorrectAtNZRight" for right note zone
    // - "VerifyAtHP" for human player station verification
  }

  /**
   * Factory method to create a vision-corrected version of any auto command. This is the
   * recommended way to add vision correction to existing autos.
   */
  public static Command withVisionCorrection(Command originalAuto, RobotContainer robotContainer) {
    return new CorrectedAuto(originalAuto, robotContainer);
  }

  /** Factory method to create a vision-corrected auto with specific waypoint corrections. */
  public static Command withVisionCorrection(
      Command originalAuto,
      RobotContainer robotContainer,
      boolean correctAtStart,
      boolean correctAtNZ,
      boolean correctAtHP) {
    return new CorrectedAuto(
        originalAuto, robotContainer, correctAtStart, correctAtNZ, correctAtHP);
  }
}
