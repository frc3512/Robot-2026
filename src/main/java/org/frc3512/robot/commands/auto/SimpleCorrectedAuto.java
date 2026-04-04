package org.frc3512.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc3512.robot.RobotContainer;

/**
 * Simple wrapper that adds vision guidance to any PathPlanner auto command.
 * This replaces the complex CorrectedAuto with a streamlined approach.
 */
public class SimpleCorrectedAuto extends Command {
  
  private final VisionGuidedAuto visionGuidedAuto;
  
  /**
   * Creates a vision-guided version of any auto command.
   * 
   * @param originalAuto The original PathPlanner auto command
   * @param robotContainer Access to robot subsystems
   */
  public SimpleCorrectedAuto(Command originalAuto, RobotContainer robotContainer) {
    this.visionGuidedAuto = new VisionGuidedAuto(
        robotContainer.getDrive(),
        robotContainer.getVision(),
        originalAuto
    );
  }
  
  @Override
  public void initialize() {
    visionGuidedAuto.initialize();
  }
  
  @Override
  public void execute() {
    visionGuidedAuto.execute();
  }
  
  @Override
  public void end(boolean interrupted) {
    visionGuidedAuto.end(interrupted);
  }
  
  @Override
  public boolean isFinished() {
    return visionGuidedAuto.isFinished();
  }
  
  /**
   * Factory method to create a vision-guided auto command.
   * This is the recommended way to add vision correction to autos.
   */
  public static Command withVisionGuidance(Command originalAuto, RobotContainer robotContainer) {
    return new SimpleCorrectedAuto(originalAuto, robotContainer);
  }
}
