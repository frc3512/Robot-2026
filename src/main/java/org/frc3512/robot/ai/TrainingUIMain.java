package org.frc3512.robot.ai;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Main robot class that starts the standalone training UI.
 * This provides a complete working training interface without complex dependencies.
 */
public class TrainingUIMain extends TimedRobot {
  private StandaloneTrainingUI trainingUI;

  @Override
  public void robotInit() {
    // Start the training UI on port 8080
    trainingUI = new StandaloneTrainingUI(8080);
    trainingUI.start();
    
    System.out.println("========================================");
    System.out.println("AI SHOOTING TRAINING SYSTEM");
    System.out.println("========================================");
    System.out.println("Training UI is now running!");
    System.out.println("Open your browser and go to: http://localhost:8080");
    System.out.println("========================================");
  }

  @Override
  public void robotPeriodic() {
    // Periodic updates if needed
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  // No end() method needed - cleanup can be done in disabledInit if needed
}
