package org.frc3512.robot.subsystems.hopper.intake;

import org.frc3512.robot.constants.CurrentLimits;
import org.frc3512.robot.constants.MechanicalConstants;
import org.frc3512.robot.subsystems.states.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem with two roller motors and one extension motor.
 * Uses three Kraken X60 motors: two for rollers, one for linear slide extension.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  
  // State-based current limit management
  private RobotState currentState = RobotState.IDLE;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    // Log average roller velocity and current
    double avgRollerVelocityRPS = (inputs.roller1VelocityRPS + inputs.roller2VelocityRPS) / 2.0;
    double avgRollerCurrentAmps = (inputs.roller1CurrentAmps + inputs.roller2CurrentAmps) / 2.0;
    
    Logger.recordOutput("Intake/AverageRollerVelocityRPS", avgRollerVelocityRPS);
    Logger.recordOutput("Intake/AverageRollerCurrentAmps", avgRollerCurrentAmps);
    Logger.recordOutput("Intake/ExtensionPositionInches", inputs.extensionPositionInches);
    Logger.recordOutput("Intake/ExtensionVelocityInchesPerSec", inputs.extensionVelocityInchesPerSec);
    Logger.recordOutput("Intake/ExtensionCurrentAmps", inputs.extensionCurrentAmps);
  }

  /**
   * Sets the robot state and updates current limits accordingly.
   * This should be called by the main state machine.
   */
  public void setRobotState(RobotState state) {
    if (currentState != state) {
      currentState = state;
      updateCurrentLimits();
      Logger.recordOutput("Intake/RobotState", state.toString());
    }
  }

  /**
   * Updates current limits for all intake motors based on current robot state.
   */
  private void updateCurrentLimits() {
    double supplyLimit = CurrentLimits.Intake.getSupplyForState(currentState);
    double statorLimit = CurrentLimits.Intake.getStatorForState(currentState);
    io.setCurrentLimits(supplyLimit, statorLimit);
    Logger.recordOutput("Intake/SupplyCurrentLimit", supplyLimit);
    Logger.recordOutput("Intake/StatorCurrentLimit", statorLimit);
  }

  /**
   * Gets the current robot state.
   */
  public RobotState getRobotState() {
    return currentState;
  }

  /**
   * Sets roller velocity in mechanism RPM.
   * @param mechanismRPM Desired roller speed in RPM (mechanism, not motor)
   */
  public void setRollerVelocity(double mechanismRPM) {
    io.setRollerVelocity(mechanismRPM);
    Logger.recordOutput("Intake/RollerSetpointRPM", mechanismRPM);
  }

  /**
   * Sets roller open loop output.
   * @param output Output voltage (-12 to 12)
   */
  public void setRollerOpenLoop(double output) {
    io.setRollerOpenLoop(output);
    Logger.recordOutput("Intake/RollerOpenLoopOutput", output);
  }

  /**
   * Sets extension position in inches.
   * @param positionInches Desired extension position in inches
   */
  public void setExtensionPosition(double positionInches) {
    io.setExtensionPosition(positionInches);
    Logger.recordOutput("Intake/ExtensionSetpointInches", positionInches);
  }

  /**
   * Sets extension open loop output.
   * @param output Output voltage (-12 to 12)
   */
  public void setExtensionOpenLoop(double output) {
    io.setExtensionOpenLoop(output);
    Logger.recordOutput("Intake/ExtensionOpenLoopOutput", output);
  }

  /**
   * Stops all intake motors.
   */
  public void stop() {
    io.stop();
    Logger.recordOutput("Intake/Stopped", true);
  }

  /**
   * Extends the intake to the extended position.
   */
  public void extend() {
    setExtensionPosition(MechanicalConstants.Intake.EXTENDED_POSITION_INCHES);
  }

  /**
   * Retracts the intake to the retracted position.
   */
  public void retract() {
    setExtensionPosition(MechanicalConstants.Intake.RETRACTED_POSITION_INCHES);
  }

  /**
   * Checks if the intake is at the extended position.
   */
  public boolean isExtended() {
    return Math.abs(inputs.extensionPositionInches - MechanicalConstants.Intake.EXTENDED_POSITION_INCHES) < 0.5;
  }

  /**
   * Checks if the intake is at the retracted position.
   */
  public boolean isRetracted() {
    return Math.abs(inputs.extensionPositionInches - MechanicalConstants.Intake.RETRACTED_POSITION_INCHES) < 0.5;
  }

  /**
   * Gets the average roller velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getAverageRollerVelocityRPM() {
    double avgRPS = (inputs.roller1VelocityRPS + inputs.roller2VelocityRPS) / 2.0;
    return avgRPS * 60.0 / MechanicalConstants.Intake.ROLLER_MOTOR_RPM_PER_MECHANISM_RPM;
  }

  /**
   * Gets the average roller current in amps.
   */
  @AutoLogOutput
  public double getAverageRollerCurrentAmps() {
    return (inputs.roller1CurrentAmps + inputs.roller2CurrentAmps) / 2.0;
  }

  /**
   * Gets the extension position in inches.
   */
  @AutoLogOutput
  public double getExtensionPositionInches() {
    return inputs.extensionPositionInches;
  }

  /**
   * Gets the extension current in amps.
   */
  @AutoLogOutput
  public double getExtensionCurrentAmps() {
    return inputs.extensionCurrentAmps;
  }

  /**
   * Gets roller 1 temperature in Celsius.
   */
  @AutoLogOutput
  public double getRoller1TempCelsius() {
    return inputs.roller1TempCelsius;
  }

  /**
   * Gets roller 2 temperature in Celsius.
   */
  @AutoLogOutput
  public double getRoller2TempCelsius() {
    return inputs.roller2TempCelsius;
  }

  /**
   * Gets extension motor temperature in Celsius.
   */
  @AutoLogOutput
  public double getExtensionTempCelsius() {
    return inputs.extensionTempCelsius;
  }

  /**
   * Checks if both roller motors are connected.
   */
  public boolean areRollersConnected() {
    return inputs.roller1Connected && inputs.roller2Connected;
  }

  /**
   * Checks if extension motor is connected.
   */
  public boolean isExtensionConnected() {
    return inputs.extensionConnected;
  }
}
