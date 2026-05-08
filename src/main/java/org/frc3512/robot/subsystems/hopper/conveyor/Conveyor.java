package org.frc3512.robot.subsystems.hopper.conveyor;

import org.frc3512.robot.constants.CurrentLimits;
import org.frc3512.robot.constants.MechanicalConstants;
import org.frc3512.robot.subsystems.states.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Conveyor subsystem with one Kraken X44 motor driving a roller floor.
 */
public class Conveyor extends SubsystemBase {
  private final ConveyorIO io;
  private final ConveyorIO.ConveyorIOInputs inputs = new ConveyorIO.ConveyorIOInputs();
  
  // State-based current limit management
  private RobotState currentState = RobotState.IDLE;

  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    Logger.recordOutput("Conveyor/VelocityRPS", inputs.velocityRPS);
    Logger.recordOutput("Conveyor/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Conveyor/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("Conveyor/TempCelsius", inputs.tempCelsius);
  }

  /**
   * Sets the robot state and updates current limits accordingly.
   * This should be called by the main state machine.
   */
  public void setRobotState(RobotState state) {
    if (currentState != state) {
      currentState = state;
      updateCurrentLimits();
      Logger.recordOutput("Conveyor/RobotState", state.toString());
    }
  }

  /**
   * Updates current limits for the conveyor motor based on current robot state.
   */
  private void updateCurrentLimits() {
    double supplyLimit = CurrentLimits.Conveyor.getSupplyForState(currentState);
    double statorLimit = CurrentLimits.Conveyor.getStatorForState(currentState);
    io.setCurrentLimits(supplyLimit, statorLimit);
    Logger.recordOutput("Conveyor/SupplyCurrentLimit", supplyLimit);
    Logger.recordOutput("Conveyor/StatorCurrentLimit", statorLimit);
  }

  /**
   * Gets the current robot state.
   */
  public RobotState getRobotState() {
    return currentState;
  }

  /**
   * Sets conveyor velocity in mechanism RPM.
   * @param mechanismRPM Desired conveyor speed in RPM (mechanism, not motor)
   */
  public void setVelocity(double mechanismRPM) {
    io.setVelocity(mechanismRPM);
    Logger.recordOutput("Conveyor/SetpointRPM", mechanismRPM);
  }

  /**
   * Sets conveyor open loop output.
   * @param output Output voltage (-12 to 12)
   */
  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
    Logger.recordOutput("Conveyor/OpenLoopOutput", output);
  }

  /**
   * Stops the conveyor motor.
   */
  public void stop() {
    io.stop();
    Logger.recordOutput("Conveyor/Stopped", true);
  }

  /**
   * Gets the conveyor velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getVelocityRPM() {
    return inputs.velocityRPS * 60.0 / MechanicalConstants.Conveyor.ROLLER_MOTOR_RPS_PER_MECHANISM_RPM;
  }

  /**
   * Gets the conveyor current in amps.
   */
  @AutoLogOutput
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  /**
   * Gets the conveyor motor temperature in Celsius.
   */
  @AutoLogOutput
  public double getTempCelsius() {
    return inputs.tempCelsius;
  }

  /**
   * Checks if the conveyor motor is connected.
   */
  public boolean isConnected() {
    return inputs.connected;
  }
}
