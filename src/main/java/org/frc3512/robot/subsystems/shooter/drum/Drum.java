package org.frc3512.robot.subsystems.shooter.drum;

import org.frc3512.robot.constants.CurrentLimits;
import org.frc3512.robot.constants.MechanicalConstants;
import org.frc3512.robot.subsystems.states.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Drum subsystem with three Kraken X60 motors.
 * Two motors on one side, one motor on other side.
 * 1:1 gear ratio, 2-inch drum radius.
 */
public class Drum extends SubsystemBase {
  private final DrumIO io;
  private final DrumIO.DrumIOInputs inputs = new DrumIO.DrumIOInputs();
  
  // State-based current limit management
  private RobotState currentState = RobotState.IDLE;

  public Drum(DrumIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    // Log average values for all three motors
    double avgVelocityRPS = (inputs.motor1VelocityRPS + inputs.motor2VelocityRPS + inputs.motor3VelocityRPS) / 3.0;
    double avgCurrentAmps = (inputs.motor1CurrentAmps + inputs.motor2CurrentAmps + inputs.motor3CurrentAmps) / 3.0;
    double avgTempCelsius = (inputs.motor1TempCelsius + inputs.motor2TempCelsius + inputs.motor3TempCelsius) / 3.0;
    
    Logger.recordOutput("Drum/AverageVelocityRPS", avgVelocityRPS);
    Logger.recordOutput("Drum/AverageCurrentAmps", avgCurrentAmps);
    Logger.recordOutput("Drum/AverageTempCelsius", avgTempCelsius);
    Logger.recordOutput("Drum/Motor1VelocityRPS", inputs.motor1VelocityRPS);
    Logger.recordOutput("Drum/Motor2VelocityRPS", inputs.motor2VelocityRPS);
    Logger.recordOutput("Drum/Motor3VelocityRPS", inputs.motor3VelocityRPS);
  }

  /**
   * Sets the robot state and updates current limits accordingly.
   * This should be called by the main state machine.
   */
  public void setRobotState(RobotState state) {
    if (currentState != state) {
      currentState = state;
      updateCurrentLimits();
      Logger.recordOutput("Drum/RobotState", state.toString());
    }
  }

  /**
   * Updates current limits for all drum motors based on current robot state.
   */
  private void updateCurrentLimits() {
    double supplyLimit = CurrentLimits.Drum.getSupplyForState(currentState);
    double statorLimit = CurrentLimits.Drum.getStatorForState(currentState);
    io.setCurrentLimits(supplyLimit, statorLimit);
    Logger.recordOutput("Drum/SupplyCurrentLimit", supplyLimit);
    Logger.recordOutput("Drum/StatorCurrentLimit", statorLimit);
  }

  /**
   * Gets the current robot state.
   */
  public RobotState getRobotState() {
    return currentState;
  }

  /**
   * Sets drum velocity in mechanism RPM.
   * @param mechanismRPM Desired drum speed in RPM (mechanism, not motor)
   */
  public void setVelocity(double mechanismRPM) {
    io.setVelocity(mechanismRPM);
    Logger.recordOutput("Drum/SetpointRPM", mechanismRPM);
  }

  /**
   * Sets drum open loop output.
   * @param output Output voltage (-12 to 12)
   */
  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
    Logger.recordOutput("Drum/OpenLoopOutput", output);
  }

  /**
   * Stops all drum motors.
   */
  public void stop() {
    io.stop();
    Logger.recordOutput("Drum/Stopped", true);
  }

  /**
   * Gets the average drum velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getAverageVelocityRPM() {
    double avgRPS = (inputs.motor1VelocityRPS + inputs.motor2VelocityRPS + inputs.motor3VelocityRPS) / 3.0;
    return avgRPS * 60.0 / MechanicalConstants.Drum.GEAR_RATIO;
  }

  /**
   * Gets the average drum current in amps.
   */
  @AutoLogOutput
  public double getAverageCurrentAmps() {
    return (inputs.motor1CurrentAmps + inputs.motor2CurrentAmps + inputs.motor3CurrentAmps) / 3.0;
  }

  /**
   * Gets the average drum temperature in Celsius.
   */
  @AutoLogOutput
  public double getAverageTempCelsius() {
    return (inputs.motor1TempCelsius + inputs.motor2TempCelsius + inputs.motor3TempCelsius) / 3.0;
  }

  /**
   * Gets the average drum surface speed in inches per second.
   */
  @AutoLogOutput
  public double getSurfaceSpeedInchesPerSec() {
    double avgRPS = (inputs.motor1VelocityRPS + inputs.motor2VelocityRPS + inputs.motor3VelocityRPS) / 3.0;
    return avgRPS * org.frc3512.robot.constants.MechanicalConstants.Drum.RPS_TO_INCHES_PER_SECOND;
  }

  /**
   * Gets the drum surface speed in RPM.
   */
  @AutoLogOutput
  public double getSurfaceSpeedRPM() {
    return getAverageVelocityRPM();
  }

  /**
   * Gets motor 1 velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getMotor1VelocityRPM() {
    return inputs.motor1VelocityRPS * 60.0 / MechanicalConstants.Drum.GEAR_RATIO;
  }

  /**
   * Gets motor 2 velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getMotor2VelocityRPM() {
    return inputs.motor2VelocityRPS * 60.0 / MechanicalConstants.Drum.GEAR_RATIO;
  }

  /**
   * Gets motor 3 velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getMotor3VelocityRPM() {
    return inputs.motor3VelocityRPS * 60.0 / MechanicalConstants.Drum.GEAR_RATIO;
  }

  /**
   * Gets motor 1 current in amps.
   */
  @AutoLogOutput
  public double getMotor1CurrentAmps() {
    return inputs.motor1CurrentAmps;
  }

  /**
   * Gets motor 2 current in amps.
   */
  @AutoLogOutput
  public double getMotor2CurrentAmps() {
    return inputs.motor2CurrentAmps;
  }

  /**
   * Gets motor 3 current in amps.
   */
  @AutoLogOutput
  public double getMotor3CurrentAmps() {
    return inputs.motor3CurrentAmps;
  }

  /**
   * Gets motor 1 temperature in Celsius.
   */
  @AutoLogOutput
  public double getMotor1TempCelsius() {
    return inputs.motor1TempCelsius;
  }

  /**
   * Gets motor 2 temperature in Celsius.
   */
  @AutoLogOutput
  public double getMotor2TempCelsius() {
    return inputs.motor2TempCelsius;
  }

  /**
   * Gets motor 3 temperature in Celsius.
   */
  @AutoLogOutput
  public double getMotor3TempCelsius() {
    return inputs.motor3TempCelsius;
  }

  /**
   * Checks if all drum motors are connected.
   */
  public boolean areMotorsConnected() {
    return inputs.motor1Connected && inputs.motor2Connected && inputs.motor3Connected;
  }

  /**
   * Checks if motor 1 is connected.
   */
  public boolean isMotor1Connected() {
    return inputs.motor1Connected;
  }

  /**
   * Checks if motor 2 is connected.
   */
  public boolean isMotor2Connected() {
    return inputs.motor2Connected;
  }

  /**
   * Checks if motor 3 is connected.
   */
  public boolean isMotor3Connected() {
    return inputs.motor3Connected;
  }
}
