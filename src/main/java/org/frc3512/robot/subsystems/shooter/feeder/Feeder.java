package org.frc3512.robot.subsystems.shooter.feeder;

import org.frc3512.robot.constants.CurrentLimits;
import org.frc3512.robot.constants.MechanicalConstants;
import org.frc3512.robot.subsystems.states.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Feeder subsystem with two Kraken X60 motors.
 * One feeds into shooter tower, one booster below drum.
 */
public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIO.FeederIOInputs inputs = new FeederIO.FeederIOInputs();
  
  // State-based current limit management
  private RobotState currentState = RobotState.IDLE;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    // Log average values for both motors
    double avgVelocityRPS = (inputs.feedVelocityRPS + inputs.boosterVelocityRPS) / 2.0;
    double avgCurrentAmps = (inputs.feedCurrentAmps + inputs.boosterCurrentAmps) / 2.0;
    double avgTempCelsius = (inputs.feedTempCelsius + inputs.boosterTempCelsius) / 2.0;
    
    Logger.recordOutput("Feeder/AverageVelocityRPS", avgVelocityRPS);
    Logger.recordOutput("Feeder/AverageCurrentAmps", avgCurrentAmps);
    Logger.recordOutput("Feeder/AverageTempCelsius", avgTempCelsius);
    Logger.recordOutput("Feeder/FeedVelocityRPS", inputs.feedVelocityRPS);
    Logger.recordOutput("Feeder/BoosterVelocityRPS", inputs.boosterVelocityRPS);
  }

  /**
   * Sets the robot state and updates current limits accordingly.
   * This should be called by the main state machine.
   */
  public void setRobotState(RobotState state) {
    if (currentState != state) {
      currentState = state;
      updateCurrentLimits();
      Logger.recordOutput("Feeder/RobotState", state.toString());
    }
  }

  /**
   * Updates current limits for all feeder motors based on current robot state.
   */
  private void updateCurrentLimits() {
    double supplyLimit = CurrentLimits.Feeder.getSupplyForState(currentState);
    double statorLimit = CurrentLimits.Feeder.getStatorForState(currentState);
    io.setCurrentLimits(supplyLimit, statorLimit);
    Logger.recordOutput("Feeder/SupplyCurrentLimit", supplyLimit);
    Logger.recordOutput("Feeder/StatorCurrentLimit", statorLimit);
  }

  /**
   * Gets the current robot state.
   */
  public RobotState getRobotState() {
    return currentState;
  }

  /**
   * Sets feed motor velocity in mechanism RPM.
   * @param mechanismRPM Desired feed speed in RPM (mechanism, not motor)
   */
  public void setFeedVelocity(double mechanismRPM) {
    io.setFeedVelocity(mechanismRPM);
    Logger.recordOutput("Feeder/FeedSetpointRPM", mechanismRPM);
  }

  /**
   * Sets feed motor open loop output.
   * @param output Output voltage (-12 to 12)
   */
  public void setFeedOpenLoop(double output) {
    io.setFeedOpenLoop(output);
    Logger.recordOutput("Feeder/FeedOpenLoopOutput", output);
  }

  /**
   * Sets booster motor velocity in mechanism RPM.
   * @param mechanismRPM Desired booster speed in RPM (mechanism, not motor)
   */
  public void setBoosterVelocity(double mechanismRPM) {
    io.setBoosterVelocity(mechanismRPM);
    Logger.recordOutput("Feeder/BoosterSetpointRPM", mechanismRPM);
  }

  /**
   * Sets booster motor open loop output.
   * @param output Output voltage (-12 to 12)
   */
  public void setBoosterOpenLoop(double output) {
    io.setBoosterOpenLoop(output);
    Logger.recordOutput("Feeder/BoosterOpenLoopOutput", output);
  }

  /**
   * Stops all feeder motors.
   */
  public void stop() {
    io.stop();
    Logger.recordOutput("Feeder/Stopped", true);
  }

  /**
   * Gets the average feeder velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getAverageVelocityRPM() {
    double avgRPS = (inputs.feedVelocityRPS + inputs.boosterVelocityRPS) / 2.0;
    return avgRPS * 60.0 / MechanicalConstants.Feeder.FEED_MOTOR_RPM_PER_MECHANISM_RPM;
  }

  /**
   * Gets the average feeder current in amps.
   */
  @AutoLogOutput
  public double getAverageCurrentAmps() {
    return (inputs.feedCurrentAmps + inputs.boosterCurrentAmps) / 2.0;
  }

  /**
   * Gets the average feeder temperature in Celsius.
   */
  @AutoLogOutput
  public double getAverageTempCelsius() {
    return (inputs.feedTempCelsius + inputs.boosterTempCelsius) / 2.0;
  }

  /**
   * Gets the feed motor velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getFeedVelocityRPM() {
    return inputs.feedVelocityRPS * 60.0 / MechanicalConstants.Feeder.FEED_MOTOR_RPM_PER_MECHANISM_RPM;
  }

  /**
   * Gets the booster motor velocity in mechanism RPM.
   */
  @AutoLogOutput
  public double getBoosterVelocityRPM() {
    return inputs.boosterVelocityRPS * 60.0 / MechanicalConstants.Feeder.FEED_MOTOR_RPM_PER_MECHANISM_RPM;
  }

  /**
   * Gets the feed motor current in amps.
   */
  @AutoLogOutput
  public double getFeedCurrentAmps() {
    return inputs.feedCurrentAmps;
  }

  /**
   * Gets the booster motor current in amps.
   */
  @AutoLogOutput
  public double getBoosterCurrentAmps() {
    return inputs.boosterCurrentAmps;
  }

  /**
   * Gets the feed motor temperature in Celsius.
   */
  @AutoLogOutput
  public double getFeedTempCelsius() {
    return inputs.feedTempCelsius;
  }

  /**
   * Gets the booster motor temperature in Celsius.
   */
  @AutoLogOutput
  public double getBoosterTempCelsius() {
    return inputs.boosterTempCelsius;
  }

  /**
   * Checks if both feeder motors are connected.
   */
  public boolean areMotorsConnected() {
    return inputs.feedConnected && inputs.boosterConnected;
  }

  /**
   * Checks if feed motor is connected.
   */
  public boolean isFeedConnected() {
    return inputs.feedConnected;
  }

  /**
   * Checks if booster motor is connected.
   */
  public boolean isBoosterConnected() {
    return inputs.boosterConnected;
  }
}
