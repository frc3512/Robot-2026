package org.frc3512.robot.subsystems.shooter.hood;

import org.frc3512.robot.constants.MechanicalConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hood subsystem with one Kraken X44 motor for angle control.
 * Base angle of 10 degrees, max angle of 45 degrees.
 */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    Logger.recordOutput("Hood/PositionDegrees", inputs.positionDegrees);
    Logger.recordOutput("Hood/VelocityDegreesPerSec", inputs.velocityDegreesPerSec);
    Logger.recordOutput("Hood/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Hood/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("Hood/TempCelsius", inputs.tempCelsius);
  }

  /**
   * Sets hood angle in degrees.
   * @param degrees Desired hood angle in degrees (clamped to mechanical limits)
   */
  public void setAngle(double degrees) {
    io.setAngle(degrees);
    Logger.recordOutput("Hood/SetpointDegrees", degrees);
  }

  /**
   * Sets hood open loop output.
   * @param output Output voltage (-12 to 12)
   */
  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
    Logger.recordOutput("Hood/OpenLoopOutput", output);
  }

  /**
   * Stops the hood motor.
   */
  public void stop() {
    io.stop();
    Logger.recordOutput("Hood/Stopped", true);
  }

  /**
   * Gets the hood position in degrees.
   */
  @AutoLogOutput
  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  /**
   * Gets the hood velocity in degrees per second.
   */
  @AutoLogOutput
  public double getVelocityDegreesPerSec() {
    return inputs.velocityDegreesPerSec;
  }

  /**
   * Gets the hood motor current in amps.
   */
  @AutoLogOutput
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  /**
   * Gets the hood motor temperature in Celsius.
   */
  @AutoLogOutput
  public double getTempCelsius() {
    return inputs.tempCelsius;
  }

  /**
   * Checks if the hood motor is connected.
   */
  public boolean isConnected() {
    return inputs.connected;
  }

  /**
   * Checks if the hood is at the minimum angle.
   */
  public boolean isAtMinAngle() {
    return Math.abs(inputs.positionDegrees - MechanicalConstants.Hood.MIN_ANGLE_DEGREES) < 1.0;
  }

  /**
   * Checks if the hood is at the maximum angle.
   */
  public boolean isAtMaxAngle() {
    return Math.abs(inputs.positionDegrees - MechanicalConstants.Hood.MAX_ANGLE_DEGREES) < 1.0;
  }
}
