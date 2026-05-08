package org.frc3512.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of hood IO.
 * Simulation is always based on voltage control.
 */
public class HoodIOSim implements HoodIO {
  private static final double KP = 2.0;
  private static final double KI = 0.0;
  private static final double KD = 0.1;
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(1);

  private final DCMotorSim motorSim;

  private boolean closedLoop = false;
  private PIDController controller = new PIDController(KP, KI, KD);
  private double appliedVolts = 0.0;

  public HoodIOSim() {
    // Create motor sim
    motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, 1.0),
        GEARBOX);
  }

  @Override
  public void updateInputs(HoodIO.HoodIOInputs inputs) {
    inputs.connected = true;
    inputs.positionDegrees = motorSim.getAngularPositionRotations() * 360.0;
    inputs.velocityDegreesPerSec = Math.toDegrees(motorSim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0; // Simulated temperature
  }

  @Override
  public void setAngle(double degrees) {
    closedLoop = true;
    double motorRotations = degrees / 360.0;
    controller.setSetpoint(motorRotations);
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  /** Runs simulation, updating motor state. */
  public void simulate(double dtSeconds, double supplyVoltage) {
    if (closedLoop) {
      double volts = MathUtil.clamp(
          controller.calculate(motorSim.getAngularPositionRotations()), -12.0, 12.0);
      appliedVolts = volts;
      motorSim.setInputVoltage(volts);
    } else {
      motorSim.setInputVoltage(appliedVolts);
    }

    motorSim.update(dtSeconds);
  }
}
