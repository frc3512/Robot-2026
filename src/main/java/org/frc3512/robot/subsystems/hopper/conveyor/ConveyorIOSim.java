package org.frc3512.robot.subsystems.hopper.conveyor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of conveyor IO.
 * Simulation is always based on voltage control.
 */
public class ConveyorIOSim implements ConveyorIO {
  private static final double KP = 0.1;
  private static final double KV = 0.12;
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(1);

  private final DCMotorSim motorSim;

  private boolean closedLoop = false;
  private PIDController controller = new PIDController(KP, 0, 0);
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  public ConveyorIOSim() {
    // Create motor sim
    motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, 1.0),
        GEARBOX);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.connected = true;
    inputs.velocityRPS = motorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0; // Simulated temperature
  }

  @Override
  public void setVelocity(double mechanismRPM) {
    closedLoop = true;
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Conveyor.ROLLER_MOTOR_RPS_PER_MECHANISM_RPM;
    ffVolts = KV * motorRPS;
    controller.setSetpoint(motorRPS);
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

  @Override
  public void setCurrentLimit(double currentLimitAmps) {
    // In simulation, current limits are not enforced on simulated motors
    // This method exists for interface compatibility
  }

  /** Runs simulation, updating motor state. */
  public void simulate(double dtSeconds, double supplyVoltage) {
    if (closedLoop) {
      double volts = MathUtil.clamp(
          ffVolts + controller.calculate(motorSim.getAngularVelocityRadPerSec()), -12.0, 12.0);
      appliedVolts = volts;
      motorSim.setInputVoltage(volts);
    } else {
      motorSim.setInputVoltage(appliedVolts);
    }

    motorSim.update(dtSeconds);
  }
}
