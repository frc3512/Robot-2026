package org.frc3512.robot.subsystems.shooter.drum;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of drum IO.
 * Simulation is always based on voltage control.
 */
public class DrumIOSim implements DrumIO {
  private static final double KP = 0.1;
  private static final double KV = 0.12;
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim motor1Sim;
  private final DCMotorSim motor2Sim;
  private final DCMotorSim motor3Sim;

  private boolean closedLoop = false;
  private PIDController controller1 = new PIDController(KP, 0, 0);
  private PIDController controller2 = new PIDController(KP, 0, 0);
  private PIDController controller3 = new PIDController(KP, 0, 0);
  private double ffVolts = 0.0;
  private double motor1AppliedVolts = 0.0;
  private double motor2AppliedVolts = 0.0;
  private double motor3AppliedVolts = 0.0;

  public DrumIOSim() {
    // Create motor sims
    motor1Sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, 1.0),
        GEARBOX);
    motor2Sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, 1.0),
        GEARBOX);
    motor3Sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, 1.0),
        GEARBOX);
  }

  @Override
  public void updateInputs(DrumIO.DrumIOInputs inputs) {
    // Update motor 1
    inputs.motor1Connected = true;
    inputs.motor1VelocityRPS = motor1Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.motor1AppliedVolts = motor1AppliedVolts;
    inputs.motor1CurrentAmps = motor1Sim.getCurrentDrawAmps();
    inputs.motor1TempCelsius = 25.0; // Simulated temperature

    // Update motor 2
    inputs.motor2Connected = true;
    inputs.motor2VelocityRPS = motor2Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.motor2AppliedVolts = motor2AppliedVolts;
    inputs.motor2CurrentAmps = motor2Sim.getCurrentDrawAmps();
    inputs.motor2TempCelsius = 25.0; // Simulated temperature

    // Update motor 3
    inputs.motor3Connected = true;
    inputs.motor3VelocityRPS = motor3Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.motor3AppliedVolts = motor3AppliedVolts;
    inputs.motor3CurrentAmps = motor3Sim.getCurrentDrawAmps();
    inputs.motor3TempCelsius = 25.0; // Simulated temperature
  }

  @Override
  public void setVelocity(double mechanismRPM) {
    closedLoop = true;
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Drum.GEAR_RATIO;
    ffVolts = KV * motorRPS;
    controller1.setSetpoint(motorRPS);
    controller2.setSetpoint(motorRPS);
    controller3.setSetpoint(motorRPS);
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    motor1AppliedVolts = output;
    motor2AppliedVolts = output;
    motor3AppliedVolts = output;
  }

  @Override
  public void stop() {
    closedLoop = false;
    motor1AppliedVolts = 0.0;
    motor2AppliedVolts = 0.0;
    motor3AppliedVolts = 0.0;
  }

  /** Runs simulation, updating all motor states. */
  public void simulate(double dtSeconds, double supplyVoltage) {
    if (closedLoop) {
      double motor1Volts = MathUtil.clamp(
          ffVolts + controller1.calculate(motor1Sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
      double motor2Volts = MathUtil.clamp(
          ffVolts + controller2.calculate(motor2Sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
      double motor3Volts = MathUtil.clamp(
          ffVolts + controller3.calculate(motor3Sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
      
      motor1AppliedVolts = motor1Volts;
      motor2AppliedVolts = motor2Volts;
      motor3AppliedVolts = motor3Volts;
      
      motor1Sim.setInputVoltage(motor1Volts);
      motor2Sim.setInputVoltage(motor2Volts);
      motor3Sim.setInputVoltage(motor3Volts);
    } else {
      motor1Sim.setInputVoltage(motor1AppliedVolts);
      motor2Sim.setInputVoltage(motor2AppliedVolts);
      motor3Sim.setInputVoltage(motor3AppliedVolts);
    }

    // Update all simulations
    motor1Sim.update(dtSeconds);
    motor2Sim.update(dtSeconds);
    motor3Sim.update(dtSeconds);
  }
}
