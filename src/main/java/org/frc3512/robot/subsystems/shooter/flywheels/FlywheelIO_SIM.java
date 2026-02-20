package org.frc3512.robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIO_SIM implements FlywheelIO {

  private final DCMotorSim leftSim;
  private final DCMotorSim middleSim;
  private final DCMotorSim rightSim;
  private final PIDController leftPid;
  private final PIDController middlePid;
  private final PIDController rightPid;
  private double setpointRPM = 0.0;

  public FlywheelIO_SIM() {
    // Simulate three motors for the flywheels
    leftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
            DCMotor.getKrakenX60(1));
    middleSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
            DCMotor.getKrakenX60(1));
    rightSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
            DCMotor.getKrakenX60(1));

    leftPid = new PIDController(0.1, 0.0, 0.0); // Simple PID, tune as needed
    middlePid = new PIDController(0.1, 0.0, 0.0);
    rightPid = new PIDController(0.1, 0.0, 0.0);
  }

  // @Override
  // public void setRPMSetpoint(double rpm) {
  //   setpointRPM = rpm;
  // }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Simulate PID control for each motor
    double targetRadPerSec = setpointRPM * 2 * Math.PI / 60.0;

    double leftVolts = leftPid.calculate(leftSim.getAngularVelocityRadPerSec(), targetRadPerSec);
    leftVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    leftSim.setInputVoltage(leftVolts);
    leftSim.update(0.02);

    double middleVolts =
        middlePid.calculate(middleSim.getAngularVelocityRadPerSec(), targetRadPerSec);
    middleVolts = MathUtil.clamp(middleVolts, -12.0, 12.0);
    middleSim.setInputVoltage(middleVolts);
    middleSim.update(0.02);

    double rightVolts = rightPid.calculate(rightSim.getAngularVelocityRadPerSec(), targetRadPerSec);
    rightVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    rightSim.setInputVoltage(rightVolts);
    rightSim.update(0.02);

    inputs.leftVelocity = leftSim.getAngularVelocityRadPerSec() * 60.0 / (2 * Math.PI);
    inputs.middleVelocity = middleSim.getAngularVelocityRadPerSec() * 60.0 / (2 * Math.PI);
    inputs.rightVelocity = rightSim.getAngularVelocityRadPerSec() * 60.0 / (2 * Math.PI);

    inputs.leftAppliedVolts = leftVolts;
    inputs.middleAppliedVolts = middleVolts;
    inputs.rightAppliedVolts = rightVolts;

    inputs.rpmSetpoint = setpointRPM;
  }
}
