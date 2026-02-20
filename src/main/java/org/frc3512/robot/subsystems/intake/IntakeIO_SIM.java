package org.frc3512.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIO_SIM implements IntakeIO {

  private final DCMotorSim rollerSim;
  private final PIDController rollerPid;
  private double rollerTarget = 0.0;

  public IntakeIO_SIM() {
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
            DCMotor.getKrakenX60(1));

    rollerPid = new PIDController(0.1, 0.0, 0.0); // Simple PID for velocity control
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Simulate roller velocity control
    double targetRadPerSec = rollerTarget * 2 * Math.PI / 60.0;
    double rollerVolts =
        rollerPid.calculate(rollerSim.getAngularVelocityRadPerSec(), targetRadPerSec);
    rollerVolts = MathUtil.clamp(rollerVolts, -12.0, 12.0);
    rollerSim.setInputVoltage(rollerVolts);
    rollerSim.update(0.02);

    inputs.rollerVelocity = rollerSim.getAngularVelocityRadPerSec() * 60.0 / (2 * Math.PI);
    inputs.rollerAppliedVolts = rollerVolts;
  }

  @Override
  public void setRollerSpeed(double speed) {
    rollerTarget = speed;
  }
}
