package org.frc3512.robot.subsystems.shooter.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

public class FeederIO_REAL implements FeederIO {

  private TalonFX feeder, secondaryFeeder;

  public FeederIO_REAL() {
    feeder = new TalonFX(FeederConstants.feederMotorID);
    secondaryFeeder = new TalonFX(FeederConstants.secondaryFeederMotorID);

    feeder.getConfigurator().apply(FeederConstants.feeder);
    secondaryFeeder.getConfigurator().apply(FeederConstants.secondaryFeeder);

    feeder.optimizeBusUtilization();
    secondaryFeeder.optimizeBusUtilization();
  }

  @Override
  public void setFeeder(double speed) {
    System.out.println("DEBUG: FeederIO_REAL.setFeeder called with speed: " + speed);
    feeder.set(speed);
    secondaryFeeder.set(speed);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Log velocity
    inputs.feederVelocity = feeder.getVelocity().getValueAsDouble() * 60.0;

    // Log applied volts
    inputs.feederAppliedOutput = feeder.getStatorCurrent().getValueAsDouble();

    // Log motor temperature
    inputs.motorTemp = (feeder.getDeviceTemp().getValueAsDouble() * 1.8) + 32.0;
  }
}
