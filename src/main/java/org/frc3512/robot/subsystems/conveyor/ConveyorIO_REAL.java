package org.frc3512.robot.subsystems.conveyor;

import com.ctre.phoenix6.hardware.TalonFX;

public class ConveyorIO_REAL implements ConveyorIO {

  private TalonFX hopper;

  public ConveyorIO_REAL() {
    hopper = new TalonFX(ConveyorConstants.hopperMotorID);

    hopper.getConfigurator().apply(ConveyorConstants.hopper);
  }

  @Override
  public void setHopper(double speed) {
    hopper.set(speed);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    // Log velocity
    inputs.hopperVelocity = hopper.getVelocity().getValueAsDouble() * 60.0;

    // Log applied volts
    inputs.hopperVolts = hopper.getStatorCurrent().getValueAsDouble();
  }
}
