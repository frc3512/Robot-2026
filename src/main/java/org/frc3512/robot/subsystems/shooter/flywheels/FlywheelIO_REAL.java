package org.frc3512.robot.subsystems.shooter.flywheels;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.List;

// Credit to the 2026 WCP CC for the majority of this code
public class FlywheelIO_REAL implements FlywheelIO {
  private static final AngularVelocity kVelocityTolerance = RPM.of(150);

  private final TalonFX leftMotor, middleMotor, rightMotor;
  private final List<TalonFX> motors;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private double wantedRPM = 0.0;

  public FlywheelIO_REAL() {
    leftMotor = new TalonFX(FlywheelConstants.leftMotorID);
    middleMotor = new TalonFX(FlywheelConstants.middleMotorID);
    rightMotor = new TalonFX(FlywheelConstants.rightMotorID);

    motors = List.of(leftMotor, middleMotor, rightMotor);

    FlywheelConstants.configureMotor(leftMotor, InvertedValue.Clockwise_Positive);
    FlywheelConstants.configureMotor(middleMotor, InvertedValue.Clockwise_Positive);
    FlywheelConstants.configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

    leftMotor.optimizeBusUtilization();
    middleMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();
  }

  @Override
  public void setRPM(double rpm) {
    wantedRPM = rpm;
  }

  @Override
  public void stop() {
    setPercentOutput(0.0);
  }

  @Override
  public boolean isVelocityWithinTolerance() {
    return motors.stream()
        .allMatch(
            motor -> {
              final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
              final AngularVelocity currentVelocity = motor.getVelocity().getValue();
              final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
              return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
            });
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    for (final TalonFX motor : motors) {
      motor.setControl(velocityRequest.withVelocity(RPM.of(wantedRPM)));
    }

    inputs.rpmSetpoint = velocityRequest.Velocity * 60.0;

    inputs.leftVelocity = leftMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.middleVelocity = middleMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.rightVelocity = rightMotor.getVelocity().getValueAsDouble() * 60.0;

    inputs.leftAppliedVolts = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.middleAppliedVolts = middleMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightAppliedVolts = rightMotor.getStatorCurrent().getValueAsDouble();

    inputs.isVelocityWithinTolerance = isVelocityWithinTolerance();
  }
}
