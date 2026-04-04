package org.frc3512.robot.subsystems.shooter.drum;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.List;

// Credit to the 2026 WCP CC for the majority of this code
public class FlywheelIO_REAL implements FlywheelIO {
  private static final AngularVelocity kVelocityTolerance = RPM.of(75);

  private final TalonFX mainMotor, secondMotor, tertiaryMotor;
  private final List<TalonFX> motors;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private double wantedRPM = 0.0;

  public FlywheelIO_REAL() {
    mainMotor = new TalonFX(FlywheelConstants.mainMotor);
    secondMotor = new TalonFX(FlywheelConstants.secondaryMotor);
    tertiaryMotor = new TalonFX(FlywheelConstants.tertiaryMotor);

    motors = List.of(mainMotor, secondMotor, tertiaryMotor);

    // TODO: Update these values with the new shooter

    FlywheelConstants.configureMotor(mainMotor, InvertedValue.Clockwise_Positive);
    FlywheelConstants.configureMotor(secondMotor, InvertedValue.Clockwise_Positive);
    FlywheelConstants.configureMotor(tertiaryMotor, InvertedValue.CounterClockwise_Positive);

    mainMotor.optimizeBusUtilization();
    secondMotor.optimizeBusUtilization();
    tertiaryMotor.optimizeBusUtilization();
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

    inputs.leftVelocity = mainMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.middleVelocity = secondMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.rightVelocity = tertiaryMotor.getVelocity().getValueAsDouble() * 60.0;

    inputs.leftAppliedVolts = mainMotor.getStatorCurrent().getValueAsDouble();
    inputs.middleAppliedVolts = secondMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightAppliedVolts = tertiaryMotor.getStatorCurrent().getValueAsDouble();

    inputs.isVelocityWithinTolerance = isVelocityWithinTolerance();
  }
}
