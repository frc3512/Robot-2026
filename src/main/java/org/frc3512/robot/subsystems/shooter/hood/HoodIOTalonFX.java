package org.frc3512.robot.subsystems.shooter.hood;

import static org.frc3512.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.frc3512.robot.constants.MechanicalConstants;

/**
 * Hood IO implementation for Talon FX motor controller.
 * One Kraken X44 motor for angle control.
 */
public class HoodIOTalonFX implements HoodIO {
  // Hardware object
  private final TalonFX motor;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  // Status signals (declared after constructor to reference initialized field)
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temp;

  public HoodIOTalonFX(int motorId) {
    motor = new TalonFX(motorId);

    // Initialize status signals after hardware object is created
    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();
    temp = motor.getDeviceTemp();

    // Configure motor
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = org.frc3512.robot.constants.CurrentLimits.Hood.LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = 2.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.1;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    // Optimize status signal rates
    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        position,
        velocity,
        appliedVolts,
        current,
        temp);
  }

  @Override
  public void updateInputs(HoodIO.HoodIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, temp).isOK();
    inputs.positionDegrees = position.getValueAsDouble() * 360.0;
    inputs.velocityDegreesPerSec = Math.toDegrees(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setAngle(double degrees) {
    // Clamp angle to mechanical limits
    double clampedDegrees = Math.max(
        MechanicalConstants.Hood.MIN_ANGLE_DEGREES,
        Math.min(MechanicalConstants.Hood.MAX_ANGLE_DEGREES, degrees));
    
    // Convert to rotations for the motor
    double motorRotations = clampedDegrees * MechanicalConstants.Hood.DEGREES_TO_ROTATIONS;
    motor.setControl(positionRequest.withPosition(motorRotations));
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }
}
