package org.frc3512.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

public class LED {
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  public LED(LEDIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);
  }

  public void setSolidColor(int r, int g, int b) {
    io.setSolidColor(r, g, b);
  }

  public void setPattern(double[] buffer, int length) {
    io.setPattern(buffer, length);
  }

  public void setOff() {
    io.setOff();
  }

  public int getLength() {
    return io.getLength();
  }

  public String getCurrentPattern() {
    return inputs.currentPattern;
  }
}
