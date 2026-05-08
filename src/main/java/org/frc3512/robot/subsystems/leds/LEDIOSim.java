package org.frc3512.robot.subsystems.leds;

public class LEDIOSim implements LEDIO {
  private final int length;
  private double[] ledBuffer;
  private String currentPattern = "OFF";

  public LEDIOSim(int length) {
    this.length = length;
    this.ledBuffer = new double[length * 3];
    setOff();
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.ledBuffer = ledBuffer.clone();
    inputs.patternLength = length;
    inputs.currentPattern = currentPattern;
  }

  @Override
  public void setSolidColor(int r, int g, int b) {
    for (int i = 0; i < length; i++) {
      ledBuffer[i * 3] = r / 255.0;
      ledBuffer[i * 3 + 1] = g / 255.0;
      ledBuffer[i * 3 + 2] = b / 255.0;
    }
    currentPattern = "SOLID_" + r + "_" + g + "_" + b;
  }

  @Override
  public void setPattern(double[] patternData, int patternLength) {
    int dataLength = Math.min(patternData.length, length * 3);
    System.arraycopy(patternData, 0, ledBuffer, 0, dataLength);
    currentPattern = "CUSTOM_PATTERN";
  }

  @Override
  public void setOff() {
    for (int i = 0; i < length * 3; i++) {
      ledBuffer[i] = 0.0;
    }
    currentPattern = "OFF";
  }

  @Override
  public int getLength() {
    return length;
  }
}
