package org.frc3512.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOAddressableLED implements LEDIO {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final int length;
  private String currentPattern = "OFF";

  public LEDIOAddressableLED(int pwmPort, int length) {
    this.led = new AddressableLED(pwmPort);
    this.buffer = new AddressableLEDBuffer(length);
    this.length = length;
    led.setLength(length);
    led.setData(buffer);
    led.start();
    setOff();
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.ledBuffer = new double[length * 3];
    inputs.patternLength = length;
    inputs.currentPattern = currentPattern;
    
    // Copy LED data to buffer for logging
    for (int i = 0; i < length; i++) {
      inputs.ledBuffer[i * 3] = buffer.getLED(i).red;
      inputs.ledBuffer[i * 3 + 1] = buffer.getLED(i).green;
      inputs.ledBuffer[i * 3 + 2] = buffer.getLED(i).blue;
    }
  }

  @Override
  public void setSolidColor(int r, int g, int b) {
    for (int i = 0; i < length; i++) {
      buffer.setLED(i, new Color(r / 255.0, g / 255.0, b / 255.0));
    }
    led.setData(buffer);
    currentPattern = "SOLID_" + r + "_" + g + "_" + b;
  }

  @Override
  public void setPattern(double[] patternData, int patternLength) {
    int dataLength = Math.min(patternData.length, length * 3);
    for (int i = 0; i < dataLength / 3; i++) {
      int r = (int) (patternData[i * 3] * 255);
      int g = (int) (patternData[i * 3 + 1] * 255);
      int b = (int) (patternData[i * 3 + 2] * 255);
      buffer.setLED(i, new Color(r / 255.0, g / 255.0, b / 255.0));
    }
    led.setData(buffer);
    currentPattern = "CUSTOM_PATTERN";
  }

  @Override
  public void setOff() {
    for (int i = 0; i < length; i++) {
      buffer.setLED(i, Color.kBlack);
    }
    led.setData(buffer);
    currentPattern = "OFF";
  }

  @Override
  public int getLength() {
    return length;
  }
}
