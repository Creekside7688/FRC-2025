// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RgbLEDs extends SubsystemBase {
  AddressableLED led = new AddressableLED(9);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
    final LEDPattern rainbow = LEDPattern.rainbow(255, 128);  
    final Distance kLedSpacing = Meters.of(1 / 120.0);
    final LEDPattern scrollingRainbow =
    rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
  /** Creates a new RGBLEDs. 
   * @return */
    public void RgbSolidRed() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    LEDPattern red = LEDPattern.solid(Color.kRed);
        red.applyTo(ledBuffer);
    led.setData(ledBuffer);
    }
  public void RgbRainbowScroll(){

  }
  @Override
  public void periodic() { 
                // This method will be called once per scheduler run
                scrollingRainbow.applyTo(ledBuffer);
                led.setData(ledBuffer);
  }
}
