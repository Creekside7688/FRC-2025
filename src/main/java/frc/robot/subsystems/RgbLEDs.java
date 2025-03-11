// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class RgbLEDs extends SubsystemBase {
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    final LEDPattern rainbow = LEDPattern.rainbow(255, 128);  
    final Distance kLedSpacing = Meters.of(1 / 30.0);
    final LEDPattern scrollingRainbow =
    rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
  /** Creates a new RGBLEDs. 
   * @return */
  public RgbLEDs() {
    led = new AddressableLED(8);
    ledBuffer =  new AddressableLEDBuffer(90);

    led.setLength(ledBuffer.getLength());
    
    led.setData(ledBuffer);
    led.start();
    

    //RgbSolidRed();
    setDefaultCommand(runPattern(scrollingRainbow).withName("Off"));
  } 
   public void RgbSolidRed() {
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuffer);
    led.setData(ledBuffer);
    }

  @Override
  public void periodic() { 
                // This method will be called once per scheduler run
                led.setData(ledBuffer);
  }

public Command runPattern(LEDPattern pattern) {
 return run(() -> pattern.applyTo(ledBuffer));
  }
  }