
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  /** Creates a new climber. */
  private final TalonSRX leftMotor = new TalonSRX(1);
  private final TalonSRX rightMotor = new TalonSRX(2);
  public climber() {
    leftMotor.follow(rightMotor);
  }


  public void open()
  {
    rightMotor.set(ControlMode.PercentOutput, 0.4);
  }

  public void close()
  {
    rightMotor.set(ControlMode.PercentOutput, -0.4);
  }

  public void stop()
  {
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
