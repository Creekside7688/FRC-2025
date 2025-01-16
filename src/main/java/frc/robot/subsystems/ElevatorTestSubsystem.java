// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorTestCommand;
import frc.robot.constants.ElevatorTestConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

public class ElevatorTestSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  /** Creates a new ElevatorTestSubsystem. */
  public ElevatorTestSubsystem() {
    motor = new SparkMax(ElevatorTestConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
    motor.set(0);

    encoder = motor.getAlternateEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void spinUp() {
    motor.set(ElevatorTestConstants.SPEED);
  }

  public void spinDown() {
    motor.set(-ElevatorTestConstants.SPEED);
  }

  public void stop() {
    motor.set(0);
    motor.stopMotor();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Elevator Test RPM", encoder.getVelocity());
  }
}
