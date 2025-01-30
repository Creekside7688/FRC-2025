// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CageClimberConstants;
import frc.robot.constants.EndEffectorConstants;

public class CageClimber extends SubsystemBase {
  private final RelativeEncoder rotationsensor = motor.getEncoder();
  private final SparkMax motor = new SparkMax(CageClimberConstants.CAGE_CLIMBER_MOTOR_ID, MotorType.kBrushless);
  /** Creates a new Climber. */
  public CageClimber() {

  }
 public void run(double speed) {
    motor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
