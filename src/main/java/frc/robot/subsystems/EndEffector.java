// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
  private final SparkMax motor = new SparkMax(EndEffectorConstants.END_EFFECTOR_MOTOR_ID, MotorType.kBrushless);
  
  private final DigitalInput sensor = new DigitalInput(8);

  public EndEffector() {

  }
  public void run(double speed) {
    motor.set(speed);
  }

  public boolean sensordetect() {
    boolean sensorvalue = sensor.get();
    return sensorvalue;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("sensordetect", sensordetect());
  }
}
