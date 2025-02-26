// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DealgerConstants;


public class Dealger extends SubsystemBase {
  /** Creates a new Dealger. */
    private final SparkMax motor;
    private final RelativeEncoder encoder;
  

    public Dealger() {
       motor = new SparkMax(DealgerConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        motor.set(0);
        encoder = motor.getEncoder();
        encoder.setPosition(0);
    }

    public void Run(double speed) {
      motor.set(speed);
    }

    public double getPose()
    {
      return encoder.getPosition();
    }

    public void stop() {
      motor.set(0);
    }


    
  @Override
  public void periodic() {
    SmartDashboard.putNumber("de algaer pos", getPose());
    // This method will be called once per scheduler run
  }
}
