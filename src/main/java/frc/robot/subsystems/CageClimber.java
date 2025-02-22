// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CageClimberConstants;
import frc.robot.constants.EndEffectorConstants;

public class CageClimber extends SubsystemBase {
  private final TalonSRX LeftMotor = new TalonSRX(1);
  private final TalonSRX RightMotor = new TalonSRX(2);

    private final  DigitalInput sensor1 = new DigitalInput(0);
     private final DigitalInput sensor2 = new DigitalInput(1);
    /** Creates a new Climber. */
    public CageClimber() {
      LeftMotor.follow(RightMotor);
      LeftMotor.setNeutralMode(NeutralMode.Brake);
      RightMotor.setNeutralMode(NeutralMode.Brake);
    
    }
   public void run(double speed) {
      RightMotor.set(ControlMode.PercentOutput, speed);
    }
  public boolean sensor1detect() {
    boolean sensorvalue = !sensor1.get();
    return sensorvalue;
  }
  public boolean sensor1detectinverted() {
    boolean sensorvalue = !sensor1.get();
    return sensorvalue;
  }
  public boolean sensor2detect() {
    boolean sensorvalue = !sensor2.get();
    return sensorvalue;
  }
  public boolean sensor2detectinverted() {
    boolean sensorvalue = !sensor2.get();
    return sensorvalue;
  }
  @Override
  public void periodic() {
   SmartDashboard.putBoolean("sensordetect1", sensor1detect());
   SmartDashboard.putBoolean("sensordetect2", sensor2detect());
    // This method will be called once per scheduler run
  }
}
