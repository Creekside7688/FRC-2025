// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorTestUP;
import frc.robot.commands.ElevatorTestOFF;
import frc.robot.commands.ElevatorTestDOWN;
import frc.robot.constants.ElevatorTestConstants;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

public class ElevatorTestSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  private Mechanism2d elevatorMech = new Mechanism2d(3, 3);
  private MechanismRoot2d elevatorRoot = 
      elevatorMech.getRoot("superstructure", 1.5, 0.5);
  private MechanismLigament2d elevator =
      elevatorRoot.append(
              new MechanismLigament2d("elevator", 0.5, 90)
              );

  /** Creates a new ElevatorTestSubsystem. */
  public ElevatorTestSubsystem() {
    motor = new SparkMax(ElevatorTestConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
    motor.set(0);

    encoder = motor.getAlternateEncoder();
  }

  @Override
  public void periodic() {
      updateDashboard();
  }

  public void spinUp() {
    motor.set(ElevatorTestConstants.SPEED);
  }

  public void spinDown() {
    motor.set(-0.2);
  }

  public void stop() {
    motor.set(0);
    motor.stopMotor();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Elevator Test RPM", encoder.getVelocity());
    SmartDashboard.putData("Elevator", elevatorMech);
  }
}
