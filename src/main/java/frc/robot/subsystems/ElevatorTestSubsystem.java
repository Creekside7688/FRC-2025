// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorTestConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;


public class ElevatorTestSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    private Mechanism2d elevatorMech = new Mechanism2d(3, 3);
    private MechanismRoot2d elevatorRoot = elevatorMech.getRoot("superstructure", 1.5, 0.5);
    private MechanismLigament2d elevator = elevatorRoot.append(
            new MechanismLigament2d("elevator", 0.5, 90));

    private double target; // Meters
    private DigitalInput sensor;

    private SparkMaxConfig config;

    /** Creates a new ElevatorTestSubsystem. */
    public ElevatorTestSubsystem() {
        motor = new SparkMax(ElevatorTestConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        motor.set(0);

        config = new SparkMaxConfig();
        
        config.encoder.positionConversionFactor(1);
        encoder = motor.getAlternateEncoder();

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sensor = new DigitalInput(ElevatorTestConstants.SENSOR_CHANNEL);
    }

    @Override
    public void periodic() {
        updateDashboard();

        if (atBottom()) {
            encoder.setPosition(0);
        }

        if (atTarget()) {
            motor.set(0);
        }

        else if ((getTarget() - getPosition()) > 0.1) {
            motor.set(ElevatorTestConstants.SPEED);
        }

        else if ((getTarget() - getPosition()) < 0.1) {
            motor.set(-ElevatorTestConstants.SPEED);
        }
    }

    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return encoder.getPosition() * ElevatorTestConstants.HEIGHT_PER_ROTATION;
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

    public void setTarget(double meters) {
        target = meters;
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - target) < 0.1;
    }

    public boolean atBottom() {
        return !sensor.get(); // Invert with not to get true when at sensor is tripped 
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Elevator Test RPM", encoder.getVelocity());
        SmartDashboard.putData("Elevator", elevatorMech);
        SmartDashboard.putNumber("Elevator Rotations", encoder.getPosition());
        SmartDashboard.putNumber("Calculated Elevator Position", this.getPosition());
    }
}
