// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

public class Elevator extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    private double target; // Meters
    private DigitalInput sensor;
    private boolean manual;
    private SparkMaxConfig config;

    /** Creates a new Elevator. */
    public Elevator() {

        manual = false;
        motor = new SparkMax(ElevatorConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        motor.set(0);

        config = new SparkMaxConfig();

        config.encoder.positionConversionFactor(ElevatorConstants.GEARBOX_RATIO);
        config.idleMode(IdleMode.kBrake);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();

        sensor = new DigitalInput(ElevatorConstants.SENSOR_CHANNEL);

        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        updateDashboard();

        if(manual == false)
        {
            goToTarget();
        }
        
    }

    public void setManualMode(boolean isManual)
    {
        manual = true;
    }

    public void goToTarget()
    {
        if (atBottom()) {
            //encoder.setPosition(0);
        }

        if (atTarget()) {
            motor.set(0);
            
            if(target != 0)
            {
                motor.setVoltage(ElevatorConstants.STALL_VOLTAGE);
            }
            else
            {
                motor.setVoltage(0);
            }
        }

        else if ((target - encoder.getPosition()) > 0.1) {
            motor.set(ElevatorConstants.AUTO_SPEED_UP);
        }

        else if ((target - encoder.getPosition()) < 0.1) {
            motor.set(ElevatorConstants.AUTO_SPEED_DOWN);
        }
    }

    public void spinUp() {
        motor.set(ElevatorConstants.MANUAL_SPEED_UP);
    }

    public void spinDown() {
        motor.set(ElevatorConstants.MANUAL_SPEED_DOWN);
    }

    public void stop() {
        motor.set(0);
        motor.stopMotor();
    }

    public void setTarget(double meters) {
        manual = false;
        target = meters;
    }

    public boolean atTarget() {
        return Math.abs(encoder.getPosition() - target) < 0.1;
    }

    public boolean atBottom() {
        return !sensor.get(); // Invert with not to get true when at sensor is tripped
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Elevator Test RPM", encoder.getVelocity());
        // SmartDashboard.putData("Elevator", elevatorMech);
        SmartDashboard.putNumber("Elevator Rotations", encoder.getPosition());
        SmartDashboard.putNumber("Target height", target);
    }
}
