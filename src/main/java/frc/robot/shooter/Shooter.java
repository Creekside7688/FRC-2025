package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(ShooterConstants.MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder;

    public Shooter() {
        motor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        motor.setIdleMode(ShooterConstants.IDLE_MODE);
        motor.burnFlash();

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(getName(), encoder.getVelocity());
    }

    public void run(double speed) {
        motor.set(speed);
    }
}
