// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class aligntest extends Command {

  private final Limelight robotcamera;
  private final SwerveDrive sd;
  private boolean finished;

  /** Creates a new aligntest. */
  public aligntest(Limelight robotcamera, SwerveDrive sd) {
    this.robotcamera = robotcamera;
    this.sd = sd;
    addRequirements(robotcamera);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        var latestResult = robotcamera.getLatestResult();
        var bestTarget = latestResult.getBestTarget();

        if (bestTarget != null) {
            var distance = bestTarget.getBestCameraToTarget();
            var distX = distance.getX();
            var speedX = 0.8;
            if (distX < 2.2 + 0.0254 && distX > 2.0 + 0.0254) {
              speedX = 0.4;
            }
            if (distX < 2 - 0.0254) {
              speedX *= -1;
            } 

            if (2 - 0.0254 < distX && distX < 2 + 0.0254) {
              speedX = 0;
              finished = true;
}                          
            sd.drive(-speedX, 0,0, true, false, true);
        }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sd.drive(0, 0, 0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
