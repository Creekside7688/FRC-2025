// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoBack extends Command {
  /** Creates a new AutoBack. */
  private final SwerveDrive sw;
  private double startTime;
  public AutoBack(SwerveDrive input) {
    sw = input;
    addRequirements(input);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    SmartDashboard.putBoolean("end auto", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sw.drive(0,-0.8,0,false,true,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("end auto", true);
    //sw.drive(0, 0, 0, true, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {    
    return sw.getPose().getY() > 0.2;
  }
}
