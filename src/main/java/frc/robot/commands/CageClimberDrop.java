// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CageClimberConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.subsystems.CageClimber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CageClimberDrop extends Command {
  private final CageClimber cageClimber;
  /** Creates a new CageClimberClimb. */
  public CageClimberDrop(CageClimber cageClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cageClimber = cageClimber;
    addRequirements(cageClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cageClimber.run(CageClimberConstants.CAGE_CLIMBER_MOTOR_SPEED_INVERTED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cageClimber.run(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //CageClimber.sensor1detectinverted();
  }
}
