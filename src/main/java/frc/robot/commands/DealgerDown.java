// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DealgerConstants;
import frc.robot.subsystems.Dealger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */


public class DealgerDown extends Command {
  boolean DealgerUp = true;
  private final Dealger dealger;
  /** Creates a new DealgerDown. */
  public DealgerDown(Dealger dealger) {
    this.dealger = dealger;
    addRequirements(dealger);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dealger.Run(0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dealger.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dealger.getPosition() > DealgerConstants.DEPLOY_ROTATIONS;
  }
}
