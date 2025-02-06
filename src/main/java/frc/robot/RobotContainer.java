// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.commands.Autos;

import frc.robot.commands.ElevatorTestOFF;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HexAlign;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.EndEffectorDrop;
import frc.robot.commands.EndEffectorGrab;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HexAlign;
import frc.robot.commands.TriggerTest;
import frc.robot.subsystems.EndEffector;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

import frc.lib.Controller;
import frc.lib.FlightControl;

import java.util.PrimitiveIterator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Elevator elevator = new Elevator();
  private final SwerveDrive sd = new SwerveDrive();
  private final Controller controller = new Controller(1);
  private final EndEffector endEffector = new EndEffector();
  private final EndEffectorGrab endEffectorGrab = new EndEffectorGrab(endEffector);
  private final EndEffectorDrop endEffectorDrop = new EndEffectorDrop(endEffector);
  private final Limelight cam =  new Limelight();
  private final FlightControl flightcont = new FlightControl(0);

  private final TriggerTest tt = new TriggerTest();

  private final HexAlign hexalign = new HexAlign(cam, sd);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureSubsystemCommands();
    //configureSwerveDriveCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // cam.setDefaultCommand(
    //   new RunCommand(() -> cam.updatesd(), cam)
    // );
    
    // sd.setDefaultCommand(
    //   new RunCommand(() -> sd.drive(
    //     -MathUtil.applyDeadband(flightcont.getJoyY(), OperatorConstants.DEADBAND), 
    //     -MathUtil.applyDeadband(flightcont.getJoyX(), OperatorConstants.DEADBAND), 
    //     -MathUtil.applyDeadband(flightcont.getTwist(), OperatorConstants.DEADBAND), 
    //     true, 
    //     true, 
    //     true)
    //   ,sd)
    // );

    controller.getA().onTrue(
            new InstantCommand(
                () -> elevator.setTarget(ElevatorConstants.LEVEL_1_HEIGHT)
                )
            );

    controller.getB().onTrue(
            new InstantCommand(
                () -> elevator.setTarget(ElevatorConstants.LEVEL_2_HEIGHT)
                )
            );
    controller.getX().onTrue(
            new InstantCommand(
                () -> elevator.setTarget(ElevatorConstants.LEVEL_3_HEIGHT)
                )
            );


    cam.setDefaultCommand(
      new RunCommand(() -> cam.updatesd(), cam)
    );
    
    sd.setDefaultCommand(
      new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(flightcont.getJoyY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getJoyX(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getTwist(), OperatorConstants.DEADBAND), 
        true, 
        true, 
        true)
      ,sd)
    );


    //controller.getX().whileTrue(endEffectorGrab);
    //controller.getY().whileTrue(endEffectorDrop);

  }

  private void configureSubsystemCommands() {
    flightcont.getButton2().whileTrue(tt);

  }

  private void configureSwerveDriveCommands() {
    flightcont.getButton2()
        .whileTrue(
            new RunCommand(
                () -> sd.zeroHeading(),
                sd
            )
        );

    flightcont.getButton3().whileTrue(new RunCommand(() -> sd.lockPosition(), sd));

}

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
