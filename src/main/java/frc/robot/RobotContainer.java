// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CageClimberClimb;
import frc.robot.commands.CageClimberDrop;
import frc.robot.commands.ElevatorTestOFF;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HexAlign;
import frc.robot.commands.ElevatorTestDOWN;
import frc.robot.commands.ElevatorTestUP;
import frc.robot.commands.ElevatorTestOFF;
import frc.robot.subsystems.CageClimber;
import frc.robot.subsystems.ElevatorTestSubsystem;


import frc.robot.commands.EndEffectorDrop;
import frc.robot.commands.EndEffectorGrab;
import frc.robot.commands.EndEffectorReverse;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HexAlign;
import frc.robot.commands.TriggerTest;

import frc.robot.subsystems.EndEffector;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.climber;
import frc.lib.Controller;
import frc.lib.FlightControl;

import java.util.PrimitiveIterator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final ElevatorTestSubsystem m_ElevatorTestSubsystem = new ElevatorTestSubsystem();
  private final SwerveDrive sd = new SwerveDrive();

  private final Controller controller = new Controller(1);
  private final Controller teoController = new Controller(2);
  private final EndEffector endEffector = new EndEffector();
  private final EndEffectorGrab endEffectorGrab = new EndEffectorGrab(endEffector);
  private final EndEffectorDrop endEffectorDrop = new EndEffectorDrop(endEffector);


  private final Limelight cam =  new Limelight();
  private final FlightControl flightcont = new FlightControl(0);
  /*private final climber clmber = new climber();
  private final lower lwer = new lower(clmber);
  private final cimb clmb = new cimb(clmber);*/

  


  private final TriggerTest tt = new TriggerTest();

  private final HexAlign hexalign = new HexAlign(cam, sd);

  private final ElevatorTestOFF elevatorTestOFF = new ElevatorTestOFF(m_ElevatorTestSubsystem);
  private final ElevatorTestUP elevatorTestUP = new ElevatorTestUP(m_ElevatorTestSubsystem);
  private final ElevatorTestDOWN elevatorTestDOWN = new ElevatorTestDOWN(m_ElevatorTestSubsystem);

  private final CageClimber cageClimber = new CageClimber();
  private final CageClimberClimb climb = new CageClimberClimb(cageClimber);
  private final CageClimberDrop drop = new CageClimberDrop(cageClimber);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    CameraServer.startAutomaticCapture();
    configureControllerBindings();
    //configureJoystickBindings();
    configureOperatorBindings();
    //configureSubsystemCommands();
    configureSwerveDriveCommands();
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
  private void configureControllerBindings() {


    //controller.getX().whileTrue(elevatorTestUP);
    //controller.getY().whileTrue(elevatorTestDOWN);
    //controller.getB().whileTrue(elevatorTestOFF);

  

    cam.setDefaultCommand(
      new RunCommand(() -> cam.updatesd(), cam)
    );

    
    sd.setDefaultCommand(
      new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(teoController.getLeftX() * -1, OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getLeftY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getRightX(), OperatorConstants.DEADBAND), 
        false, 
        true, 
        true)
      ,sd)
    );




    //controller.getLeftBumper().whileTrue(endEffectorGrab);
    //controller.getRightBumper().whileTrue(endEffectorDrop);
    //controller.getLeftBumper().whileTrue(climb);
    //controller.getRightBumper().whileTrue(drop);


    teoController.getRightTrigger().whileTrue(new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(teoController.getLeftX() * -1, OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getLeftY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getRightX(), OperatorConstants.DEADBAND), 
        true, 
        true, 
        true)
      ,sd));


    
  }

  public void configureJoystickBindings(){
     
    sd.setDefaultCommand(
      new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(flightcont.getJoyX() * -1, OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getJoyY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getTwist(), OperatorConstants.DEADBAND), 
        false, 
        true, 
        true)
      ,sd)
    );



    flightcont.getButton1().whileTrue(new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(flightcont.getJoyX() * -1, OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getJoyY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getTwist(), OperatorConstants.DEADBAND), 
        true, 
        true, 
        true)
      ,sd));
  }


  public void configureOperatorBindings(){
    //climber commands
    controller.getLeftBumper().whileTrue(climb);
    controller.getRightBumper().whileTrue(drop);
    
    //elevator commands
    controller.getY().whileTrue(elevatorTestUP);
    controller.getX().whileTrue(elevatorTestDOWN);
    controller.getB().whileTrue(elevatorTestOFF);

    //end effector commands
    controller.getLeftTrigger().whileTrue(endEffectorGrab);
    controller.getRightTrigger().whileTrue(endEffectorDrop);
  }



  private void configureSwerveDriveCommands() {
    controller.getRightStick()
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
