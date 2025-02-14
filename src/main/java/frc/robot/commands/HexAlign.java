// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.values;

import java.util.Arrays;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ApriltagConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HexAlign extends Command {
  private final SwerveDrive swerveDrive;
  private final Limelight limelight;

  double targetYaw;
  double targetRange;
  double targetHeight;
  double rotate;
  double forward;
  double strafe;

  int currentTag;

  String checkTag;
  
  boolean finished;
  
  
  
  /** Creates a new HexAlign. */
  
  public HexAlign(Limelight limelight, SwerveDrive swerveDrive) {
    this.finished = false;
    this.limelight = limelight;
    this.swerveDrive = swerveDrive;
    addRequirements( limelight , swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    SmartDashboard.putBoolean("HexRun", true);
    if (limelight.CameraHasTargets()) {

      currentTag = limelight.getAprilTag();
      checkTag = String.valueOf(currentTag);

      if (Arrays.stream(ApriltagConstants.CoralTags).anyMatch(checkTag::equals)) {

        targetHeight = ApriltagConstants.CoralHeight;

      } else if (Arrays.stream(ApriltagConstants.ProcessTags).anyMatch(checkTag::equals)) {

        targetHeight = ApriltagConstants.ProcessHeight;

      } else if (Arrays.stream(ApriltagConstants.ReefTags).anyMatch(checkTag::equals)) {

        targetHeight = ApriltagConstants.ReefHeight;
        
      } else if (Arrays.stream(ApriltagConstants.BargeTags).anyMatch(checkTag::equals)) {

        targetHeight = ApriltagConstants.BargeHeight;
        
      }

      targetYaw = limelight.TargetYaw();

      targetRange = (PhotonUtils.calculateDistanceToTargetMeters(
        ApriltagConstants.CameraVertPos,
        targetHeight,
        Units.degreesToRadians(ApriltagConstants.CameraAngle),
        Math.abs(Units.degreesToRadians(limelight.TargetPitch()))
      ));
      SmartDashboard.putNumber("TargetRange", targetRange);

      rotate = (ApriltagConstants.CameraAprilAngle - targetYaw) * DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND;
      forward = (ApriltagConstants.CameraTagRange - targetRange) * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND;
      strafe = 0;
      SmartDashboard.putNumber("APL Rotate", rotate);
      SmartDashboard.putNumber("APL Forward", forward);
      SmartDashboard.putNumber("APL Strafe", strafe);


      swerveDrive.drive(0,  0 , 0, true, false, true);
      
          swerveDrive.drive(0,  1 , 0, true, false, true);

    } else {
      SmartDashboard.putBoolean("HexRun", false);
      finished = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetRange < 1) {
      finished = true;
    } else {
      targetYaw = limelight.TargetYaw();

      rotate = (Math.abs(targetYaw)+8)/ApriltagConstants.RotateControl;
      strafe = (Math.abs(targetYaw)-5)/ApriltagConstants.StrafeControl;
      if (strafe < 0) {
        strafe = 0;
      }

      if (strafe > 1) {
        strafe = 1;
      }
      
      if (rotate > 1) {
        rotate = 0.1;
      }

      if (targetYaw > 0) {
        rotate = -rotate;
        strafe = -strafe;
      }

      forward = 1;

      forward = forward * ApriltagConstants.SpeedControl;
      rotate = rotate * ApriltagConstants.SpeedControl;
      strafe = strafe * ApriltagConstants.SpeedControl;


      swerveDrive.drive(forward, rotate, strafe, false, false, true);
      
      
      targetRange = (PhotonUtils.calculateDistanceToTargetMeters(
        ApriltagConstants.CameraVertPos,
        targetHeight,
        Units.degreesToRadians(ApriltagConstants.CameraAngle),
        Math.abs(Units.degreesToRadians(limelight.TargetPitch()))
        )); 
          SmartDashboard.putNumber("TargetRange", targetRange);
        }

    }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("HexRun", false);
    swerveDrive.drive(0, 0, 0, true, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
