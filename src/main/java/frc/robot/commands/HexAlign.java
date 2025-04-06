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

  boolean finished = false;
  


    /** Creates a new HexAlign. */
    
  public HexAlign(Limelight limelight, SwerveDrive swerveDrive) {

    this.limelight = limelight;
    this.swerveDrive = swerveDrive;
    addRequirements( limelight , swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limelight.CameraHasTargets()) {

      currentTag = limelight.getBestAprilTag();

      if (Arrays.stream(ApriltagConstants.CoralTags).anyMatch(a -> a == currentTag)) {

        targetHeight = ApriltagConstants.CoralHeight;

      } else if (Arrays.stream(ApriltagConstants.ProcessTags).anyMatch(a -> a == currentTag)) {

        targetHeight = ApriltagConstants.ProcessHeight;

      } else if (Arrays.stream(ApriltagConstants.ReefTags).anyMatch(a -> a == currentTag)) {

        targetHeight = ApriltagConstants.ReefHeight;
        
      } else if (Arrays.stream(ApriltagConstants.BargeTags).anyMatch(a -> a == currentTag)) {

        targetHeight = ApriltagConstants.BargeHeight;
        
      }

      targetYaw = limelight.getBestYaw();

      targetRange = PhotonUtils.calculateDistanceToTargetMeters(
        ApriltagConstants.CameraVertPos,
        targetHeight,
        Units.degreesToRadians(ApriltagConstants.CameraAngle),
        Units.degreesToRadians(limelight.getBestPitch())
      );

      rotate = (ApriltagConstants.CameraAprilAngle - targetYaw) * ApriltagConstants.CameraTurnGain * DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND;
      forward = (ApriltagConstants.CameraTagRange - targetRange) * ApriltagConstants.RobotStrafeGain * DriveConstants.MAXIMUM_LIMITED_SPEED_METRES_PER_SECOND;
      strafe = 0;
      SmartDashboard.putNumber("APL Rotate", rotate);
      SmartDashboard.putNumber("APL Forward", forward);
      SmartDashboard.putNumber("APL Strafe", strafe);

    }
    finished = true;
    swerveDrive.drive(forward, strafe, rotate, true, false, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
