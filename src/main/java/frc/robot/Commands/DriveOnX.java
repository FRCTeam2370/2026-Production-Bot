// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.FieldInfo;
import frc.robot.Subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOnX extends SequentialCommandGroup {
  SwerveSubsystem mSwerve;
  /** Creates a new DriveOnX. */
  public DriveOnX(SwerveSubsystem mSwerve, DoubleSupplier ySupplier) {
    this.mSwerve = mSwerve;
    Pose2d pose;
    if(SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > 4){
      pose = FieldInfo.fieldPoints.RightHubSweep;
    }else{
      pose = FieldInfo.fieldPoints.LeftHubSweep;
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(mSwerve.PathfindToPose(()-> pose), 
      new TeleopSwerve(mSwerve, 
        ySupplier,
        ()-> SwerveSubsystem.poseEstimator.getEstimatedPosition().getX() - pose.getX(), 
        ()-> 2 * (SwerveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getRotations() - (ySupplier.getAsDouble() > 0 ? FieldInfo.fieldPoints.RightHubSweep.getRotation().getRotations() : FieldInfo.fieldPoints.LeftHubSweep.getRotation().getRotations())),
        ()-> false, 
        false));
  }
}
