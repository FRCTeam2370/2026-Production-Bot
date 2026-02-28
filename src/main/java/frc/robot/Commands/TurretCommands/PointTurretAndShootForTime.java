// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TurretCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Shooter.ShootAtVelocity;
import frc.robot.Commands.Shooter.ShootAtVelocityForTime;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.UptakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PointTurretAndShootForTime extends ParallelCommandGroup {
  /** Creates a new PointTurretAndShoot. */
  public PointTurretAndShootForTime(Translation3d pose, double seconds, TurretSubsystem mTurretSubsystem, SwerveSubsystem mSwerve, UptakeSubsystem mUptakeSubsystem, SpindexerSubsystem mSpindexerSubsystem, ShooterSubsystem mShooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PointTurretAtPointForTime(pose, seconds, mTurretSubsystem, mSwerve), new ShootAtVelocityForTime(mShooterSubsystem, mUptakeSubsystem, mSpindexerSubsystem, mSwerve, seconds));
  }
}
