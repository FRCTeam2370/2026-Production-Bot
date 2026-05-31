// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utils.BallLogic;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FindADotHop extends SequentialCommandGroup {
  /** Creates a new FindADot. */
  public FindADotHop(SwerveSubsystem mSwerve, Supplier<Pose2d> dots, BooleanSupplier searchMore) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(mSwerve.PathfindToPose(dots).andThen(new PIDToPoint(dots, mSwerve, searchMore)));
  }
}
