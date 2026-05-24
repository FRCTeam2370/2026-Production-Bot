// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MakeAuto extends Command {
  Timer timer = new Timer();
  SwerveSubsystem mSwerveSubsystem;
  Command autoPath;
  Pose2d dot;
  /** Creates a new MakeAuto. */
  public MakeAuto(Command autoPath, Pose2d dot, SwerveSubsystem mSwerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.autoPath = autoPath;
    this.mSwerveSubsystem = mSwerveSubsystem;
    this.dot = dot;
    addRequirements(mSwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    SmartDashboard.putBoolean("Done", false);
    CommandScheduler.getInstance().schedule(autoPath);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.advanceIfElapsed(15)){
      SmartDashboard.putBoolean("Done", true);
      CommandScheduler.getInstance().cancel(autoPath);
      CommandScheduler.getInstance().schedule(AutoBuilder.pathfindToPose(dot, Constants.SwerveConstants.telePathConstraints));
      return true;
    }else{
      return false;
    }
  }
}
