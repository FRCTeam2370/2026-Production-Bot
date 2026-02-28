// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIntakePosAndSpeed extends Command {
  /** Creates a new SetIntakePosAndSpeed. */
  double pos, initialTargetSpeed, velocityOffset;
  SwerveSubsystem mSwerve;
  public SetIntakePosAndSpeed(double pos, double initialTargetSpeed, IntakeSubsystem mIntakeSubsystem, SwerveSubsystem mSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pos = pos;
    this.initialTargetSpeed = initialTargetSpeed;
    this.mSwerve = mSwerve;
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocityOffset = Math.abs(mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond / SwerveConstants.maxSpeed) * (100 - initialTargetSpeed);
    IntakeSubsystem.setIntakePos(pos);
    IntakeSubsystem.intakeWithVelocity(initialTargetSpeed + velocityOffset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.setIntakePos(Rotation2d.fromDegrees(90).getRotations());
    IntakeSubsystem.intakeWithoutVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
