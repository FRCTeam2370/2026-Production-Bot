// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIntakePosAndSpeed extends Command {
  /** Creates a new SetIntakePosAndSpeed. */
  double pos, initialTargetSpeed, velocityOffset;
  SwerveSubsystem mSwerve;
  Timer jiggleTimer = new Timer();
  public SetIntakePosAndSpeed(double pos, double initialTargetSpeed, IntakeSubsystem mIntakeSubsystem, SwerveSubsystem mSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pos = pos;
    this.initialTargetSpeed = initialTargetSpeed;
    this.mSwerve = mSwerve;
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    jiggleTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocityOffset = initialTargetSpeed == 0 ? 0 : mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond / SwerveConstants.maxSpeed * (100 - initialTargetSpeed);

    if(TurretSubsystem.isShooting && mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond == 0 && mSwerve.getRobotRelativeSpeeds().vyMetersPerSecond == 0 && jiggleTimer.get() % 1 < 0.5){
      IntakeSubsystem.setIntakePos(pos + Rotation2d.fromDegrees(70).getRotations());
    }else{
      IntakeSubsystem.setIntakePos(pos);
    }
    SmartDashboard.putNumber("jiggle timer", jiggleTimer.get() % 3);

    if(RobotContainer.driver.leftTrigger().getAsBoolean()){
      IntakeSubsystem.intakeWithVelocity(-initialTargetSpeed);
    }else{
      IntakeSubsystem.intakeWithVelocity(initialTargetSpeed + velocityOffset);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.setIntakePos(Rotation2d.fromDegrees(90).getRotations());
    IntakeSubsystem.intakeWithoutVelocity(0);
    jiggleTimer.stop();
    jiggleTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
