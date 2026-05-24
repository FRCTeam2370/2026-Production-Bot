// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.FieldInfo.Dot;

public class AutoHandler extends SubsystemBase {
  SendableChooser<Command> autoChooser;
  SendableChooser<Dot> dotChooser;
  SwerveSubsystem mSwerve;
  private Command auto = null;
  Command lastAuto = null;
  Dot lastDot = null;
  /** Creates a new AutoHandler. */
  public AutoHandler(SendableChooser<Command> autoChooser, SendableChooser<Dot> dotChooser, SwerveSubsystem mSwerve) {
    this.autoChooser = autoChooser;
    this.mSwerve = mSwerve;
    this.dotChooser = dotChooser;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.isDisabled()){
      makeAuto();
    }
  }

  private void makeAuto(){
    if(autoChooser.getSelected() != lastAuto && dotChooser.getSelected() != lastDot){
      auto = Commands.race(new WaitCommand(15), autoChooser.getSelected()).andThen(mSwerve.PathfindToPose(()-> dotChooser.getSelected() == Dot.LEFT_DOT ? FieldConstants.dot1Pose : dotChooser.getSelected() == Dot.MIDDLE_DOT ? FieldConstants.dot2Pose : FieldConstants.dot3Pose));
      lastAuto = autoChooser.getSelected();
      lastDot = dotChooser.getSelected();
    }
  }

  public Command getAutonomousCommand(){
    return auto;
  }

}
