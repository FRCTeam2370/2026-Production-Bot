// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Blue;
import frc.robot.Constants.FieldConstants.Red;
import frc.robot.Subsystems.LEDSubsystem.LEDState;

public class FieldInfo extends SubsystemBase {
  boolean prefireRed = false, prefireBlue = false, endgame = false;

  public static class FieldPoints {
    public Translation3d HubPose;
    public Translation3d PassPose1;
    public Translation3d PassPose2;
    public FieldPoints(Translation3d HubPose, Translation3d PassPose1, Translation3d PassPose2){
      this.HubPose = HubPose;
      this.PassPose1 = PassPose1;
      this.PassPose2 = PassPose2;
    }
  }

  public static FieldPoints fieldPoints = new FieldPoints(null, null, null);

  /** Creates a new FieldInfo. */
  public FieldInfo() {
    if(SwerveSubsystem.color.get() == Alliance.Blue){
      fieldPoints.HubPose = Blue.HubFieldPoseBlue;
      fieldPoints.PassPose1 = Blue.PassPose1;
      fieldPoints.PassPose2 = Blue.PassPose2;
    }else{
      fieldPoints.HubPose = Red.HubFieldPoseRed;
      fieldPoints.PassPose1 = Red.PassPose1;
      fieldPoints.PassPose2 = Red.PassPose2;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Hub Active", isHubActive());
    // if(isHubActive()){
    //     if(SwerveSubsystem.color.get() == Alliance.Blue){
    //       LEDSubsystem.mLEDState = LEDState.Blue;
    //     }else{
    //       LEDSubsystem.mLEDState = LEDState.Red;
    //     }
    // }else{
    //   if(SwerveSubsystem.color.get() == Alliance.Blue){
    //     LEDSubsystem.mLEDState = LEDState.Red;
    //   }else{
    //     LEDSubsystem.mLEDState = LEDState.Blue;
    //   }
    // }
    // if(prefireBlue){
    //   LEDSubsystem.mLEDState = LEDState.PrepareBlue;
    // }else if(prefireRed){
    //   LEDSubsystem.mLEDState = LEDState.PrepareRed;
    // }else if (endgame){
    //   if(SwerveSubsystem.color.get() == Alliance.Blue){
    //     LEDSubsystem.mLEDState = LEDState.EndgameBlue;
    //   }else{
    //     LEDSubsystem.mLEDState = LEDState.EndgameRed;
    //   }
    // }

    if(DriverStation.getMatchTime() < 31 && DriverStation.getMatchTime() > 29){
      if(SwerveSubsystem.color.get() == Alliance.Blue){
        LEDSubsystem.mLEDState = LEDState.EndgameBlue;
      }else{
        LEDSubsystem.mLEDState = LEDState.EndgameRed;
      }
    }
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public boolean isHubActive() {
  Optional<Alliance> alliance = SwerveSubsystem.color;
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    return false;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return true;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return false;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return true;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      redInactiveFirst = true;
    }
  }

  // Shift was is active for blue if red won auto, or red if blue won auto.
  boolean shift1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;
    case Blue -> redInactiveFirst;
  };

  if(matchTime < 115 && matchTime > 105){
    selectPrefire(alliance.get());
  }else if(matchTime < 90 && matchTime > 80){
    selectPrefire(alliance.get());
  }else if(matchTime < 65 && matchTime > 55){
    selectPrefire(alliance.get());
  }else if(matchTime < 40 && matchTime > 30){
    selectPrefire(alliance.get());
  }else{
    prefireBlue = false;
    prefireRed = false;
  }

  if (matchTime > 130) {
    // Transition shift, hub is active.
    return true;
  } else if (matchTime > 105) {
    // Shift 1
    return shift1Active;
  } else if (matchTime > 80) {
    // Shift 2
    return !shift1Active;
  } else if (matchTime > 55) {
    // Shift 3
    return shift1Active;
  } else if (matchTime > 30) {
    // Shift 4
    return !shift1Active;
  } else {
    // End game, hub always active.
    endgame = true;
    return true;
  }
}

private void selectPrefire(Alliance alliance){
  if(alliance == Alliance.Blue){
      prefireRed= false;
      prefireBlue = true;
    }else{
      prefireBlue = false;
      prefireRed = true;
    }
}
}
