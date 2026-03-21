// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Blue;
import frc.robot.Constants.FieldConstants.Red;
import frc.robot.Subsystems.LEDSubsystem.LEDState;

public class FieldInfo extends SubsystemBase {
  boolean prefireRed = false, prefireBlue = false, endgame = false;
  public static Timer turretTimer = new Timer();

  public static class FieldPoints {
    public Translation3d HubPose, PassPose1, PassPose2, EvilPassRight, EvilPassLeft;
    public Pose2d ClimbLeft, ClimbRight, LeftHubSweep, RightHubSweep;
    public FieldPoints(Translation3d HubPose, Translation3d PassPose1, Translation3d PassPose2, Pose2d ClimbLeft, Pose2d ClimbRight, Pose2d LeftHubSweep, Pose2d RightHubSweep){
      this.HubPose = HubPose;
      this.PassPose1 = PassPose1;
      this.PassPose2 = PassPose2;
      this.ClimbLeft = ClimbLeft;
      this.ClimbRight = ClimbRight;
      this.LeftHubSweep = LeftHubSweep;
      this.RightHubSweep = RightHubSweep;
    }
  }

  public static FieldPoints fieldPoints = new FieldPoints(null, null, null, null, null, null, null);

  /** Creates a new FieldInfo. */
  public FieldInfo() {
    turretTimer.reset();
    turretTimer.start();
    if(SwerveSubsystem.color.get() == Alliance.Blue){
      fieldPoints.HubPose = Blue.HubFieldPoseBlue;
      fieldPoints.PassPose1 = Blue.PassPose1;
      fieldPoints.PassPose2 = Blue.PassPose2;
      fieldPoints.ClimbLeft = Blue.ClimbLeft;
      fieldPoints.ClimbRight = Blue.ClimbRight;
      fieldPoints.LeftHubSweep = Blue.StartHubSweepLeft;
      fieldPoints.RightHubSweep = Blue.StartHubSweepRight;
      fieldPoints.EvilPassLeft = Blue.EvilPassLeft;
      fieldPoints.EvilPassRight = Blue.EvilPassRight;
    }else{
      fieldPoints.HubPose = Red.HubFieldPoseRed;
      fieldPoints.PassPose1 = Red.PassPose1;
      fieldPoints.PassPose2 = Red.PassPose2;
      fieldPoints.ClimbLeft = Red.ClimbLeft;
      fieldPoints.ClimbRight = Red.ClimbRight;
      fieldPoints.LeftHubSweep = Red.StartHubSweepLeft;
      fieldPoints.RightHubSweep = Red.StartHubSweepRight;
      fieldPoints.EvilPassLeft = Red.EvilPassLeft;
      fieldPoints.EvilPassRight = Red.EvilPassRight;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Timer", turretTimer.get());
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Hub Active", isHubActive());
    isHubActive();
    if(!DriverStation.isDisabled()){
      if(DriverStation.getMatchTime() > 30){
        if(isHubActive()){
          if(SwerveSubsystem.color.get() == Alliance.Blue){
            LEDSubsystem.mLEDState = LEDState.Blue;
          }else{
            LEDSubsystem.mLEDState = LEDState.Red;
          }
        }else{
          if(SwerveSubsystem.color.get() == Alliance.Blue){
            LEDSubsystem.mLEDState = LEDState.Red;
          }else{
            LEDSubsystem.mLEDState = LEDState.Blue;
          }
        }

        if(prefireBlue){
          LEDSubsystem.mLEDState = LEDState.PrepareBlue;
          RobotContainer.driver.setRumble(RumbleType.kBothRumble, 1);
        }else if(prefireRed){
          LEDSubsystem.mLEDState = LEDState.PrepareRed;
          RobotContainer.driver.setRumble(RumbleType.kBothRumble, 1);
        }else{
          RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
        }
      }else{
        RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
        if(DriverStation.getMatchTime() > 29.5){
          LEDSubsystem.startEndgame();
          if(SwerveSubsystem.color.get() == Alliance.Blue){
            LEDSubsystem.mLEDState = LEDState.EndgameBlue;
          }else{
            LEDSubsystem.mLEDState = LEDState.EndgameRed;
          }
        }
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

    // We're teleop enabled, compute
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    boolean redInactiveFirst = false;
    if(!gameData.isEmpty()){
        switch (gameData.charAt(0)) {
        case 'R' -> redInactiveFirst = true;
        case 'B' -> redInactiveFirst = false;
        default -> {
          // If we have invalid game data, assume hub is active.
          redInactiveFirst = true;
        }
      }
    }else{//assume we won auto :)
      if(alliance.get() == Alliance.Red){
        redInactiveFirst = true;
      }else{
        redInactiveFirst = false;
      }
    }
    

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if(matchTime < 110 && matchTime > 105){
      selectPrefire(shift1Active);
    }else if(matchTime < 85 && matchTime > 80){
      selectPrefire(!shift1Active);
    }else if(matchTime < 60 && matchTime > 55){
      selectPrefire(shift1Active);
    }else if(matchTime < 35 && matchTime > 30){
      selectPrefire(shift1Active);
    }else{
      prefireBlue = false;
      prefireRed = false;
    }

    if (matchTime > 120) {// could be -> (matchTime > 130)
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

  private void selectPrefire(boolean redFirst){
    if(SwerveSubsystem.color.isPresent() && SwerveSubsystem.color.get() == Alliance.Blue){
      if(redFirst){
          prefireRed= true;
          prefireBlue = false;
        }else{
          prefireBlue = true;
          prefireRed = false;
        }
    }else{
      if(redFirst){
        prefireRed= false;
        prefireBlue = true;
      }else{
        prefireBlue = false;
        prefireRed = true;
      }
    }
    
  }

  public boolean canShoot(){
    if(isHubActive()){
      return true;
    }else if(SwerveSubsystem.color.isPresent() && SwerveSubsystem.color.get() == Alliance.Blue){
      if(prefireBlue){
        return true;
      }
    }else{
      if(prefireRed){
        return true;
      }else{
        return false;
      }
    }

    return false;
  }
}
