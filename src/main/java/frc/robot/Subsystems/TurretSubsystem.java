// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.LEDSubsystem.LEDState;
import frc.robot.RobotContainer;

public class TurretSubsystem extends SubsystemBase {
  public static TalonFX turretRotationMotor = new TalonFX(TurretConstants.TurretRotationID, "*");
  public static TalonFX elevationMotor = new TalonFX(TurretConstants.shooterElevationMotorID, "*");
  public static CANcoder turretCANcoder = new CANcoder(TurretConstants.turretEncoderID, "*");

  private static TalonFXConfiguration turretRotConfig = new TalonFXConfiguration();
  public static TalonFXConfiguration turretElevationMotorConfig = new TalonFXConfiguration();

  private static MotionMagicDutyCycle turretRotMagicCycle = new MotionMagicDutyCycle(0);
  public static MotionMagicDutyCycle elevationMagicCycle = new MotionMagicDutyCycle(0);

  public static boolean canShoot = false;

  public static class ActiveAimPose {
    public Translation3d aimPoint;
    public LEDState ledState;
    public ActiveAimPose(Translation3d aimPoint, LEDState ledState){
      this.aimPoint = aimPoint;
      this.ledState = ledState;
    }
  }

  public static ActiveAimPose activeAimPoint = new ActiveAimPose(SwerveSubsystem.color.isPresent() && SwerveSubsystem.color.get() == Alliance.Blue ? FieldConstants.HubFieldPoseBlue : FieldConstants.HubFieldPoseRed, LEDState.Hub);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    configTurret();
    turretElevationConfiguration();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Actual Position", krakenToRotation2d(Rotation2d.fromRotations(turretRotationMotor.getPosition().getValueAsDouble())).getDegrees());
    //SmartDashboard.putNumber("Turret Elevation Motor Current", elevationMotor.getStatorCurrent().getValueAsDouble());
    //SmartDashboard.putNumber("Turret Elevation Position", elevationMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevation degrees", Rotation2d.fromRotations(krakenToElevationRotations(elevationMotor.getPosition().getValueAsDouble())).getDegrees());
    //SmartDashboard.putNumber("Turret Ticks", turretRotationMotor.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Turret CAN coder", turretCANcoder.getAbsolutePosition().getValueAsDouble());
    if(RobotContainer.driver.povDown().getAsBoolean() && !RobotContainer.shouldDial){
      RobotContainer.shouldDial = true;
    }else if(RobotContainer.driver.povDown().getAsBoolean() && RobotContainer.shouldDial){
      RobotContainer.shouldDial = false;
    }
  }

  public static void aimTurretAtPoint(Pose2d pose){
    double targetRot;
    if(RobotContainer.shouldDial){
      targetRot = turretRotationsToKraken(SwerveSubsystem.turretRotationToPose450(pose).getRotations() + 0.25*RobotContainer.dial.getRawAxis(0));
      turretRotationMotor.setControl(turretRotMagicCycle.withPosition(targetRot));
    }else{
      targetRot = turretRotationsToKraken(SwerveSubsystem.turretRotationToPose450(pose).getRotations());
      turretRotationMotor.setControl(turretRotMagicCycle.withPosition(targetRot));
    }
    if(turretRotationMotor.getPosition().getValueAsDouble() > targetRot * 0.975 && turretRotationMotor.getPosition().getValueAsDouble() < targetRot * 1.975){
      canShoot = true;
    }else{
      canShoot = false;
    }
  }

  public static void aimTurretAtDegree(double degrees){
    turretRotationMotor.setControl(turretRotMagicCycle.withPosition(turretRotationsToKraken(Rotation2d.fromDegrees(degrees).getRotations())));
  }

  public static void aimtTurretAtRotation(double rot){
    turretRotationMotor.setControl(turretRotMagicCycle.withPosition(turretRotationsToKraken(rot)));
  }

  public static Rotation2d getTurretRotation(){
    return krakenToRotation2d(Rotation2d.fromRotations(turretRotationMotor.getPosition().getValueAsDouble()));
  }

  public static void shooterAim(double position) {
    elevationMotor.setControl(elevationMagicCycle.withPosition(position));
  }

  public static void shooterManualAim(double speed) {
    elevationMotor.set(speed);
  }

  public static void shooterAimZero() {
    while (elevationMotor.getStatorCurrent().getValueAsDouble() < 20) {
      shooterManualAim(.10);
    }
  }

  public static void setElevation(double degrees){
    if(degrees == Double.NaN){
      degrees = TurretConstants.TurretMaxAngle.getDegrees();
    }

    double returnDegrees = Math.max(Math.min(degrees, TurretConstants.TurretMaxAngle.getDegrees()), TurretConstants.TurretMinAngle.getDegrees());
    
    elevationMotor.setControl(elevationMagicCycle.withPosition(elevationRotationsToKraken(returnDegrees / 360)));
  }

  private static void configTurret(){
    turretRotationMotor.setNeutralMode(NeutralModeValue.Coast);
    turretRotationMotor.setPosition(turretRotationsToKraken(TurretConstants.TurretCableChainPoint.getRotations() - TurretConstants.TurretStartOffset.getRotations()) + (turretCANcoder.getAbsolutePosition().getValueAsDouble() * TurretConstants.encoderRatio));
    //turretRotationMotor.setPosition(turretRotationsToKraken(TurretConstants.TurretCableChainPoint.getRotations() - TurretConstants.TurretStartOffset.getRotations()));

    turretRotConfig.Slot0.kP = 0.22;
    turretRotConfig.Slot0.kI = 0.005;
    turretRotConfig.Slot0.kD = 0.001;

    turretRotConfig.MotionMagic.MotionMagicAcceleration = 150;
    turretRotConfig.MotionMagic.MotionMagicCruiseVelocity = 100;

    turretRotConfig.CurrentLimits.StatorCurrentLimit = 40;

    turretRotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


    turretRotationMotor.getConfigurator().apply(turretRotConfig);
  }

  private static void turretElevationConfiguration() {
    elevationMotor.setNeutralMode(NeutralModeValue.Brake);

    turretElevationMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    elevationMotor.setPosition(elevationRotationsToKraken(TurretConstants.TurretStartElevation.getRotations()));

    turretElevationMotorConfig.Slot0.kP = 0.2;//1.8
    turretElevationMotorConfig.Slot0.kI = 0.001;//0.12
    turretElevationMotorConfig.Slot0.kD = 0.0;//0.01

    turretElevationMotorConfig.MotionMagic.MotionMagicAcceleration = 300;
    turretElevationMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 240;

    turretElevationMotorConfig.CurrentLimits.StatorCurrentLimit = 40;

    elevationMotor.getConfigurator().apply(turretElevationMotorConfig);
  }

  public static double turretRotationsToKraken(double rot){
    return rot * TurretConstants.turretRatio;
  }

  private static Rotation2d krakenToRotation2d(Rotation2d krakenRot)  {
    return Rotation2d.fromRotations(krakenRot.getRotations()/TurretConstants.turretRatio);
  }

  private static double elevationRotationsToKraken(double rot){
    return rot * TurretConstants.elevationRatio;
  }

  private static double krakenToElevationRotations(double krak){
    return krak/TurretConstants.elevationRatio;
  }

}