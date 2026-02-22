// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  public static TalonFX turretRotationMotor = new TalonFX(TurretConstants.TurretRotationID, "*");
  public static TalonFX elevationMotor = new TalonFX(TurretConstants.shooterElevationMotorID, "*");

  private static TalonFXConfiguration turretRotConfig = new TalonFXConfiguration();
  public static TalonFXConfiguration turretElevationMotorConfig = new TalonFXConfiguration();

  private static MotionMagicDutyCycle turretRotMagicCycle = new MotionMagicDutyCycle(0);
  public static MotionMagicDutyCycle elevationMagicCycle = new MotionMagicDutyCycle(0);

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
    SmartDashboard.putNumber("Turret Ticks", turretRotationMotor.getPosition().getValueAsDouble());
  }

  public static void aimTurretAtPoint(Pose2d pose){
    turretRotationMotor.setControl(turretRotMagicCycle.withPosition(turretRotationsToKraken(SwerveSubsystem.turretRotationToPose(pose).getRotations())));
  }

  public static void aimTurretAtDegree(double degrees){
    turretRotationMotor.setControl(turretRotMagicCycle.withPosition(Rotation2d.fromDegrees(degrees).getRotations()));
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

  public static void setElevation(Rotation2d rot){
    elevationMotor.setControl(elevationMagicCycle.withPosition(elevationRotationsToKraken(rot.getRotations())));
  }

  private static void configTurret(){
    turretRotationMotor.setNeutralMode(NeutralModeValue.Coast);
    turretRotationMotor.setPosition(turretRotationsToKraken(TurretConstants.TurretCableChainPoint.getRotations() - TurretConstants.TurretStartOffset.getRotations()));

    turretRotConfig.Slot0.kP = 0.1;
    turretRotConfig.Slot0.kI = 0.0;
    turretRotConfig.Slot0.kD = 0.0;

    turretRotConfig.MotionMagic.MotionMagicAcceleration = 80;
    turretRotConfig.MotionMagic.MotionMagicCruiseVelocity = 40;

    turretRotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


    turretRotationMotor.getConfigurator().apply(turretRotConfig);
  }

  public static void turretElevationConfiguration() {
    elevationMotor.setPosition(elevationRotationsToKraken(TurretConstants.TurretMaxAngle.getRotations()));

    elevationMotor.setNeutralMode(NeutralModeValue.Brake);

    turretElevationMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    turretElevationMotorConfig.Slot0.kP = 0.1;//1.8
    turretElevationMotorConfig.Slot0.kI = 0.0;//0.12
    turretElevationMotorConfig.Slot0.kD = 0.0;//0.01

    turretElevationMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
    turretElevationMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 40;

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