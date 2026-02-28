// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Subsystems.SwerveSubsystem;

/** Add your docs here. */
public class TurretLogic {
    SwerveSubsystem mSwerve;
    double trueAngle = 0;
    double zeroOfTheDerivativeOfTheDesiredAngle = TurretConstants.TurretMinAngle.getRadians();
    boolean usingLower = false;
    BrentSolver brentSolver = new BrentSolver(0.01,0.01,0.01);

    public TurretLogic(SwerveSubsystem mSwerve){
        this.mSwerve = mSwerve;
    }

    public class TurretAimPose {
        public double vel;
        public Translation3d aimPose;
        public boolean usingLower;
        public double elevationAngleDegrees;
        public TurretAimPose(){}
    }

    public TurretAimPose getAimPose(Translation3d targetPose, double distanceToTarget){
        double robotFieldXVel = mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond * Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians()) - mSwerve.getRobotRelativeSpeeds().vyMetersPerSecond * Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians());
        double robotFieldYVel = mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond * Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians()) + mSwerve.getRobotRelativeSpeeds().vyMetersPerSecond * Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians());
        double robotSpeed = Math.sqrt(Math.pow(robotFieldXVel, 2) + Math.pow(robotFieldYVel, 2));//mps

        double targetPoseRelativeToRobotX = targetPose.getX() - SwerveSubsystem.turretToField().getX();//SwerveSubsystem.poseEstimator.getEstimatedPosition().getX();
        double targetPoseRelativeToRobotY = targetPose.getY() - SwerveSubsystem.turretToField().getY();//SwerveSubsystem.poseEstimator.getEstimatedPosition().getY();

        double angleRobotVelocityToTarget = Math.atan2(robotFieldYVel, robotFieldXVel) - Math.atan2(targetPoseRelativeToRobotY, targetPoseRelativeToRobotX);

        double lateralOffsetVelocityConstant = robotSpeed * Math.sin(angleRobotVelocityToTarget) / Math.sqrt(Math.pow(targetPoseRelativeToRobotX,2) + Math.pow(targetPoseRelativeToRobotY, 2));
        double lateralOffsetVelocityX = lateralOffsetVelocityConstant * targetPoseRelativeToRobotY;//y
        double lateralOffsetVelocityY = lateralOffsetVelocityConstant * -targetPoseRelativeToRobotX;//x

        double flattenedTargetPoseX = Math.sqrt(Math.pow(targetPoseRelativeToRobotX, 2) + Math.pow(targetPoseRelativeToRobotY, 2));
        double flattenedTargetPoseY = targetPose.getZ() - TurretConstants.TurretVerticalOffset;

        double flattenedRobotVel = robotSpeed * Math.cos(angleRobotVelocityToTarget);
        double shooterVel = (5.769 * distanceToTarget + (robotSpeed*0.5)) + 35.92 + (9.06 * -flattenedRobotVel) + (0.964 * Math.pow(flattenedRobotVel,2));//robot speed added
        //double shooterVel = (23.66 + 6.13*distanceToTarget - 0.151*Math.pow(distanceToTarget, 2) - 8*flattenedRobotVel + 0.634*Math.pow(flattenedRobotVel, 2));
        double launchSpeed = 0.0754888*Math.PI*0.5*shooterVel *20/18;
        double flattenedInitialVel = Math.sqrt(Math.pow(launchSpeed, 2) - Math.pow(robotSpeed * Math.sin(angleRobotVelocityToTarget), 2));
        
        
        
        try{
            zeroOfTheDerivativeOfTheDesiredAngle = brentSolver.findRoot((double theta)-> Math.pow(flattenedInitialVel,2)*flattenedTargetPoseX*Math.cos(2*theta) - flattenedInitialVel*flattenedTargetPoseX*flattenedRobotVel*Math.cos(theta) + flattenedTargetPoseY*flattenedInitialVel*Math.sin(theta), TurretConstants.TurretMinAngle.getRadians(), TurretConstants.TurretStartElevation.getRadians());
        }catch(Exception e){
            //System.out.println("Zero of the derivative failed" + e);
        }

        try{
            trueAngle = brentSolver.findRoot((double theta)-> flattenedInitialVel*flattenedTargetPoseX*(flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*Math.sin(theta) - 4.905*Math.pow(flattenedTargetPoseX,2) - (flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*flattenedTargetPoseY, zeroOfTheDerivativeOfTheDesiredAngle, TurretConstants.TurretStartElevation.getRadians());
            usingLower = false;
        }catch(Exception e){
            try{
                trueAngle = brentSolver.findRoot((double theta)-> flattenedInitialVel*flattenedTargetPoseX*(flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*Math.sin(theta) - 4.905*Math.pow(flattenedTargetPoseX,2) - (flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*flattenedTargetPoseY, TurretConstants.TurretMinAngle.getRadians(), zeroOfTheDerivativeOfTheDesiredAngle); 
                usingLower = true;
            }catch(Exception E){
                //System.out.println("Cry" + E);
            }
        }
        
        double angleToTarget = Math.atan2(targetPoseRelativeToRobotY, targetPoseRelativeToRobotX);

        double vUnajustedX = Math.cos(trueAngle) * Math.cos(angleToTarget) * flattenedInitialVel;
        double vUnajustedY = Math.cos(trueAngle) * Math.sin(angleToTarget) * flattenedInitialVel;
        double vUnajustedZ = Math.sin(trueAngle) * flattenedInitialVel;

        double velocityOffset = 3;
        TurretAimPose turretAimPose = new TurretAimPose();
        turretAimPose.aimPose = new Translation3d(-vUnajustedX - lateralOffsetVelocityX, 
            -vUnajustedY - lateralOffsetVelocityY, 
            vUnajustedZ + TurretConstants.TurretVerticalOffset);
        turretAimPose.vel = shooterVel - velocityOffset;
        turretAimPose.usingLower = usingLower;

        return turretAimPose;
    }
}
