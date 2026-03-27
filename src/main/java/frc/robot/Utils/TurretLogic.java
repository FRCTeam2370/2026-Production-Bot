// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import java.security.Timestamp;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.MsvcRuntimeException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.FieldInfo;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

/** Add your docs here. */
public class TurretLogic {
    SwerveSubsystem mSwerve;
    double trueAngle = 0;
    double zeroOfTheDerivativeOfTheDesiredAngle = TurretConstants.ElevationMinAngle.getRadians();
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

        //this block of code calculates the velocity and speed of the Turret relative to the field using the velocity of the robot and some math
        //Because the turret isn't in the center of the robot, we need to calculate its velocity when we are turning
        double tangentialVelocityX = -SwerveSubsystem.gyro.getAngularVelocityZWorld().getValueAsDouble() * (Math.PI/180) * TurretConstants.RobotToTurret.getY();
        double tangentialVelocityY = SwerveSubsystem.gyro.getAngularVelocityZWorld().getValueAsDouble() * (Math.PI/180) * TurretConstants.RobotToTurret.getX();

        //next, we calculated the turret's robot relative veolicty using the robots xy velocity and adding the tangetial velocities respectively
        double turretRelativeXVel = mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond + tangentialVelocityX;
        double turretRelativeYVel = mSwerve.getRobotRelativeSpeeds().vyMetersPerSecond + tangentialVelocityY;

        //lastly, we get the field relative velocities using the angle of the gyro and calculate the speed of the turret using those values
        double turretFieldXVel = turretRelativeXVel * Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians()) - turretRelativeYVel * Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians());
        double turretFieldYVel = turretRelativeXVel * Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians()) + turretRelativeYVel * Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians());
        double turretSpeed = Math.sqrt(Math.pow(turretFieldXVel, 2) + Math.pow(turretFieldYVel, 2));//mps

        //logging stuff
        SmartDashboard.putNumber("Turret Velocity", turretSpeed);
        SmartDashboard.putNumber("Turret Vel X", turretFieldXVel);
        SmartDashboard.putNumber("Turret Vel Y", turretRelativeYVel);

        //gets the targetpose relative to the turret's position
        double targetPoseRelativeToRobotX = targetPose.getX() - SwerveSubsystem.turretToField().getX();//SwerveSubsystem.poseEstimator.getEstimatedPosition().getX();
        double targetPoseRelativeToRobotY = targetPose.getY() - SwerveSubsystem.turretToField().getY();//SwerveSubsystem.poseEstimator.getEstimatedPosition().getY();

        //gets the angle between the velocity of the turret's angle to the target and the angle to the target
        double angleRobotVelocityToTarget = Math.atan2(/*robotFieldYVel*/turretFieldYVel, turretFieldXVel/*robotFieldXVel*/) - Math.atan2(targetPoseRelativeToRobotY, targetPoseRelativeToRobotX);


        double lateralOffsetVelocityConstant = /*robotSpeed*/turretSpeed * Math.sin(angleRobotVelocityToTarget) / Math.sqrt(Math.pow(targetPoseRelativeToRobotX,2) + Math.pow(targetPoseRelativeToRobotY, 2));
        //^this is the Speed of the Turret * the y component of the angleRobotVelocityToTarget / the distance from the turret to the target
        double lateralOffsetVelocityX = lateralOffsetVelocityConstant * targetPoseRelativeToRobotY;
        double lateralOffsetVelocityY = lateralOffsetVelocityConstant * -targetPoseRelativeToRobotX;

        /*we now turn a 3d problem into a 2d problem for calculating the z offset
         * our new flattened x axis is the distance from the turret to the target
         * our new flattened y axis is the z axis form the 3d problem
        */
        double flattenedTargetPoseX = Math.sqrt(Math.pow(targetPoseRelativeToRobotX, 2) + Math.pow(targetPoseRelativeToRobotY, 2));
        double flattenedTargetPoseY = targetPose.getZ() - TurretConstants.TurretVerticalOffset;

        /*here we flatten the turret's velocity on the flattned x axis above
         * this gives us a velocity of the turret in the direction of our desired aimpoint
         * we can calculate our shooter velocity and launch speed of the ball with some simple calculations
         * then, we calculate the flattened initial velocity of the ball by subtracting our turret's flattened velocity from the launch speed
         */
        double flattenedRobotVel = /*robotSpeed*/turretSpeed * Math.cos(angleRobotVelocityToTarget);
        double shooterVel = (5.769 * distanceToTarget + (/*robotSpeed*/ turretSpeed*0.5)) + 35.92 + (9.06 * -flattenedRobotVel) + (0.964 * Math.pow(flattenedRobotVel,2)) + (TurretSubsystem.activeAimPoint.aimPoint == FieldInfo.fieldPoints.PassPose1 || TurretSubsystem.activeAimPoint.aimPoint == FieldInfo.fieldPoints.PassPose2 ? 0 : 3) /*constant*/;//robot speed added
        //double shooterVel = (23.66 + 6.13*distanceToTarget - 0.151*Math.pow(distanceToTarget, 2) - 8*flattenedRobotVel + 0.634*Math.pow(flattenedRobotVel, 2));
        double launchSpeed = 0.0754888 * Math.PI * 0.5 * shooterVel * 20/18;
        double flattenedInitialVel = Math.sqrt(Math.pow(launchSpeed, 2) - Math.pow(/*robotSpeed*/turretSpeed * Math.sin(angleRobotVelocityToTarget), 2));
        
        
        //Now for the fun stuff
        /*Here we have some try catches that house brent solvers
         * these solvers are used to find the zeros of important functions that describe how the elevation of the turret should aim (trueAngle)
         */
        try{
            zeroOfTheDerivativeOfTheDesiredAngle = brentSolver.findRoot((double theta)-> Math.pow(flattenedInitialVel,2)*flattenedTargetPoseX*Math.cos(2*theta) - flattenedInitialVel*flattenedTargetPoseX*flattenedRobotVel*Math.cos(theta) + flattenedTargetPoseY*flattenedInitialVel*Math.sin(theta), TurretConstants.ElevationMinAngle.getRadians(), TurretConstants.TurretStartElevation.getRadians());
        }catch(Exception e){
            //System.out.println("Zero of the derivative failed" + e);
        }

        try{
            trueAngle = brentSolver.findRoot((double theta)-> flattenedInitialVel*flattenedTargetPoseX*(flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*Math.sin(theta) - 4.905*Math.pow(flattenedTargetPoseX,2) - (flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*flattenedTargetPoseY, zeroOfTheDerivativeOfTheDesiredAngle, TurretConstants.TurretStartElevation.getRadians());
            usingLower = false;
        }catch(Exception e){
            try{
                trueAngle = brentSolver.findRoot((double theta)-> flattenedInitialVel*flattenedTargetPoseX*(flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*Math.sin(theta) - 4.905*Math.pow(flattenedTargetPoseX,2) - (flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*flattenedTargetPoseY, TurretConstants.ElevationMinAngle.getRadians(), zeroOfTheDerivativeOfTheDesiredAngle); 
                usingLower = true;
            }catch(Exception E){
                //System.out.println("Cry" + E);
            }
        }
        
        //this one's pretty self explanatory (it just gets the angle from the the turret to the target pose)
        double angleToTarget = Math.atan2(targetPoseRelativeToRobotY, targetPoseRelativeToRobotX);

        //calculate offsets using trueAngle, angleToTarget, and flattenedIntitialVel for each translation
        double vUnajustedX = Math.cos(trueAngle) * Math.cos(angleToTarget) * flattenedInitialVel;
        double vUnajustedY = Math.cos(trueAngle) * Math.sin(angleToTarget) * flattenedInitialVel;
        double vUnajustedZ = Math.sin(trueAngle) * flattenedInitialVel;

        //add a velocity offset for a little extra tuning
        double velocityOffset = distanceToTarget*flattenedRobotVel*2;

        //creates a new varial of type TurretAimPose that holds the Translation3d and double for the aimpose and the turret velocity
        TurretAimPose turretAimPose = new TurretAimPose();
        turretAimPose.aimPose = new Translation3d(-vUnajustedX - lateralOffsetVelocityX, 
            -vUnajustedY - lateralOffsetVelocityY,
            vUnajustedZ + TurretConstants.TurretVerticalOffset);
        
        turretAimPose.vel = shooterVel;// + velocityOffset;
        //This guy is just so we know if the brent solver calculated the angle for the lob shot or the direct shot
        turretAimPose.usingLower = usingLower;

        return turretAimPose;
    }

    public TurretAimPose jacobsStupidOne(Translation3d targetPose, boolean useGreater){
        Pose2d turretFieldPose = SwerveSubsystem.turretToField();
        //this block of code calculates the velocity and speed of the Turret relative to the field using the velocity of the robot and some math
        //Because the turret isn't in the center of the robot, we need to calculate its velocity when we are turning
        double tangentialVelocityX = -SwerveSubsystem.gyro.getAngularVelocityZWorld().getValueAsDouble() * (Math.PI/180) * TurretConstants.RobotToTurret.getY();
        double tangentialVelocityY = SwerveSubsystem.gyro.getAngularVelocityZWorld().getValueAsDouble() * (Math.PI/180) * TurretConstants.RobotToTurret.getX();

        //next, we calculated the turret's robot relative veolicty using the robots xy velocity and adding the tangetial velocities respectively
        double turretRelativeXVel = mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond + tangentialVelocityX;
        double turretRelativeYVel = mSwerve.getRobotRelativeSpeeds().vyMetersPerSecond + tangentialVelocityY;

        double fieldRelTurXVel = turretRelativeXVel*Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians()) - turretRelativeYVel*Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians());
        double fieldRelTurYVel = turretRelativeXVel*Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians()) + turretRelativeYVel*Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians());

        //lastly, we get the field relative velocities using the angle of the gyro and calculate the speed of the turret using those values
        //double turretFieldXVel = turretRelativeXVel * Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians()) - turretRelativeYVel * Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians());
        //double turretFieldYVel = turretRelativeXVel * Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians()) + turretRelativeYVel * Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians());
        double turretSpeed = Math.sqrt(Math.pow(fieldRelTurXVel, 2) + Math.pow(fieldRelTurYVel, 2));//mps

        double targetPoseXRelativeToTurret = targetPose.getX() - turretFieldPose.getX();
        double targetPoseYRelativeToTurret = targetPose.getY() - turretFieldPose.getY();

        double targetPoseRelativeToTurretVelX = targetPoseXRelativeToTurret - fieldRelTurXVel;
        double targetPoseRelativeToTurretVelY = targetPoseYRelativeToTurret - fieldRelTurYVel;

        double angleRelativeToAjustedTarget = Math.atan2(targetPoseRelativeToTurretVelY, targetPoseRelativeToTurretVelX);
        double distanceToAdjustedTarget = Math.sqrt(Math.pow(targetPoseRelativeToTurretVelX, 2) + Math.pow(targetPoseRelativeToTurretVelY, 2));

        double flattenedRobotVel = turretSpeed * Math.cos(angleRelativeToAjustedTarget);
        double shooterVel = 1.15*Math.pow(distanceToAdjustedTarget, 1.04) +2*distanceToAdjustedTarget + 50;//5*distanceToAdjustedTarget + 50;//distance to target in meters + 50 just because (idk I'll make a better function later)
        double launchSpeed = 0.0754888 * Math.PI * 0.5 * shooterVel * 20/18;//Launch speed of the ball 

        //double flattenedX = Math.sqrt(Math.pow(targetPoseRelativeToTurretVelX, 2) + Math.pow(targetPoseRelativeToTurretVelY, 2));
        double flattenedY = targetPose.getZ() - TurretConstants.TurretVerticalOffset;

        //SmartDashboard.putBoolean("valid lauch speed", Math.pow(launchSpeed, 4) > (-9.81 * (-9.81*(Math.pow(flattenedX, 2) + 2*flattenedY*Math.pow(launchSpeed, 4)))));

        double g = 9.81;
        double a = (g*Math.pow(distanceToAdjustedTarget, 2)) / (2*Math.pow(launchSpeed, 2));
        double b = -distanceToAdjustedTarget;
        double c = flattenedY + a;

        double theta = Math.atan2(-b + Math.sqrt(Math.pow(b, 2) - 4*a*c), 2*a);
        double theta2 = Math.atan2(-b - Math.sqrt(Math.pow(b, 2) - 4*a*c),  2*a);

        // double aimTheta1 = Math.atan2(Math.pow(launchSpeed, 2) + Math.sqrt(Math.pow(launchSpeed, 4) - (-9.81 * (-9.81*(Math.pow(flattenedX, 2) + 2*flattenedY*Math.pow(launchSpeed, 4))))), -9.81 * flattenedX);
        // double aimTheta2 = Math.atan2(Math.pow(launchSpeed, 2) - Math.sqrt(Math.pow(launchSpeed, 4) - (-9.81 * (-9.81*(Math.pow(flattenedX, 2) + 2*flattenedY*Math.pow(launchSpeed, 4))))), -9.81 * flattenedX);

        //double aimPoseFieldX = targetPoseRelativeToTurretVelX * Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians()) - targetPoseRelativeToTurretVelX * Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians());
        //double aimPoseFieldY = targetPoseRelativeToTurretVelX * Math.sin(SwerveSubsystem.getgyro0to360(270).getRadians()) + targetPoseRelativeToTurretVelY * Math.cos(SwerveSubsystem.getgyro0to360(270).getRadians());
        double aimPoseFieldX = turretFieldPose.getX() + targetPoseRelativeToTurretVelX;
        double aimPoseFieldY = turretFieldPose.getY() + targetPoseRelativeToTurretVelY;

        SmartDashboard.putNumber("aimTheta1", theta);
        SmartDashboard.putNumber("aimTheta2", theta2);
        SwerveSubsystem.field.getObject("Jacob's AimPose").setPose(new Pose2d(aimPoseFieldX, aimPoseFieldY, new Rotation2d()));

        TurretAimPose returnPose = new TurretAimPose();
        returnPose.vel = shooterVel + 5;
        returnPose.aimPose = new Translation3d(aimPoseFieldX, aimPoseFieldY, flattenedY);
        if(useGreater && distanceToAdjustedTarget < 4){
            returnPose.elevationAngleDegrees = Math.toDegrees(theta);
        }else{
            returnPose.elevationAngleDegrees = Math.toDegrees(theta2);
        }
        

        return returnPose;
    }
}
