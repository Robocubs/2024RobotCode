package com.team1701.robot.commands;

import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command {
    private final Shooter mShooter;
    private final Drive mDrive;
    private double distanceToTarget;
    private Rotation2d headingToTarget;
    private Rotation2d shooterAngleFromHorizontal;
    private Pose2d currentPose;
    private final Translation2d targetSpeaker;
    private Rotation2d robotRelativeRotationDemand;
    private boolean shotFired;

    public Shoot(Shooter shooter, Drive drive, boolean allianceIsBlue) {
        mShooter = shooter;
        mDrive = drive;
        targetSpeaker = allianceIsBlue
                ? FieldConstants.kBlueSpeakerOpeningCenter.toTranslation2d()
                : FieldConstants.kRedSpeakerOpeningCenter.toTranslation2d();
        addRequirements(shooter, drive);
    }

    @Override
    public void initialize() {
        shotFired = false;
        currentPose = PoseEstimator.getInstance().getPose2d();
        distanceToTarget = currentPose.getTranslation().getDistance(targetSpeaker);
        headingToTarget =
                new Rotation2d(targetSpeaker.getX() - currentPose.getX(), targetSpeaker.getY() - currentPose.getY());
        robotRelativeRotationDemand = headingToTarget.minus(currentPose.getRotation());
        shooterAngleFromHorizontal = new Rotation2d(
                distanceToTarget - Constants.Shooter.kShooterAxisOffset,
                FieldConstants.kBlueSpeakerOpeningCenter.getZ() - Constants.Shooter.kShooterAxisHeight);
    }

    @Override
    public void execute() {
        if (currentPose.getRotation() != headingToTarget) {
            DriveCommands.rotateRelativeToRobot(
                    mDrive, robotRelativeRotationDemand, Constants.Drive.kFastKinematicLimits, true);
        }
        // TODO add throughBoreEncoder
        if (true /* TODO throughBoreEncoder is not at position (within tolerance) */) {
            mShooter.setRotationAngle(shooterAngleFromHorizontal);
        } else {
            // TODO: add check for value change
            mShooter.setRotationBrake(true);
            mShooter.setRollerSpeed(Units.rotationsPerMinuteToRadiansPerSecond(Constants.Motors.kMaxKrakenRPM));
        }
        // TODO: implement autostart/stop with sensors

        if (true /* TODO !shooterHasPiece */) {
            mShooter.stopRollers();
            shotFired = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRollers();
        mShooter.setRotationBrake(false); // so angle can be continually adjusted
    }

    @Override
    public boolean isFinished() {
        return shotFired;
    }
}
