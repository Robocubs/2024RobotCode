package com.team1701.robot.commands;

import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
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

    private final RobotState mRobotState = new RobotState();

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
        currentPose = mRobotState.getPose2d();
        distanceToTarget = currentPose.getTranslation().getDistance(targetSpeaker);
        headingToTarget =
                new Rotation2d(targetSpeaker.getX() - currentPose.getX(), targetSpeaker.getY() - currentPose.getY());
        robotRelativeRotationDemand = headingToTarget.minus(currentPose.getRotation());
        shooterAngleFromHorizontal = new Rotation2d(
                distanceToTarget - Constants.Shooter.kShooterAxisOffset,
                FieldConstants.kBlueSpeakerOpeningCenter.getZ() - Constants.Shooter.kShooterAxisHeight);
        mShooter.setRotationBrake(true);
    }

    @Override
    public void execute() {
        if (currentPose.getRotation() != headingToTarget) {
            DriveCommands.rotateRelativeToRobot(
                    mDrive, robotRelativeRotationDemand, Constants.Drive.kFastKinematicLimits, true);
        }
        // TODO add throughBoreEncoder
        mShooter.setRotationAngle(shooterAngleFromHorizontal);

        // TODO: add check for value change
        mShooter.setRollerSpeed(Units.rotationsPerMinuteToRadiansPerSecond(Constants.Motors.kMaxKrakenRPM));
        // TODO: implement autostart/stop with sensors

        if (true /* TODO !shooterHasPiece */) {
            mShooter.stopRollers();
            shotFired = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return shotFired;
    }
}
