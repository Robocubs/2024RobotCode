package com.team1701.robot.commands;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.util.FieldUtil;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command {
    private static final String kLoggingPrefix = "Command/Shoot/";
    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kSpeedToleranceRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "SpeedToleranceRadiansPerSecond", 25.0);
    private Rotation2d headingTolerance;

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;
    private final boolean mWaitForHeading;
    private final boolean mWaitForSpeed;
    private final boolean mWaitForAngle;
    private final Timer mTimer = new Timer();

    private TimeLockedBoolean mLockedReadyToShoot;
    private boolean mShooting;
    private boolean mStartedWithNote;

    public Shoot(Shooter shooter, Indexer indexer, RobotState robotState, boolean waitForHeading) {
        this(shooter, indexer, robotState, waitForHeading, true, true);
    }

    public Shoot(
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            boolean waitForHeading,
            boolean waitForSpeed,
            boolean waitForAngle) {
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        mWaitForHeading = waitForHeading;
        mWaitForSpeed = waitForSpeed;
        mWaitForAngle = waitForAngle;

        mLockedReadyToShoot = new TimeLockedBoolean(.1, Timer.getFPGATimestamp());

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        mShooting = false;
        mStartedWithNote = mRobotState.hasNote();
        mLockedReadyToShoot.update(false, Timer.getFPGATimestamp());
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        if (mRobotState.isClimbMode()) {
            cancel();
            return;
        }
        var currentPose = mRobotState.getPose2d();
        var fieldRelativeSpeeds = mRobotState.getFieldRelativeSpeeds();
        var droppedVelocity = Constants.Shooter.kShooterSpeedInterpolator.get(mRobotState.getDistanceToSpeaker())
                * Constants.Shooter.kRollerSpeedToNoteSpeed;
        var robotVelocityTowardsSpeaker = ChassisSpeeds.fromFieldRelativeSpeeds(
                        fieldRelativeSpeeds, mRobotState.getSpeakerHeading())
                .vxMetersPerSecond;
        var timeInAir = mRobotState.getDistanceToSpeaker() / (robotVelocityTowardsSpeaker + droppedVelocity);
        var endTranslation = new Translation2d(
                currentPose.getX() + fieldRelativeSpeeds.vxMetersPerSecond * timeInAir,
                currentPose.getY() + fieldRelativeSpeeds.vyMetersPerSecond * timeInAir);

        var setpoint = mRobotState.isSpeakerMode()
                ? ShooterUtil.calculateShooterSetpoint(FieldUtil.getDistanceToSpeaker(endTranslation))
                : ShooterUtil.calculateStationarySetpoint(mRobotState);

        mShooter.setSetpoint(setpoint);

        var targetHeading = mRobotState.isSpeakerMode()
                ? mRobotState
                        .getSpeakerPose()
                        .toTranslation2d()
                        .minus(endTranslation)
                        .getAngle()
                        .minus(setpoint.releaseAngle())
                : mRobotState.getStationaryTargetHeading();
        headingTolerance = mRobotState.getToleranceSpeakerHeading();

        var atAngle = !mWaitForAngle
                || GeometryUtil.isNear(
                        mShooter.getAngle(), setpoint.angle(), Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading =
                !mWaitForHeading || GeometryUtil.isNear(targetHeading, currentPose.getRotation(), headingTolerance);

        var atSpeed = !mWaitForSpeed
                || setpoint.speeds()
                        .allMatch(mShooter.getRollerSpeedsRadiansPerSecond(), kSpeedToleranceRadiansPerSecond.get());

        if (mLockedReadyToShoot.update(atAngle && atHeading && atSpeed, Timer.getFPGATimestamp())) {
            mIndexer.setForwardShoot();
            mShooting = true;
        }

        if (!mShooting) {
            if (mIndexer.hasNoteAtExit()) {
                mIndexer.stop();
            } else {
                mIndexer.setForwardLoad();
            }
        }

        mRobotState.setShootingState(new ShootingState(setpoint, true, atAngle, atSpeed, atHeading, false, mShooting));
    }

    @Override
    public void end(boolean interrupted) {
        mShooting = false;
        mTimer.stop();

        mRobotState.setShootingState(ShootingState.kDefault);
        mShooter.stop();
        mIndexer.stop();
    }

    @Override
    public boolean isFinished() {
        var doesNotHaveNote = !mRobotState.hasNote() && (mStartedWithNote || mTimer.hasElapsed(0.25));
        return mShooting && doesNotHaveNote;
    }
}
