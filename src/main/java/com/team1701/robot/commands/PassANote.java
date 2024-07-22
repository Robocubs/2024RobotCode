package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.Util;
import com.team1701.lib.util.tuning.LoggedTunableBoolean;
import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.lib.util.tuning.LoggedTunableValue;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSetpoint;
import com.team1701.robot.util.FieldUtil;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class PassANote extends Command {
    private static final String kLoggingPrefix = "Command/PassANote/";

    private static final LoggedTunableBoolean mTuningEnabled =
            new LoggedTunableBoolean(kLoggingPrefix + "TuningEnabled", false);

    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.02);

    private static final TrapezoidProfile.State kZeroState = new TrapezoidProfile.State(0.0, 0.0);
    private static final KinematicLimits kKinematicLimits = Constants.Drive.kFastTrapezoidalKinematicLimits;

    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity",
            Constants.Drive.kFastTrapezoidalKinematicLimits.maxDriveVelocity() / Constants.Drive.kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAngularAcceleration", kMaxAngularVelocity.get() / 2.0);

    private Supplier<Rotation2d> mHeadingTolerance;

    private final Drive mDrive;
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;
    private final Supplier<Translation2d> mFieldRelativeSpeeds;
    private final PIDController mRotationController;

    @AutoLogOutput
    private final boolean mUseMidTarget;

    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = kZeroState;

    private boolean mShooting;
    private TimeLockedBoolean mLockedReadyToShoot;

    public PassANote(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            Supplier<Translation2d> fieldRelativeSpeeds,
            Supplier<Rotation2d> headingTolerance) {
        this(drive, shooter, indexer, robotState, fieldRelativeSpeeds, headingTolerance, false);
    }

    public PassANote(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            Supplier<Translation2d> fieldRelativeSpeeds,
            Supplier<Rotation2d> headingTolerance,
            boolean useMidTarget) {
        mDrive = drive;
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        mFieldRelativeSpeeds = fieldRelativeSpeeds;
        mRotationController = new PIDController(6.0, 0, 0, Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxAngularVelocity.get(), kMaxAngularAcceleration.get()));
        mHeadingTolerance = headingTolerance;
        mUseMidTarget = useMidTarget;

        mLockedReadyToShoot = new TimeLockedBoolean(.1, Timer.getFPGATimestamp());

        addRequirements(mDrive, shooter, indexer);
    }

    @Override
    public void initialize() {
        mShooting = false;
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);

        mRotationController.reset();
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);

        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        var headingError = mRobotState.getHeading().minus(mRobotState.getLongPassHeading());
        mRotationState = new TrapezoidProfile.State(
                MathUtil.angleModulus(headingError.getRadians()), fieldRelativeChassisSpeeds.omegaRadiansPerSecond);

        mLockedReadyToShoot.update(false, Timer.getFPGATimestamp());

        LoggedTunableValue.ifChanged(
                hashCode(),
                () -> {
                    mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            Math.min(
                                    kMaxAngularVelocity.get(),
                                    kKinematicLimits.maxDriveVelocity() / Constants.Drive.kModuleRadius),
                            Math.min(
                                    kMaxAngularAcceleration.get(),
                                    kKinematicLimits.maxDriveAcceleration() / Constants.Drive.kModuleRadius)));
                    mRotationController.setPID(6, 0, 0);
                },
                kMaxAngularVelocity,
                kMaxAngularAcceleration);
    }

    @Override
    public void execute() {
        var currentPose = mRobotState.getPose2d();
        var fieldRelativeSpeeds = mFieldRelativeSpeeds.get();

        var droppedVelocity = ShooterUtil.calculatePassingSpeed(
                        mUseMidTarget ? mRobotState.getMidPassDistance() : mRobotState.getLongPassDistance())
                * Constants.Shooter.kRollerSpeedToNoteSpeed;
        var robotVelocityTowardsTarget = fieldRelativeSpeeds
                .rotateBy(mRobotState.getLongPassHeading().unaryMinus())
                .getX();

        var timeInAir = (mUseMidTarget ? mRobotState.getMidPassDistance() : mRobotState.getLongPassDistance())
                / (robotVelocityTowardsTarget + droppedVelocity);

        var endTranslation = new Translation2d(
                currentPose.getX() + fieldRelativeSpeeds.getX() * timeInAir,
                currentPose.getY() + fieldRelativeSpeeds.getY() * timeInAir);

        var shooterSetpoint = mTuningEnabled.get()
                ? new ShooterSetpoint(
                        Constants.Shooter.kTunableShooterSpeedRadiansPerSecond.get(),
                        Rotation2d.fromRadians(Constants.Shooter.kTunableShooterAngleRadians.get()))
                : ShooterUtil.calculatePassingSetpoint(
                        mUseMidTarget
                                ? FieldUtil.getDistanceToMidPassTarget(endTranslation)
                                : FieldUtil.getDistanceToLongPassTarget(endTranslation));

        mShooter.setSetpoint(shooterSetpoint);

        var targetHeading = mUseMidTarget
                ? mRobotState.getMidPassHeading().minus(shooterSetpoint.releaseAngle())
                : mRobotState.getLongPassHeading().minus(shooterSetpoint.releaseAngle());
        var headingError = mRobotState.getHeading().minus(targetHeading);

        Rotation2d headingSetpoint;
        double rotationalVelocity;
        if (GeometryUtil.isNear(GeometryUtil.kRotationIdentity, headingError, mHeadingTolerance.get())
                && Util.epsilonEquals(fieldRelativeSpeeds.getX(), 0)
                && Util.epsilonEquals(fieldRelativeSpeeds.getY(), 0)) {
            rotationalVelocity = 0;
            mRotationState = kZeroState;
            headingSetpoint = targetHeading;
        } else {
            var rotationPidOutput = mRotationController.calculate(headingError.getRadians(), 0);
            mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, kZeroState);
            rotationalVelocity = mRotationState.velocity + rotationPidOutput;
            headingSetpoint = Rotation2d.fromRadians(targetHeading.getRadians() + mRotationState.position);
        }

        mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds.getX(), fieldRelativeSpeeds.getY(), rotationalVelocity, currentPose.getRotation()));

        var atSpeed = shooterSetpoint.speeds().allMatch(mShooter.getRollerSpeedsRadiansPerSecond(), 50.0);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), shooterSetpoint.angle(), Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading = GeometryUtil.isNear(
                targetHeading, mRobotState.getHeading(), Constants.Shooter.kPassingHeadingTolerance);

        var atPose = (mRobotState.getLongPassDistance() > 6 && !mRobotState.inOpponentWing()) || mUseMidTarget;

        if (mLockedReadyToShoot.update(atSpeed && atAngle && atHeading && atPose, Timer.getFPGATimestamp())) {
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

        mRobotState.setShootingState(
                new ShootingState(shooterSetpoint, true, atAngle, atSpeed, atHeading, atPose, mShooting));
        Logger.recordOutput(kLoggingPrefix + "AtPose", atPose);
        Logger.recordOutput(kLoggingPrefix + "HeadingSetpoint", headingSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        mShooting = false;

        mRobotState.setShootingState(ShootingState.kDefault);
        mShooter.stop();
        mIndexer.stop();
        mDrive.stop();
    }
}
