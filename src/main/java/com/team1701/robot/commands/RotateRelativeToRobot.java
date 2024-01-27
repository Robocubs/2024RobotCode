package com.team1701.robot.commands;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/* Note: positive rotations are counterclockwise (turning left) */

public class RotateRelativeToRobot extends Command {
    private RobotState mRobotState = new RobotState();
    private static final String kLoggingPrefix = "Command/RotateRelativeToRobot/";
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final KinematicLimits kMaxKinematicLimits = Constants.Drive.kFastTrapezoidalKinematicLimits;
    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity", kMaxKinematicLimits.maxDriveVelocity() / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAngularAcceleration", kMaxAngularVelocity.get() / 2.0);
    // TODO: Update PID values
    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 4.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.0);
    private static final LoggedTunableNumber kRotationToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "RotationToleranceRadians", 0.01);

    private final Drive mDrive;
    private final Rotation2d mTargetRelativeRotation;
    private Rotation2d mTargetFieldRotation;
    private final KinematicLimits mKinematicLimits;
    private final boolean mFinishAtRotation;
    private final PIDController mRotationController;

    private Rotation2d mRotationalOffset = GeometryUtil.kRotationIdentity;
    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = new TrapezoidProfile.State();

    RotateRelativeToRobot(
            Drive drive, Rotation2d targetRelativeRotation, KinematicLimits kinematicLimits, boolean finishAtRotation) {
        mDrive = drive;
        mTargetRelativeRotation = targetRelativeRotation;
        mTargetFieldRotation = new Rotation2d();
        mKinematicLimits = kinematicLimits;
        mFinishAtRotation = finishAtRotation;

        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxAngularVelocity.get(), kMaxAngularAcceleration.get()));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);
        mRotationalOffset = mRobotState.getPose2d().getRotation();

        Logger.recordOutput(kLoggingPrefix + "requestedRotation/robotRelative", mTargetRelativeRotation);
        Logger.recordOutput(kLoggingPrefix + "requestedRotation/fieldRelative", mTargetFieldRotation);

        mRotationController.reset();

        var currentActualRotation = mRobotState.getPose2d().getRotation();
        mTargetFieldRotation = currentActualRotation.plus(mTargetRelativeRotation);
        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        mRotationState = new TrapezoidProfile.State(
                MathUtil.inputModulus(
                        currentActualRotation.getRadians(),
                        mTargetFieldRotation.getRadians() - Math.PI,
                        mTargetFieldRotation.getRadians() + Math.PI),
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput(kLoggingPrefix + "SuccessfullyInitialized", true);
    }

    @Override
    public void execute() {
        var hash = hashCode();
        if (kMaxAngularVelocity.hasChanged(hash)
                || kMaxAngularAcceleration.hasChanged(hash)
                || kRotationKp.hasChanged(hash)
                || kRotationKi.hasChanged(hash)
                || kRotationKd.hasChanged(hash)) {
            mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    Math.min(kMaxAngularVelocity.get(), mKinematicLimits.maxDriveVelocity() / kModuleRadius),
                    Math.min(kMaxAngularAcceleration.get(), mKinematicLimits.maxDriveAcceleration() / kModuleRadius)));
            mRotationController.setPID(kRotationKp.get(), kRotationKi.get(), kRotationKd.get());
        }

        var currentActualRotation = mRobotState.getPose2d().getRotation();

        // Calculate rotational velocity
        var rotationPidOutput =
                mRotationController.calculate(currentActualRotation.getRadians(), mRotationState.position);
        var rotationTargetState = new TrapezoidProfile.State(mTargetFieldRotation.getRadians(), 0.0);
        mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, rotationTargetState);
        var rotationalVelocity = mRotationState.velocity + rotationPidOutput;

        // Set drive outputs
        var atTargetRotation = GeometryUtil.isNear(
                mTargetRelativeRotation.plus(mRotationalOffset),
                currentActualRotation,
                Rotation2d.fromRadians(kRotationToleranceRadians.get()));
        if (atTargetRotation) {
            mDrive.stop();
            currentActualRotation = mRobotState.getPose2d().getRotation();
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotationalVelocity, currentActualRotation));
            currentActualRotation = Rotation2d.fromRadians(mRotationState.position);
        }

        Logger.recordOutput(
                kLoggingPrefix + "RotationError", Rotation2d.fromRadians(mRotationController.getPositionError()));
        Logger.recordOutput(kLoggingPrefix + "ActualFieldRotation", mRotationalOffset);
        Logger.recordOutput(kLoggingPrefix + "TargetRelativeRotation", mTargetRelativeRotation);
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return mFinishAtRotation && atTargetPose();
    }

    public boolean atTargetPose() {
        var currentRotation = mRobotState.getPose2d().getRotation();
        var rotationError = (mTargetRelativeRotation.plus(mRotationalOffset)).minus(currentRotation);

        return Util.inRange(rotationError.getRadians(), kRotationToleranceRadians.get())
                && GeometryUtil.isNear(
                        mTargetRelativeRotation.plus(mRotationalOffset),
                        currentRotation,
                        Rotation2d.fromRadians(kRotationToleranceRadians.get()));
    }
}
