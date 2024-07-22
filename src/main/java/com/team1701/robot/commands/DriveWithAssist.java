package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.lib.util.tuning.LoggedTunableValue;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithAssist extends Command {
    private static final String kLoggingPrefix = "Command/DriveWithAimAssist/";

    private static final LoggedTunableNumber kLoopsLatency =
            new LoggedTunableNumber(kLoggingPrefix + "LoopsLatency", 2.0);
    private static final LoggedTunableNumber kTranslationKp =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationnKp", 2.0);
    private static final LoggedTunableNumber kTranslationKi =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKi", 0.0);
    private static final LoggedTunableNumber kTranslationKd =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKd", 0.0);

    private static final double kTranslationTolerance = 0.01;
    private static final double kMaxTranslationPIDOutput = 0.75;

    private final Drive mDrive;
    private final RobotState mRobotState;
    private final DoubleSupplier mThrottleSupplier;
    private final DoubleSupplier mStrafeSupplier;
    private final DoubleSupplier mRotationSupplier;
    private final Supplier<KinematicLimits> mKinematicLimits;
    private final PIDController mTranslationController;

    public DriveWithAssist(
            Drive drive,
            RobotState robotState,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier,
            Supplier<KinematicLimits> kinematicLimits) {
        mDrive = drive;
        mRobotState = robotState;
        mThrottleSupplier = throttleSupplier;
        mStrafeSupplier = strafeSupplier;
        mRotationSupplier = rotationSupplier;
        mKinematicLimits = kinematicLimits;
        mTranslationController = new PIDController(
                kTranslationKp.get(), kTranslationKi.get(), kTranslationKd.get(), Constants.kLoopPeriodSeconds);
        mTranslationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(mKinematicLimits.get());

        mTranslationController.reset();

        LoggedTunableValue.ifChanged(
                hashCode(),
                () -> {
                    mTranslationController.setPID(kTranslationKp.get(), kTranslationKi.get(), kTranslationKd.get());
                },
                kTranslationKp,
                kTranslationKi,
                kTranslationKd);
    }

    @Override
    public void execute() {
        mDrive.setKinematicLimits(mKinematicLimits.get());

        var currentPose = mRobotState.getPose2d();
        var endTranslation = new Translation2d(
                currentPose.getX()
                        + mThrottleSupplier.getAsDouble() * Constants.kLoopPeriodSeconds * kLoopsLatency.get(),
                currentPose.getY()
                        + mStrafeSupplier.getAsDouble() * Constants.kLoopPeriodSeconds * kLoopsLatency.get());

        var translationVelocities = DriveCommands.calculateDriveWithJoysticksVelocities(
                mThrottleSupplier.getAsDouble(),
                mStrafeSupplier.getAsDouble(),
                mDrive.getFieldRelativeHeading(),
                mKinematicLimits.get().maxDriveVelocity());
        var rotation = MathUtil.applyDeadband(mRotationSupplier.getAsDouble(), Constants.Controls.kDriverDeadband);
        var rotationRadiansPerSecond = Math.copySign(rotation * rotation, rotation)
                * mKinematicLimits.get().maxDriveVelocity()
                / Constants.Drive.kModuleRadius;

        var notePose = mRobotState.getDetectedNoteForPickup();

        if (notePose.isEmpty() || mRobotState.hasNote() || MathUtil.isNear(0, translationVelocities.getNorm(), 0.1)) {
            mDrive.setVelocity(new ChassisSpeeds(
                    translationVelocities.getX(), translationVelocities.getY(), rotationRadiansPerSecond));
            return;
        }

        var fieldRelativeTranslationToNote = notePose.isEmpty()
                ? GeometryUtil.kTranslationIdentity
                : notePose.get().pose().toPose2d().getTranslation().minus(endTranslation);
        var robotRelativeTranslationToNote = fieldRelativeTranslationToNote.rotateBy(
                currentPose.getRotation().unaryMinus());
        var yError = robotRelativeTranslationToNote.getY();

        if (!MathUtil.isNear(0, yError, kTranslationTolerance)) {

            var yPidOutput = mTranslationController.calculate(yError, 0);
            yPidOutput = MathUtil.clamp(yPidOutput, -kMaxTranslationPIDOutput, kMaxTranslationPIDOutput);
            translationVelocities =
                    new Translation2d(translationVelocities.getX(), translationVelocities.getY() - yPidOutput);
        }

        mDrive.setVelocity(new ChassisSpeeds(
                translationVelocities.getX(), translationVelocities.getY(), rotationRadiansPerSecond * 0.5));
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }
}

// make it decide which note to go for based off which direction the robot is driving, distance from robot to note
