package com.team1701.robot.subsystems.drive;

import java.util.Arrays;
import java.util.stream.Stream;

import com.team1701.lib.drivers.gyros.GyroIO;
import com.team1701.lib.drivers.gyros.GyroInputsAutoLogged;
import com.team1701.lib.swerve.SwerveSetpoint;
import com.team1701.lib.swerve.SwerveSetpointGenerator;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.SignalSamplingThread;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.SwerveModule.SwerveModuleIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private final PoseEstimator mPoseEstimator = PoseEstimator.getInstance();
    private final GyroInputsAutoLogged mGyroInputs = new GyroInputsAutoLogged();
    private final GyroIO mGyroIO;
    private final SwerveModule[] mModules;
    private final SwerveSetpointGenerator mSetpointGenerator = new SwerveSetpointGenerator(Constants.Drive.kKinematics);
    private final SignalSamplingThread mOdometryThread =
            new SignalSamplingThread("OdometryThread", 1 / Constants.Drive.kOdometryFrequency);

    private KinematicLimits mKinematicLimits = Constants.Drive.kFastKinematicLimits;
    private ChassisSpeeds mDesiredChassisSpeeds = new ChassisSpeeds();
    private Rotation2d[] mDesiredModuleOrientations;
    private SwerveSetpoint mPreviousSetpoint = new SwerveSetpoint(Constants.Drive.kNumModules);
    private SwerveModulePosition[] mMeasuredModulePositions;
    private Rotation2d mFieldRelativeHeading = GeometryUtil.kRotationIdentity;
    private Rotation2d mYawOffset = GeometryUtil.kRotationIdentity;
    private double mPreviousOdometryTimestamp = 0.0;
    private TimeLockedBoolean mWasMovingRecently = new TimeLockedBoolean(1.0, 0.0, false, false);
    private DriveControlState mDriveControlState = DriveControlState.VELOCITY_CONTROL;

    @AutoLogOutput(key = "Drive/MeasuredStates")
    private SwerveModuleState[] mMeasuredModuleStates;

    public Drive(GyroIO gyroIO, SwerveModuleIO[] moduleIOs) {
        if (moduleIOs.length != Constants.Drive.kNumModules) {
            throw new IllegalArgumentException("Module IOs must have length " + Constants.Drive.kNumModules);
        }

        mDesiredModuleOrientations = new Rotation2d[moduleIOs.length];
        Arrays.setAll(mDesiredModuleOrientations, i -> GeometryUtil.kRotationIdentity);

        mMeasuredModuleStates = new SwerveModuleState[moduleIOs.length];
        Arrays.setAll(mMeasuredModuleStates, i -> new SwerveModuleState());

        mMeasuredModulePositions = new SwerveModulePosition[moduleIOs.length];
        Arrays.setAll(mMeasuredModulePositions, i -> new SwerveModulePosition());

        gyroIO.enableYawSampling(mOdometryThread);
        mGyroIO = gyroIO;
        mModules = new SwerveModule[moduleIOs.length];
        for (var i = 0; i < mModules.length; i++) {
            var swerveModuleIO = moduleIOs[i];

            swerveModuleIO.driveMotorIO().enablePositionSampling(mOdometryThread);
            swerveModuleIO.steerMotorIO().enablePositionSampling(mOdometryThread);

            mModules[i] = new SwerveModule(i, moduleIOs[i]);
        }

        for (var module : mModules) {
            module.setSteerBrakeMode(false);
            module.setDriveBrakeMode(false);
        }

        updateInputs();
        zeroModules();
    }

    @Override
    public void periodic() {
        updateInputs();
        updateOdometry();
        updateDesiredStates();
    }

    private void updateInputs() {
        var odometryLock = mOdometryThread.getLock();
        odometryLock.lock();
        try {
            for (var module : mModules) {
                module.updateInputs();
            }

            mGyroIO.updateInputs(mGyroInputs);
        } finally {
            odometryLock.unlock();
        }

        Logger.processInputs("Drive/Gyro", mGyroInputs);

        for (var module : mModules) {
            module.periodic();
        }

        mMeasuredModuleStates = Stream.of(mModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
        mMeasuredModulePositions =
                Stream.of(mModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);

        mFieldRelativeHeading = mGyroInputs.yaw.minus(mYawOffset);

        var timestamp = Timer.getFPGATimestamp();
        for (var state : mMeasuredModuleStates) {
            mWasMovingRecently.update(!Util.inRange(state.speedMetersPerSecond, 0.02), timestamp);
        }
    }

    private void updateOdometry() {
        var modulePositionSamples =
                Stream.of(mModules).map(SwerveModule::getPositionSamples).toArray(SwerveModulePosition[][]::new);
        var yawSamples = mGyroInputs.yawSamples;

        var minSamples = Stream.of(modulePositionSamples)
                .mapToInt(samples -> samples.length)
                .min()
                .orElse(0);
        minSamples = Math.min(minSamples, yawSamples.length);

        var timestamp = Timer.getFPGATimestamp();
        var sampleDt = (timestamp - mPreviousOdometryTimestamp) / (minSamples + 1);
        var sampleTime = mPreviousOdometryTimestamp + sampleDt;
        for (var i = 0; i < minSamples; i++) {
            var modulePositions = new SwerveModulePosition[modulePositionSamples.length];
            for (var j = 0; j < modulePositionSamples.length; j++) {
                modulePositions[j] = modulePositionSamples[j][i];
            }

            mPoseEstimator.updateWithTime(sampleTime, yawSamples[i], modulePositions);
            sampleTime += sampleDt;
        }

        mPoseEstimator.update(mGyroInputs.yaw, mMeasuredModulePositions);

        mPreviousOdometryTimestamp = timestamp;
    }

    private void updateDesiredStates() {
        if (DriverStation.isDisabled()) {
            setModuleStatesToIdle();
            return;
        }

        if (mDriveControlState == DriveControlState.ORIENT_MODULES
                && Stream.of(mMeasuredModuleStates)
                        .allMatch(state -> MathUtil.isNear(
                                0.0, state.speedMetersPerSecond, Constants.Drive.kMinLockVelocityMetersPerSecond))) {
            var desiredModuleStates = new SwerveModuleState[mModules.length];
            for (int i = 0; i < mModules.length; ++i) {
                desiredModuleStates[i] = SwerveModuleState.optimize(
                        new SwerveModuleState(0.0, mDesiredModuleOrientations[i]), mModules[i].getAngle());
                mModules[i].setOrient(desiredModuleStates[i].angle);
            }

            mPreviousSetpoint = new SwerveSetpoint(mDesiredChassisSpeeds, desiredModuleStates);
            Logger.recordOutput("Drive/DesiredStates", desiredModuleStates);
            return;
        }

        var desiredChassisSpeedIsZero = Util.epsilonEquals(mDesiredChassisSpeeds.vxMetersPerSecond, 0.0)
                && Util.epsilonEquals(mDesiredChassisSpeeds.vyMetersPerSecond, 0.0)
                && Util.epsilonEquals(mDesiredChassisSpeeds.omegaRadiansPerSecond, 0.0);
        var wasMovingRecently = mWasMovingRecently.update(!desiredChassisSpeedIsZero, Timer.getFPGATimestamp());
        if (!wasMovingRecently) {
            setModuleStatesToIdle();
            return;
        }

        var desiredRelativePose = new Pose2d(
                mDesiredChassisSpeeds.vxMetersPerSecond * Constants.kLoopPeriodSeconds,
                mDesiredChassisSpeeds.vyMetersPerSecond * Constants.kLoopPeriodSeconds,
                Rotation2d.fromRadians(mDesiredChassisSpeeds.omegaRadiansPerSecond * Constants.kLoopPeriodSeconds));
        var twistToDesiredPose = GeometryUtil.kPoseIdentity.log(desiredRelativePose);
        var outputChassisSpeeds = new ChassisSpeeds(
                twistToDesiredPose.dx / Constants.kLoopPeriodSeconds,
                twistToDesiredPose.dy / Constants.kLoopPeriodSeconds,
                twistToDesiredPose.dtheta / Constants.kLoopPeriodSeconds);
        var desiredSetpoint = mSetpointGenerator.generateSetpoint(
                mKinematicLimits, mPreviousSetpoint, outputChassisSpeeds, Constants.kLoopPeriodSeconds);
        for (var i = 0; i < mModules.length; i++) {
            mModules[i].setState(desiredSetpoint.moduleStates[i]);
        }

        mPreviousSetpoint = desiredSetpoint;
        Logger.recordOutput("Drive/DesiredStates", desiredSetpoint.moduleStates);
    }

    private void setModuleStatesToIdle() {
        for (var module : mModules) {
            module.stop();
        }

        mPreviousSetpoint = new SwerveSetpoint(Constants.Drive.kKinematics, mMeasuredModuleStates);
        Logger.recordOutput("Drive/DesiredStates", new SwerveModuleState[] {});
    }

    public void setKinematicLimits(KinematicLimits kinematicLimits) {
        mKinematicLimits = kinematicLimits;
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        mDesiredChassisSpeeds = chassisSpeeds;
        mDriveControlState = DriveControlState.VELOCITY_CONTROL;
    }

    public ChassisSpeeds getVelocity() {
        return Constants.Drive.kKinematics.toChassisSpeeds(mMeasuredModuleStates);
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getVelocity(), mFieldRelativeHeading);
    }

    @AutoLogOutput
    public double getSpeedMetersPerSecond() {
        var chassisSpeeds = Constants.Drive.kKinematics.toChassisSpeeds(mMeasuredModuleStates);
        return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    @AutoLogOutput()
    public Rotation2d getFieldRelativeHeading() {
        return mFieldRelativeHeading;
    }

    public void engageSwerveLock() {
        orientModules(new Rotation2d[] {
            Rotation2d.fromDegrees(45),
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(45)
        });
    }

    public void orientModules(Rotation2d orientation) {
        orientModules(Stream.generate(() -> orientation).limit(mModules.length).toArray(Rotation2d[]::new));
    }

    public void orientModules(Rotation2d[] orientations) {
        mDriveControlState = DriveControlState.ORIENT_MODULES;
        mDesiredChassisSpeeds = new ChassisSpeeds();
        mDesiredModuleOrientations = orientations;
    }

    public void zeroGyroscope() {
        zeroGyroscope(GeometryUtil.kRotationIdentity);
    }

    public void zeroGyroscope(Rotation2d rotation) {
        mYawOffset = mGyroInputs.yaw.minus(rotation);
        mFieldRelativeHeading = rotation;
    }

    public void zeroModules() {
        for (var module : mModules) {
            module.zeroSteeringMotor();
        }

        mMeasuredModuleStates = Stream.of(mModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
        mMeasuredModulePositions =
                Stream.of(mModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);

        mPoseEstimator.resetPosition(mGyroInputs.yaw, mMeasuredModulePositions, mPoseEstimator.getPose2d());
    }

    public void stop() {
        setVelocity(new ChassisSpeeds());
    }

    public enum DriveControlState {
        VELOCITY_CONTROL,
        ORIENT_MODULES,
    }
}
