package com.team1701.robot.subsystems.drive;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOSim;
import com.team1701.lib.drivers.encoders.EncoderInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.tuning.LoggedTunableValue;
import com.team1701.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final int mIndex;
    private final MotorIO mDriveMotorIO;
    private final MotorIO mSteerMotorIO;
    private final EncoderIO mSteerEncoderIO;
    private final MotorInputsAutoLogged mDriveMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mSteerMotorInputs = new MotorInputsAutoLogged();
    private final EncoderInputsAutoLogged mSteerEncoderInputs = new EncoderInputsAutoLogged();
    private final Rotation2d mSteerEncoderOffset;

    private Rotation2d mMeasuredAngle = GeometryUtil.kRotationIdentity;
    private Rotation2d mAngleOffset = GeometryUtil.kRotationIdentity;
    private boolean mAngleOffsetNotInitialized = true;

    public static record SwerveModuleIO(
            MotorIO driveMotorIO, MotorIO steerMotorIO, EncoderIO steerEncoderIO, Rotation2d steerEncoderOffset) {
        public static SwerveModuleIO createSim(DCMotor driveMotor, DCMotor steerMotor, Rotation2d encoderOffset) {
            var driveMotorIO =
                    new MotorIOSim(driveMotor, Constants.Drive.kDriveReduction, 0.025, Constants.kLoopPeriodSeconds);
            var steerMotorIO =
                    new MotorIOSim(steerMotor, Constants.Drive.kSteerReduction, 0.004, Constants.kLoopPeriodSeconds);
            steerMotorIO.enableContinuousInput(0, 2 * Math.PI);
            var encoderIO = new EncoderIOSim(() -> steerMotorIO
                    .getPosition()
                    .times(Constants.Drive.kSteerReduction)
                    .plus(encoderOffset));
            return new SwerveModuleIO(driveMotorIO, steerMotorIO, encoderIO, encoderOffset);
        }
    }

    public SwerveModule(int index, SwerveModuleIO moduleIO) {
        mIndex = index;
        mDriveMotorIO = moduleIO.driveMotorIO;
        mSteerMotorIO = moduleIO.steerMotorIO;
        mSteerEncoderIO = moduleIO.steerEncoderIO;
        mSteerEncoderOffset = moduleIO.steerEncoderOffset;
        mDriveMotorIO.setFeedforward(
                Constants.Drive.kDriveKs.get(), Constants.Drive.kDriveKv.get(), Constants.Drive.kDriveKa.get());
        mDriveMotorIO.setPID(Constants.Drive.kDriveKp.get(), 0, Constants.Drive.kDriveKd.get());
        mSteerMotorIO.setPID(Constants.Drive.kSteerKp.get(), 0, Constants.Drive.kSteerKd.get());
    }

    // Separated from periodic to support thread locking of odometry inputs
    public void updateInputs() {
        mDriveMotorIO.updateInputs(mDriveMotorInputs);
        mSteerMotorIO.updateInputs(mSteerMotorInputs);
        mSteerEncoderIO.updateInputs(mSteerEncoderInputs);
    }

    public void periodic() {
        Logger.processInputs("Drive/Module/" + mIndex + "/Drive", mDriveMotorInputs);
        Logger.processInputs("Drive/Module/" + mIndex + "/Steer", mSteerMotorInputs);
        Logger.processInputs("Drive/Module/" + mIndex + "/AbsoluteEncoder", mSteerEncoderInputs);

        // Zero the steering motor position on the first loop after receiving an absolute encoder position
        if (mAngleOffsetNotInitialized && !mSteerEncoderInputs.position.equals(GeometryUtil.kRotationIdentity)) {
            zeroSteeringMotor();
            mAngleOffsetNotInitialized = false;
        }

        mMeasuredAngle = toModuleAngle(Rotation2d.fromRadians(mSteerMotorInputs.positionRadians));

        LoggedTunableValue.ifChanged(
                hashCode(),
                () -> {
                    mDriveMotorIO.setFeedforward(
                            Constants.Drive.kDriveKs.get(),
                            Constants.Drive.kDriveKv.get(),
                            Constants.Drive.kDriveKa.get());
                    mDriveMotorIO.setPID(Constants.Drive.kDriveKp.get(), 0, Constants.Drive.kDriveKd.get());

                    mSteerMotorIO.setPID(Constants.Drive.kSteerKp.get(), 0, Constants.Drive.kSteerKd.get());
                },
                Constants.Drive.kDriveKs,
                Constants.Drive.kDriveKv,
                Constants.Drive.kDriveKa,
                Constants.Drive.kDriveKp,
                Constants.Drive.kDriveKd,
                Constants.Drive.kSteerKp,
                Constants.Drive.kSteerKd);
    }

    public Rotation2d getAngle() {
        return mMeasuredAngle;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                mDriveMotorInputs.positionRadians * Constants.Drive.kWheelRadiusMeters, mMeasuredAngle);
    }

    public SwerveModulePosition[] getPositionSamples() {
        var states = new SwerveModulePosition
                [Math.min(
                        mDriveMotorInputs.positionRadiansSamples.length,
                        mSteerMotorInputs.positionRadiansSamples.length)];

        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModulePosition(
                    mDriveMotorInputs.positionRadiansSamples[i] * Constants.Drive.kWheelRadiusMeters,
                    toModuleAngle(Rotation2d.fromRadians(mSteerMotorInputs.positionRadiansSamples[i])));
        }

        return states;
    }

    public double getSpeedRadiansPerSecond() {
        return mDriveMotorInputs.velocityRadiansPerSecond;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                mDriveMotorInputs.velocityRadiansPerSecond * Constants.Drive.kWheelRadiusMeters, mMeasuredAngle);
    }

    public void setState(SwerveModuleState state) {
        mDriveMotorIO.setVelocityControl(state.speedMetersPerSecond / Constants.Drive.kWheelRadiusMeters);
        mSteerMotorIO.setPositionControl(state.angle.minus(mAngleOffset));
    }

    public void setOrient(Rotation2d steerAngle) {
        mDriveMotorIO.setPercentOutput(0);
        mSteerMotorIO.setPositionControl(steerAngle.minus(mAngleOffset));
    }

    public void runCharacterization(double input) {
        mDriveMotorIO.runCharacterization(input);
        mSteerMotorIO.setPositionControl(mAngleOffset.unaryMinus());
    }

    public void setDriveBrakeMode(boolean enable) {
        mDriveMotorIO.setBrakeMode(enable);
    }

    public void setSteerBrakeMode(boolean enable) {
        mSteerMotorIO.setBrakeMode(enable);
    }

    public void zeroSteeringMotor() {
        mAngleOffset = mSteerEncoderInputs
                .position
                .plus(mSteerEncoderOffset)
                .minus(Rotation2d.fromRadians(mSteerMotorInputs.positionRadians));
        mMeasuredAngle = mSteerEncoderInputs.position.plus(mSteerEncoderOffset);
    }

    public void stop() {
        mDriveMotorIO.setPercentOutput(0.0);
        mSteerMotorIO.setPercentOutput(0.0);
    }

    private Rotation2d toModuleAngle(Rotation2d steerMotorPosition) {
        return GeometryUtil.angleModulus(steerMotorPosition.plus(mAngleOffset));
    }
}
