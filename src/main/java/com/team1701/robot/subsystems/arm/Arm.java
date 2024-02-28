package com.team1701.robot.subsystems.arm;

import java.util.Optional;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOSim;
import com.team1701.lib.drivers.encoders.EncoderInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final MotorIO mRotationMotorIO;

    private final EncoderIO mAngleEncoderIO;

    private Rotation2d mAngle;

    @AutoLogOutput(key = "Arm/LeftArmLigament")
    private Mechanism2d mLeftArmMechanism;

    @AutoLogOutput(key = "Arm/RightArmMechanism")
    private Mechanism2d mRightArmMechanism;

    private MechanismRoot2d mLeftArmRoot;
    private MechanismRoot2d mRightArmRoot;

    private final MotorInputsAutoLogged mRotationMotorInputs = new MotorInputsAutoLogged();

    private final EncoderInputsAutoLogged mAngleEncoderInputs = new EncoderInputsAutoLogged();

    private Optional<Rotation2d> mRotationMotorOffset = Optional.empty();

    public Arm(MotorIO rotationMotor, EncoderIO angleEncoder) {
        mRotationMotorIO = rotationMotor;

        mAngleEncoderIO = angleEncoder;

        mRotationMotorIO.setBrakeMode(true);

        mRotationMotorIO.setPID(
                Constants.Arm.kArmRotationKff.get(),
                Constants.Arm.kArmRotationKp.get(),
                0,
                Constants.Arm.kArmRotationKd.get());

        createMechanism2d();
    }

    @AutoLogOutput
    private void createMechanism2d() {
        var heightFromCenter = 0;
        var distanceFromCenter = 0;
        var length = 0;

        mLeftArmMechanism = new Mechanism2d(10, 10);
        mRightArmMechanism = new Mechanism2d(10, 10);

        mLeftArmRoot = mLeftArmMechanism.getRoot("leftRoot", 5 - distanceFromCenter, heightFromCenter);
        mRightArmRoot = mRightArmMechanism.getRoot("rightRoot", 5 - distanceFromCenter, heightFromCenter);

        mLeftArmRoot.append(
                new MechanismLigament2d("leftArm", length, getAngle().getDegrees()));
        mRightArmRoot.append(
                new MechanismLigament2d("rightArm", length, getAngle().getDegrees()));
    }

    @AutoLogOutput
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(mRotationMotorInputs.positionRadians);
    }

    public static MotorIOSim createRotationMotorIOSim(DCMotor rotationMotor) {
        return new MotorIOSim(rotationMotor, Constants.Arm.kRotationReduction, 0.14, Constants.kLoopPeriodSeconds);
    }

    public static EncoderIOSim createEncoderSim(MotorIOSim rotationMotor) {
        var randomStartingAngle = Rotation2d.fromRotations(0.25);
        var startingValue = randomStartingAngle.plus(Rotation2d.fromRotations(Math.random()));
        var offset = startingValue.minus(rotationMotor.getPosition());
        return new EncoderIOSim(() -> rotationMotor.getPosition().plus(offset));
    }

    @Override
    public void periodic() {
        var hash = hashCode();

        mRotationMotorIO.updateInputs(mRotationMotorInputs);

        mAngleEncoderIO.updateInputs(mAngleEncoderInputs);

        Logger.processInputs("Arm/Motors/Rotation", mRotationMotorInputs);

        Logger.processInputs("Arm/Encoder", mAngleEncoderInputs);

        if (Constants.Arm.kArmRotationKff.hasChanged(hash)
                || Constants.Arm.kArmRotationKp.hasChanged(hash)
                || Constants.Arm.kArmRotationKd.hasChanged(hash)) {
            mRotationMotorIO.setPID(
                    Constants.Arm.kArmRotationKff.get(),
                    Constants.Arm.kArmRotationKp.get(),
                    0,
                    Constants.Arm.kArmRotationKd.get());
        }

        var angle = mAngleEncoderInputs.position.plus(Constants.Arm.kArmAngleEncoderOffset);

        mAngle = GeometryUtil.angleModulus(angle, GeometryUtil.kRotationMinusHalfPi, GeometryUtil.kRotationThreeHalfPi);

        if (mRotationMotorOffset.isEmpty() && !Util.epsilonEquals(mAngleEncoderInputs.position.getRadians(), 0)) {
            mRotationMotorOffset = Optional.of(angle.div(Constants.Arm.kAngleReduction));

            zeroArmRotation();
        }

        Logger.recordOutput("Arm/calculatedAngle", angle);
        Logger.recordOutput("Arm/calculatedAngleModulus", mAngle);

        Logger.recordOutput("Arm/Mechanism/Left", mLeftArmMechanism);
        Logger.recordOutput("Arm/Mechanism/Right", mRightArmMechanism);
    }

    public void zeroArmRotation() {
        mRotationMotorIO.setPosition(mAngle);
    }

    public void setRotationAngle(Rotation2d rotation) {
        if (mRotationMotorOffset.isEmpty()) {
            mRotationMotorIO.setPercentOutput(0);
        } else {
            mRotationMotorIO.setSmoothPositionControl(
                    rotation,
                    Constants.Arm.kMaxRotationVelocityRadiansPerSecond.get(),
                    Constants.Arm.kMaxRotationAccelerationRadiansPerSecondSquared.get());

            // mLeftArmLigament.setAngle(rotation.getDegrees());
            Logger.recordOutput("Arm/Rotation/Demand", rotation);
        }
    }

    public void rotateToAmpPosition() {
        setRotationAngle(Rotation2d.fromDegrees(Constants.Arm.kArmAmpRotationDegrees.get()));
    }

    public void rotateHome() {
        setRotationAngle(Rotation2d.fromDegrees(0));
    }

    public void stop() {
        mRotationMotorIO.setPercentOutput(0);
    }

    public enum ArmPosition {
        // TODO: update values
        HOME(0),
        LOW(45),
        LOW_CLIMB(75),
        HIGH_CLIMB(90),
        AMP(Constants.Arm.kArmAmpRotationDegrees.get());

        public Rotation2d armRotation;

        private ArmPosition(double degrees) {
            armRotation = Rotation2d.fromDegrees(degrees);
        }
    }
}
