package com.team1701.robot.subsystems.arm;

import java.util.Optional;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOSim;
import com.team1701.lib.drivers.encoders.EncoderInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
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
    private final MotorIO mLeftWinchIO;
    private final MotorIO mRightWinchIO;

    private final EncoderIO mAngleEncoderIO;

    private Mechanism2d mLeftArmMechanism;
    private Mechanism2d mRightArmMechanism;

    private MechanismRoot2d mLeftArmRoot;
    private MechanismRoot2d mRightArmRoot;

    private MechanismLigament2d mLeftArmLigament;
    private MechanismLigament2d mRightArmLigament;

    private final MotorInputsAutoLogged mRotationMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mLeftWinchMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mRightWinchMotorInputs = new MotorInputsAutoLogged();

    private final EncoderInputsAutoLogged mAngleEncoderInputs = new EncoderInputsAutoLogged();

    private Optional<Rotation2d> mRotationMotorOffset = Optional.empty();

    public Arm(MotorIO rotationMotor, MotorIO leftWinch, MotorIO rightWinch, EncoderIO angleEncoder) {
        mRotationMotorIO = rotationMotor;
        mLeftWinchIO = leftWinch;
        mRightWinchIO = rightWinch;
        mAngleEncoderIO = angleEncoder;

        mRotationMotorIO.setBrakeMode(true);
        mLeftWinchIO.setBrakeMode(true);
        mRightWinchIO.setBrakeMode(true);

        mRotationMotorIO.setPID(
                Constants.Arm.kArmRotationKff.get(),
                Constants.Arm.kArmRotationKp.get(),
                0,
                Constants.Arm.kArmRotationKd.get());

        setWinchPID(Constants.Arm.kWinchKff.get(), Constants.Arm.kWinchKp.get(), 0, Constants.Arm.kWinchKd.get());

        createMechanism2d();
    }

    private void setWinchPID(double ff, double p, double i, double d) {
        mLeftWinchIO.setPID(ff, p, i, d);
        mRightWinchIO.setPID(ff, p, i, d);
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

        mLeftArmLigament = mLeftArmRoot.append(
                new MechanismLigament2d("leftArm", length, getAngle().getDegrees()));
        mRightArmLigament = mRightArmRoot.append(
                new MechanismLigament2d("rightArm", length, getAngle().getDegrees()));
    }

    @AutoLogOutput
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(mRotationMotorInputs.positionRadians);
    }

    public static MotorIOSim createRotationMotorIOSim(DCMotor shooterMotor) {
        return new MotorIOSim(shooterMotor, Constants.Arm.kRotationReduction, 0.14, Constants.kLoopPeriodSeconds);
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
        mLeftWinchIO.updateInputs(mLeftWinchMotorInputs);
        mRightWinchIO.updateInputs(mRightWinchMotorInputs);

        mAngleEncoderIO.updateInputs(mAngleEncoderInputs);

        Logger.processInputs("Arm/Motors/Rotation", mRotationMotorInputs);
        Logger.processInputs("Arm/Motors/LeftWinch", mLeftWinchMotorInputs);
        Logger.processInputs("Arm/Motors/RightWinch", mRightWinchMotorInputs);

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

        if (Constants.Arm.kWinchKff.hasChanged(hash)
                || Constants.Arm.kWinchKp.hasChanged(hash)
                || Constants.Arm.kWinchKd.hasChanged(hash)) {
            setWinchPID(Constants.Arm.kWinchKff.get(), Constants.Arm.kWinchKp.get(), 0, Constants.Arm.kWinchKd.get());
        }

        if (mRotationMotorOffset.isEmpty() && !Util.epsilonEquals(mAngleEncoderInputs.position.getRadians(), 0)) {
            mRotationMotorOffset =
                    Optional.of(mAngleEncoderInputs.position.minus(Constants.Arm.kArmAngleEncoderOffset));

            zeroShooterRotation();
        }

        Logger.recordOutput("Arm/Mechanism/Left", mLeftArmMechanism);
        Logger.recordOutput("Arm/Mechanism/Right", mRightArmMechanism);
    }

    public void zeroShooterRotation() {
        mRotationMotorIO.setPosition(mRotationMotorOffset
                .get()
                .minus(Constants.Arm.kArmAngleEncoderOffset)
                .times(Constants.Arm.kEncoderToArmReduction));
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

    public void retractWinch(double distanceMeters) {
        var rotations = distanceMeters / Constants.Arm.kWinchCircumference;
        mLeftWinchIO.setSmoothPositionControl(
                Rotation2d.fromRotations(rotations),
                Constants.Arm.kMaxRotationVelocityRadiansPerSecond.get(),
                Constants.Arm.kMaxRotationAccelerationRadiansPerSecondSquared.get());
        mRightWinchIO.setSmoothPositionControl(
                Rotation2d.fromRotations(rotations),
                Constants.Arm.kMaxRotationVelocityRadiansPerSecond.get(),
                Constants.Arm.kMaxRotationAccelerationRadiansPerSecondSquared.get());
    }
}
