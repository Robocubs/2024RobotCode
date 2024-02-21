package com.team1701.robot.subsystems.shooter;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final MotorIO mRightUpperShooterMotorIO;
    private final MotorIO mRightLowerShooterMotorIO;
    private final MotorIO mLeftUpperShooterMotorIO;
    private final MotorIO mLeftLowerShooterMotorIO;

    private final MotorIO mRotationShooterMotorIO;

    private final EncoderIO mAngleEncoderIO;

    private final MotorInputsAutoLogged mRightUpperShooterMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mRightLowerShooterMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mLeftUpperShooterMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mLeftLowerShooterMotorInputs = new MotorInputsAutoLogged();

    private final MotorInputsAutoLogged mRotationShooterMotorInputs = new MotorInputsAutoLogged();

    private final EncoderInputsAutoLogged mAngleEncoderInputs = new EncoderInputsAutoLogged();

    private Mechanism2d mShooterMechanism;
    private MechanismLigament2d mShooterLigament;
    private MechanismLigament2d mShooterPost;
    private MechanismRoot2d mShooterRoot;

    private Optional<Rotation2d> mRotationMotorOffset = Optional.empty();

    public Shooter(
            MotorIO rightUpperMotor,
            MotorIO rightLowerMotor,
            MotorIO leftUpperMotor,
            MotorIO leftLowerMotor,
            MotorIO rotationMotor,
            EncoderIO angleEncoder) {
        mRightUpperShooterMotorIO = rightUpperMotor;
        mRightLowerShooterMotorIO = rightLowerMotor;
        mLeftUpperShooterMotorIO = leftUpperMotor;
        mLeftLowerShooterMotorIO = leftLowerMotor;

        mRotationShooterMotorIO = rotationMotor;

        mRightUpperShooterMotorIO.setBrakeMode(false);
        mRightLowerShooterMotorIO.setBrakeMode(false);
        mLeftUpperShooterMotorIO.setBrakeMode(false);
        mLeftLowerShooterMotorIO.setBrakeMode(false);

        mRotationShooterMotorIO.setBrakeMode(true);

        setRollerPID(
                Constants.Shooter.kRollerKff.get(),
                Constants.Shooter.kRollerKp.get(),
                0,
                Constants.Shooter.kRollerKd.get());

        setRotationPID(
                Constants.Shooter.kRotationKff.get(),
                Constants.Shooter.kRotationKp.get(),
                0,
                Constants.Shooter.kRotationKd.get());

        mAngleEncoderIO = angleEncoder;

        createMechanism2d();
    }

    public static MotorIOSim createRollerMotorSim(DCMotor shooterMotor) {
        return new MotorIOSim(shooterMotor, Constants.Shooter.kRollerReduction, 0.0023, Constants.kLoopPeriodSeconds);
    }

    public static MotorIOSim createRotationMotorSim(DCMotor shooterMotor) {
        return new MotorIOSim(shooterMotor, Constants.Shooter.kAngleReduction, 0.14, Constants.kLoopPeriodSeconds);
    }

    public static EncoderIOSim createEncoderSim(MotorIOSim rotationMotor) {
        var randomStartingAngle = Rotation2d.fromRotations(0.25);
        var startingValue = randomStartingAngle.plus(Constants.Shooter.kShooterAngleEncoderOffset);
        var offset = startingValue.minus(rotationMotor.getPosition());
        return new EncoderIOSim(() -> rotationMotor.getPosition().plus(offset));
    }

    @AutoLogOutput
    private void createMechanism2d() {
        mShooterMechanism = new Mechanism2d(10, 10);
        mShooterRoot = mShooterMechanism.getRoot("root", 5 - Units.inchesToMeters(3), 0);
        mShooterPost = mShooterRoot.append(new MechanismLigament2d(
                "post", Constants.Shooter.kShooterAxisHeight, 90, 5, new Color8Bit(156, 32, 49)));
        mShooterLigament = mShooterPost.append(new MechanismLigament2d(
                "arm",
                Units.inchesToMeters(12.5) /*Constants.Robot.kShooterHingeToShooterExit.getY()*/,
                getAngle().getDegrees() - 90,
                10,
                new Color8Bit(156, 32, 49)));
    }

    @Override
    public void periodic() {
        var hash = hashCode();
        mRightUpperShooterMotorIO.updateInputs(mRightUpperShooterMotorInputs);
        mRightLowerShooterMotorIO.updateInputs(mRightLowerShooterMotorInputs);
        mLeftUpperShooterMotorIO.updateInputs(mLeftUpperShooterMotorInputs);
        mLeftLowerShooterMotorIO.updateInputs(mLeftLowerShooterMotorInputs);

        mRotationShooterMotorIO.updateInputs(mRotationShooterMotorInputs);

        mAngleEncoderIO.updateInputs(mAngleEncoderInputs);

        Logger.processInputs("Shooter/Motors/RightUpperRoller", mRightUpperShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/RightLowerRoller", mRightLowerShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/LeftUpperRoller", mLeftUpperShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/LeftLowerRoller", mLeftLowerShooterMotorInputs);

        Logger.processInputs("Shooter/Motors/Rotation", mRotationShooterMotorInputs);

        Logger.processInputs("Shooter/Encoder", mRotationShooterMotorInputs);

        if (Constants.Shooter.kRollerKff.hasChanged(hash)
                || Constants.Shooter.kRollerKp.hasChanged(hash)
                || Constants.Shooter.kRollerKd.hasChanged(hash)) {
            setRollerPID(
                    Constants.Shooter.kRollerKff.get(),
                    Constants.Shooter.kRollerKp.get(),
                    0,
                    Constants.Shooter.kRollerKd.get());
        }

        if (Constants.Shooter.kRotationKff.hasChanged(hash)
                || Constants.Shooter.kRotationKp.hasChanged(hash)
                || Constants.Shooter.kRotationKd.hasChanged(hash)) {
            setRotationPID(
                    Constants.Shooter.kRotationKff.get(),
                    Constants.Shooter.kRotationKp.get(),
                    0,
                    Constants.Shooter.kRotationKd.get());
        }

        // Initialize the rotation motor offset once the angle encoder is sending values
        if (mRotationMotorOffset.isEmpty() && !Util.epsilonEquals(mAngleEncoderInputs.position.getRadians(), 0)) {
            mRotationMotorOffset =
                    Optional.of(mAngleEncoderInputs.position.minus(Constants.Shooter.kShooterAngleEncoderOffset));

            zeroShooterRotation();
        }

        Logger.recordOutput("Shooter/mech", mShooterMechanism);
    }

    private void setRollerPID(double ff, double p, double i, double d) {
        mRightUpperShooterMotorIO.setPID(ff, p, i, d);
        mRightLowerShooterMotorIO.setPID(ff, p, i, d);
        mLeftUpperShooterMotorIO.setPID(ff, p, i, d);
        mLeftLowerShooterMotorIO.setPID(ff, p, i, d);
    }

    private void setRotationPID(double ff, double p, double i, double d) {
        mRotationShooterMotorIO.setPID(ff, p, i, d);
    }

    public void zeroShooterRotation() {
        mRotationShooterMotorIO.setPosition(mRotationMotorOffset
                .get()
                .minus(Constants.Shooter.kShooterAngleEncoderOffset)
                .times(Constants.Shooter.kEncoderToShooterReduction));
    }

    @AutoLogOutput
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(mRotationShooterMotorInputs.positionRadians);
    }

    public double[] getRollerSpeedsRadiansPerSecond() {
        // left motor speeds inverted for comparison in Shoot commands
        return new double[] {
            mRightUpperShooterMotorInputs.velocityRadiansPerSecond,
            mRightLowerShooterMotorInputs.velocityRadiansPerSecond,
            mLeftUpperShooterMotorInputs.velocityRadiansPerSecond,
            mLeftLowerShooterMotorInputs.velocityRadiansPerSecond
        };
    }

    public double[] getRightRollerSpeedsRadiansPerSecond() {
        return new double[] {
            mRightUpperShooterMotorInputs.velocityRadiansPerSecond,
            mRightLowerShooterMotorInputs.velocityRadiansPerSecond
        };
    }

    public double[] getLeftRollerSpeedsRadiansPerSecond() {
        return new double[] {
            mLeftUpperShooterMotorInputs.velocityRadiansPerSecond, mLeftLowerShooterMotorInputs.velocityRadiansPerSecond
        };
    }

    public void setUnifiedRollerSpeed(double radiansPerSecond) {
        Logger.recordOutput("Shooter/Motors/Rollers/UpperRightDemandRadiansPerSecond", radiansPerSecond);
        Logger.recordOutput("Shooter/Motors/Rollers/LowerRightDemandRadiansPerSecond", radiansPerSecond);
        Logger.recordOutput("Shooter/Motors/Rollers/UpperLeftDemandRadiansPerSecond", radiansPerSecond);
        Logger.recordOutput("Shooter/Motors/Rollers/LowerLeftDemandRadiansPerSecond", radiansPerSecond);
        mRightUpperShooterMotorIO.setVelocityControl(radiansPerSecond);
        mRightLowerShooterMotorIO.setVelocityControl(radiansPerSecond);
        mLeftUpperShooterMotorIO.setVelocityControl(radiansPerSecond);
        mLeftLowerShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setRightRollerSpeeds(double radiansPerSecond) {
        setUpperRightRollerSpeed(radiansPerSecond);
        setLowerRightRollerSpeed(radiansPerSecond);
    }

    public void setLeftRollerSpeeds(double radiansPerSecond) {
        setUpperLeftRollerSpeed(radiansPerSecond);
        setLowerLeftRollerSpeed(radiansPerSecond);
    }

    public void setUpperRightRollerSpeed(double radiansPerSecond) {
        Logger.recordOutput("Shooter/Motors/Rollers/UpperRightDemandRadiansPerSecond", radiansPerSecond);
        mRightUpperShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setLowerRightRollerSpeed(double radiansPerSecond) {
        Logger.recordOutput("Shooter/Motors/Rollers/LowerRightDemandRadiansPerSecond", radiansPerSecond);
        mRightLowerShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setUpperLeftRollerSpeed(double radiansPerSecond) {
        Logger.recordOutput("Shooter/Motors/Rollers/UpperLeftDemandRadiansPerSecond", radiansPerSecond);
        mLeftUpperShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setLowerLeftRollerSpeed(double radiansPerSecond) {
        Logger.recordOutput("Shooter/Motors/Rollers/LowerLeftDemandRadiansPerSecond", radiansPerSecond);
        mLeftLowerShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setRotationAngle(Rotation2d rotation) {
        if (mRotationMotorOffset.isEmpty()) {
            mRotationShooterMotorIO.setPercentOutput(0.0);
            return;
        }

        mRotationShooterMotorIO.setSmoothPositionControl(
                rotation,
                Constants.Shooter.kMaxRotationVelocityRadiansPerSecond.get(),
                Constants.Shooter.kMaxRotationAccelerationRadiansPerSecondSquared.get());

        mShooterLigament.setAngle(rotation.getDegrees() - 90);
        Logger.recordOutput("Shooter/Motors/Rotation/Demand", rotation);
    }

    public void stopRollers() {
        mRightUpperShooterMotorIO.setPercentOutput(0);
        mRightLowerShooterMotorIO.setPercentOutput(0);
        mLeftUpperShooterMotorIO.setPercentOutput(0);
        mLeftLowerShooterMotorIO.setPercentOutput(0);
    }

    public void stopRotation() {
        mRotationShooterMotorIO.setPercentOutput(0);
    }
}
