package com.team1701.robot.subsystems.shooter;

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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private final MotorIO mUpperRollerMotorIO;
    private final MotorIO mLowerRollerMotorIO;

    private final MotorIO mRotationMotorIO;

    private final EncoderIO mAngleEncoderIO;

    private final MotorInputsAutoLogged mUpperShooterMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mLowerShooterMotorInputs = new MotorInputsAutoLogged();

    private final MotorInputsAutoLogged mRotationShooterMotorInputs = new MotorInputsAutoLogged();

    private final EncoderInputsAutoLogged mAngleEncoderInputs = new EncoderInputsAutoLogged();

    private Mechanism2d mShooterMechanism;
    private MechanismLigament2d mShooterLigament;
    private MechanismLigament2d mShooterPost;
    private MechanismRoot2d mShooterRoot;
    private Rotation2d mAngle;

    private final SlewRateLimiter mUpperRollerSlewRateLimiter = new SlewRateLimiter(Constants.Shooter.kRollerRampRate);
    private final SlewRateLimiter mLowerRollerSlewRateLimiter = new SlewRateLimiter(Constants.Shooter.kRollerRampRate);

    private Optional<Rotation2d> mRotationMotorOffset = Optional.empty();

    public Shooter(MotorIO upperMotor, MotorIO lowerMotor, MotorIO rotationMotor, EncoderIO angleEncoder) {
        mUpperRollerMotorIO = upperMotor;
        mLowerRollerMotorIO = lowerMotor;

        mRotationMotorIO = rotationMotor;

        mUpperRollerMotorIO.setBrakeMode(false);
        mLowerRollerMotorIO.setBrakeMode(false);

        mRotationMotorIO.setBrakeMode(true);

        setRollerPID();
        setRotationPID();

        mAngleEncoderIO = angleEncoder;

        createMechanism2d();
    }

    public static record ShooterSpeeds(double upperSpeed, double lowerSpeed) {
        public ShooterSpeeds(double radiansPerSecond) {
            this(radiansPerSecond, radiansPerSecond);
        }

        public boolean allMatch(ShooterSpeeds speeds, double tolerance) {
            return upperMatches(speeds.upperSpeed, tolerance) && lowerMatches(speeds.lowerSpeed, tolerance);
        }

        public boolean allMatch(double radiansPerSecond, double tolerance) {
            return upperMatches(radiansPerSecond, tolerance) && lowerMatches(radiansPerSecond, tolerance);
        }

        public boolean upperMatches(double radiansPerSecond, double tolerance) {
            return MathUtil.isNear(radiansPerSecond, upperSpeed, tolerance);
        }

        public boolean lowerMatches(double radiansPerSecond, double tolerance) {
            return MathUtil.isNear(radiansPerSecond, lowerSpeed, tolerance);
        }

        public double[] toArray() {
            return new double[] {upperSpeed, lowerSpeed};
        }
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
        mUpperRollerMotorIO.updateInputs(mUpperShooterMotorInputs);
        mLowerRollerMotorIO.updateInputs(mLowerShooterMotorInputs);
        mRotationMotorIO.updateInputs(mRotationShooterMotorInputs);
        mAngleEncoderIO.updateInputs(mAngleEncoderInputs);

        Logger.processInputs("Shooter/Motors/UpperRoller", mUpperShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/LowerRoller", mLowerShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/Rotation", mRotationShooterMotorInputs);
        Logger.processInputs("Shooter/Encoder", mAngleEncoderInputs);

        Logger.recordOutput(
                "Shooter/Motors/UpperRollerWattage",
                mUpperShooterMotorInputs.appliedVoltage * mUpperShooterMotorInputs.outputCurrent);
        Logger.recordOutput(
                "Shooter/Motors/LowerRollerWattage",
                mLowerShooterMotorInputs.appliedVoltage * mLowerShooterMotorInputs.outputCurrent);

        if (Constants.Shooter.kUpperRollerKs.hasChanged(hash)
                || Constants.Shooter.kUpperRollerKv.hasChanged(hash)
                || Constants.Shooter.kLowerRollerKs.hasChanged(hash)
                || Constants.Shooter.kLowerRollerKv.hasChanged(hash)
                || Constants.Shooter.kRollerKp.hasChanged(hash)
                || Constants.Shooter.kRollerKd.hasChanged(hash)) {
            setRollerPID();
        }

        if (Constants.Shooter.kRotationKp.hasChanged(hash) || Constants.Shooter.kRotationKd.hasChanged(hash)) {
            setRotationPID();
        }

        var angle = mAngleEncoderInputs
                .position
                .plus(Constants.Shooter.kShooterAngleEncoderOffset)
                .times(Constants.Shooter.kEncoderToShooterReduction);

        mAngle = GeometryUtil.angleModulus(angle, GeometryUtil.kRotationMinusHalfPi, GeometryUtil.kRotationThreeHalfPi);

        // Initialize the rotation motor offset once the angle encoder is sending values
        if (mRotationMotorOffset.isEmpty() && !Util.epsilonEquals(mAngleEncoderInputs.position.getRadians(), 0)) {
            mRotationMotorOffset = Optional.of(angle.div(Constants.Shooter.kAngleReduction));

            zeroShooter();
        }

        Logger.recordOutput("Shooter/calculatedAngle", angle);
        Logger.recordOutput("Shooter/calculatedAngleModulus", mAngle);
        Logger.recordOutput("Shooter/mech", mShooterMechanism);
    }

    private void setRollerPID() {
        mUpperRollerMotorIO.setFeedforward(
                Constants.Shooter.kUpperRollerKs.get(), Constants.Shooter.kUpperRollerKv.get(), 0);
        mLowerRollerMotorIO.setFeedforward(
                Constants.Shooter.kLowerRollerKs.get(), Constants.Shooter.kLowerRollerKv.get(), 0);
        mUpperRollerMotorIO.setPID(Constants.Shooter.kRollerKp.get(), 0, Constants.Shooter.kRollerKd.get());
        mLowerRollerMotorIO.setPID(Constants.Shooter.kRollerKp.get(), 0, Constants.Shooter.kRollerKd.get());
    }

    private void setRotationPID() {
        mRotationMotorIO.setPID(Constants.Shooter.kRotationKp.get(), 0, Constants.Shooter.kRotationKd.get());
    }

    public void zeroShooter() {
        mRotationMotorIO.setPosition(mAngle);
    }

    @AutoLogOutput
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(mRotationShooterMotorInputs.positionRadians);
    }

    public ShooterSpeeds getRollerSpeedsRadiansPerSecond() {
        return new ShooterSpeeds(
                mUpperShooterMotorInputs.velocityRadiansPerSecond, mLowerShooterMotorInputs.velocityRadiansPerSecond);
    }

    public void setRollerSpeeds(ShooterSpeeds shooterSpeeds) {
        setUpperRollerSpeed(shooterSpeeds.upperSpeed);
        setLowerRollerSpeed(shooterSpeeds.lowerSpeed);
    }

    public void setUnifiedSpeed(double radiansPerSecond) {
        setRollerSpeeds(new ShooterSpeeds(radiansPerSecond));
    }

    public void setUpperRollerSpeed(double radiansPerSecond) {
        var calculatedSlew = mUpperRollerSlewRateLimiter.calculate(radiansPerSecond);
        if (Util.epsilonEquals(radiansPerSecond, 0)) {
            stopUpperRoller();
        } else {
            mUpperRollerMotorIO.setVelocityControl(calculatedSlew);
            Logger.recordOutput("Shooter/Motors/UpperRoller/DemandRadiansPerSecond", calculatedSlew);
        }
    }

    public void setLowerRollerSpeed(double radiansPerSecond) {
        var calculatedSlew = mLowerRollerSlewRateLimiter.calculate(radiansPerSecond);
        if (Util.epsilonEquals(radiansPerSecond, 0)) {
            stopLowerRoller();
        } else {
            mLowerRollerMotorIO.setVelocityControl(calculatedSlew);
            Logger.recordOutput("Shooter/Motors/LowerRoller/DemandRadiansPerSecond", calculatedSlew);
        }
    }

    public void runUpperRollerCharacterization(double input) {
        mUpperRollerMotorIO.runCharacterization(input);
    }

    public void runLowerRollerCharacterization(double input) {
        mLowerRollerMotorIO.runCharacterization(input);
    }

    public void setRotationAngle(Rotation2d rotation) {
        if (mRotationMotorOffset.isEmpty()) {
            mRotationMotorIO.setPercentOutput(0.0);
            return;
        }
        mRotationMotorIO.setPositionControl(rotation);

        mShooterLigament.setAngle(rotation.getDegrees() - 90);
        Logger.recordOutput("Shooter/Motors/Rotation/Demand", rotation);
    }

    public void setShooterUp() {
        mRotationMotorIO.setPercentOutput(.1);
    }

    public void setShooterDown() {
        mRotationMotorIO.setPercentOutput(-.1);
    }

    public void stopRollers() {
        stopUpperRoller();
        stopLowerRoller();
    }

    public void stopUpperRoller() {
        mUpperRollerMotorIO.stopMotor();
        Logger.recordOutput("Shooter/Motors/UpperRoller/DemandRadiansPerSecond", 0);
    }

    public void stopLowerRoller() {
        mLowerRollerMotorIO.stopMotor();
        Logger.recordOutput("Shooter/Motors/LowerRoller/DemandRadiansPerSecond", 0);
    }

    public void stopRotation() {
        mRotationMotorIO.stopMotor();
    }

    public void setRotationPercentOutput(double percent) {
        mRotationMotorIO.setPercentOutput(percent);
    }
}
