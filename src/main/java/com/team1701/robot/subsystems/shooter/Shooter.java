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
    private final MotorIO mRightUpperRollerMotorIO;
    private final MotorIO mRightLowerRollerMotorIO;
    private final MotorIO mLeftUpperRollerMotorIO;
    private final MotorIO mLeftLowerRollerMotorIO;

    private final MotorIO mRotationMotorIO;

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
    private Rotation2d mAngle;

    private final SlewRateLimiter mLeftUpperRollerSlewRateLimiter =
            new SlewRateLimiter(Constants.Shooter.kRollerRampRate);
    private final SlewRateLimiter mRightUpperRollerSlewRateLimiter =
            new SlewRateLimiter(Constants.Shooter.kRollerRampRate);
    private final SlewRateLimiter mLeftLowerRollerSlewRateLimiter =
            new SlewRateLimiter(Constants.Shooter.kRollerRampRate);
    private final SlewRateLimiter mRightLowerRollerSlewRateLimiter =
            new SlewRateLimiter(Constants.Shooter.kRollerRampRate);

    private Optional<Rotation2d> mRotationMotorOffset = Optional.empty();

    public Shooter(
            MotorIO rightUpperMotor,
            MotorIO rightLowerMotor,
            MotorIO leftUpperMotor,
            MotorIO leftLowerMotor,
            MotorIO rotationMotor,
            EncoderIO angleEncoder) {
        mRightUpperRollerMotorIO = rightUpperMotor;
        mRightLowerRollerMotorIO = rightLowerMotor;
        mLeftUpperRollerMotorIO = leftUpperMotor;
        mLeftLowerRollerMotorIO = leftLowerMotor;

        mRotationMotorIO = rotationMotor;

        mRightUpperRollerMotorIO.setBrakeMode(false);
        mRightLowerRollerMotorIO.setBrakeMode(false);
        mLeftUpperRollerMotorIO.setBrakeMode(false);
        mLeftLowerRollerMotorIO.setBrakeMode(false);

        mRotationMotorIO.setBrakeMode(true);

        setRollerPID(
                Constants.Shooter.kRollerKff.get(),
                Constants.Shooter.kRollerKp.get(),
                0,
                Constants.Shooter.kRollerKd.get());

        setRotationPID(0, Constants.Shooter.kRotationKp.get(), 0, Constants.Shooter.kRotationKd.get());

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
        mRightUpperRollerMotorIO.updateInputs(mRightUpperShooterMotorInputs);
        mRightLowerRollerMotorIO.updateInputs(mRightLowerShooterMotorInputs);
        mLeftUpperRollerMotorIO.updateInputs(mLeftUpperShooterMotorInputs);
        mLeftLowerRollerMotorIO.updateInputs(mLeftLowerShooterMotorInputs);

        mRotationMotorIO.updateInputs(mRotationShooterMotorInputs);

        mAngleEncoderIO.updateInputs(mAngleEncoderInputs);

        Logger.processInputs("Shooter/Motors/RightUpperRoller", mRightUpperShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/RightLowerRoller", mRightLowerShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/LeftUpperRoller", mLeftUpperShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/LeftLowerRoller", mLeftLowerShooterMotorInputs);

        Logger.processInputs("Shooter/Motors/Rotation", mRotationShooterMotorInputs);

        Logger.processInputs("Shooter/Encoder", mAngleEncoderInputs);

        if (Constants.Shooter.kRollerKff.hasChanged(hash)
                || Constants.Shooter.kRollerKp.hasChanged(hash)
                || Constants.Shooter.kRollerKd.hasChanged(hash)) {
            setRollerPID(
                    Constants.Shooter.kRollerKff.get(),
                    Constants.Shooter.kRollerKp.get(),
                    0,
                    Constants.Shooter.kRollerKd.get());
        }

        if (Constants.Shooter.kRotationKp.hasChanged(hash) || Constants.Shooter.kRotationKd.hasChanged(hash)) {
            setRotationPID(0, Constants.Shooter.kRotationKp.get(), 0, Constants.Shooter.kRotationKd.get());
        }

        var angle = mAngleEncoderInputs
                .position
                .plus(Constants.Shooter.kShooterAngleEncoderOffset)
                .times(Constants.Shooter.kEncoderToShooterReduction);

        mAngle = GeometryUtil.angleModulus(angle, GeometryUtil.kRotationMinusHalfPi, GeometryUtil.kRotationThreeHalfPi);

        // Initialize the rotation motor offset once the angle encoder is sending values
        if (mRotationMotorOffset.isEmpty() && !Util.epsilonEquals(mAngleEncoderInputs.position.getRadians(), 0)) {
            mRotationMotorOffset = Optional.of(angle.div(Constants.Shooter.kAngleReduction));

            zeroShooterRotation();
        }

        Logger.recordOutput("Shooter/calculatedAngle", angle);
        Logger.recordOutput("Shooter/calculatedAngleModulus", mAngle);
        Logger.recordOutput("Shooter/mech", mShooterMechanism);
    }

    private void setRollerPID(double ff, double p, double i, double d) {
        mRightUpperRollerMotorIO.setPID(ff, p, i, d);
        mRightLowerRollerMotorIO.setPID(ff, p, i, d);
        mLeftUpperRollerMotorIO.setPID(ff, p, i, d);
        mLeftLowerRollerMotorIO.setPID(ff, p, i, d);
    }

    private void setRotationPID(double ff, double p, double i, double d) {
        mRotationMotorIO.setPID(ff, p, i, d);
    }

    public void zeroShooterRotation() {
        mRotationMotorIO.setPosition(mAngle);
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

    public double[] getUpperRollerSpeedsRadiansPerSecond() {
        return new double[] {
            mLeftUpperShooterMotorInputs.velocityRadiansPerSecond,
            mRightUpperShooterMotorInputs.velocityRadiansPerSecond
        };
    }

    public double[] getLowerRollerSpeedsRadiansPerSecond() {
        return new double[] {
            mLeftLowerShooterMotorInputs.velocityRadiansPerSecond,
            mRightLowerShooterMotorInputs.velocityRadiansPerSecond
        };
    }

    public void setUnifiedRollerSpeed(double radiansPerSecond) {
        setRightRollerSpeeds(radiansPerSecond);
        setLeftRollerSpeeds(radiansPerSecond);
    }

    public void setRightRollerSpeeds(double radiansPerSecond) {
        setUpperRightRollerSpeed(radiansPerSecond);
        setLowerRightRollerSpeed(radiansPerSecond);
    }

    public void setLeftRollerSpeeds(double radiansPerSecond) {
        setUpperLeftRollerSpeed(radiansPerSecond);
        setLowerLeftRollerSpeed(radiansPerSecond);
    }

    public void setUpperRollerSpeeds(double radiansPerSecond) {
        setUpperLeftRollerSpeed(radiansPerSecond);
        setUpperRightRollerSpeed(radiansPerSecond);
    }

    public void setLowerRollerSpeeds(double radiansPerSecond) {
        setLowerLeftRollerSpeed(radiansPerSecond);
        setLowerRightRollerSpeed(radiansPerSecond);
    }

    public void setUpperRightRollerSpeed(double radiansPerSecond) {
        var calculatedSlew = mRightUpperRollerSlewRateLimiter.calculate(radiansPerSecond);
        var velocity = radiansPerSecond == 0 ? 0 : calculatedSlew;
        Logger.recordOutput("Shooter/Motors/Rollers/UpperRightDemandRadiansPerSecond", velocity);
        mRightUpperRollerMotorIO.setVelocityControl(velocity);
    }

    public void setLowerRightRollerSpeed(double radiansPerSecond) {
        var calculatedSlew = mRightLowerRollerSlewRateLimiter.calculate(radiansPerSecond);
        var velocity = radiansPerSecond == 0 ? 0 : calculatedSlew;
        Logger.recordOutput("Shooter/Motors/Rollers/LowerRightDemandRadiansPerSecond", velocity);
        mRightLowerRollerMotorIO.setVelocityControl(velocity);
    }

    public void setUpperLeftRollerSpeed(double radiansPerSecond) {
        var calculatedSlew = mLeftUpperRollerSlewRateLimiter.calculate(radiansPerSecond);
        var velocity = radiansPerSecond == 0 ? 0 : calculatedSlew;
        Logger.recordOutput("Shooter/Motors/Rollers/UpperLeftDemandRadiansPerSecond", velocity);
        mLeftUpperRollerMotorIO.setVelocityControl(velocity);
    }

    public void setLowerLeftRollerSpeed(double radiansPerSecond) {
        var calculatedSlew = mLeftLowerRollerSlewRateLimiter.calculate(radiansPerSecond);
        var velocity = radiansPerSecond == 0 ? 0 : calculatedSlew;
        Logger.recordOutput("Shooter/Motors/Rollers/LowerLeftDemandRadiansPerSecond", velocity);
        mLeftLowerRollerMotorIO.setVelocityControl(velocity);
    }

    public void setRotationAngle(Rotation2d rotation) {
        if (mRotationMotorOffset.isEmpty()) {
            mRotationMotorIO.setPercentOutput(0.0);
            return;
        }

        // mRotationShooterMotorIO.setSmoothPositionControl(
        //         rotation,
        //         Constants.Shooter.kMaxRotationVelocityRadiansPerSecond.get(),
        //         Constants.Shooter.kMaxRotationAccelerationRadiansPerSecondSquared.get());
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
        mRightUpperRollerMotorIO.setPercentOutput(0);
        mRightLowerRollerMotorIO.setPercentOutput(0);
        mLeftUpperRollerMotorIO.setPercentOutput(0);
        mLeftLowerRollerMotorIO.setPercentOutput(0);
        mRightUpperRollerMotorIO.stopMotor();
        mRightLowerRollerMotorIO.stopMotor();
        mLeftUpperRollerMotorIO.stopMotor();
        mLeftLowerRollerMotorIO.stopMotor();
    }

    public void stopRotation() {
        mRotationMotorIO.setPercentOutput(0);
        mRotationMotorIO.stopMotor();
    }

    public void setRotationPercentOutput(double percent) {
        mRotationMotorIO.setPercentOutput(percent);
    }
}
