package com.team1701.robot.subsystems.shooter;

import java.util.Optional;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOSim;
import com.team1701.lib.drivers.encoders.EncoderInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorIOSparkFlex;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.Robot;
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
        mUpperRollerMotorIO.updateInputs(mUpperShooterMotorInputs);
        mLowerRollerMotorIO.updateInputs(mLowerShooterMotorInputs);

        mRotationMotorIO.updateInputs(mRotationShooterMotorInputs);

        mAngleEncoderIO.updateInputs(mAngleEncoderInputs);

        Logger.processInputs("Shooter/Motors/UpperRoller", mUpperShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/LowerRoller", mLowerShooterMotorInputs);

        Logger.processInputs("Shooter/Motors/Rotation", mRotationShooterMotorInputs);

        Logger.processInputs("Shooter/Encoder", mAngleEncoderInputs);

        if (!Robot.isSimulation()) {
            Logger.recordOutput(
                    "Shooter/Motors/UpperRollerWattage",
                    ((MotorIOSparkFlex) mUpperRollerMotorIO).getAppliedVoltage()
                            * ((MotorIOSparkFlex) mUpperRollerMotorIO).getOutputCurrent());
            Logger.recordOutput(
                    "Shooter/Motors/LowerRollerWattage",
                    ((MotorIOSparkFlex) mLowerRollerMotorIO).getAppliedVoltage()
                            * ((MotorIOSparkFlex) mLowerRollerMotorIO).getOutputCurrent());
        }

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

            zeroShooter();
        }

        Logger.recordOutput("Shooter/calculatedAngle", angle);
        Logger.recordOutput("Shooter/calculatedAngleModulus", mAngle);
        Logger.recordOutput("Shooter/mech", mShooterMechanism);
    }

    private void setRollerPID(double ff, double p, double i, double d) {
        mUpperRollerMotorIO.setPID(ff, p, i, d);
        mLowerRollerMotorIO.setPID(ff, p, i, d);
    }

    private void setRotationPID(double ff, double p, double i, double d) {
        mRotationMotorIO.setPID(ff, p, i, d);
    }

    public void zeroShooter() {
        mRotationMotorIO.setPosition(mAngle);
    }

    @AutoLogOutput
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(mRotationShooterMotorInputs.positionRadians);
    }

    /**
     * @return double[] with upper, then lower speeds
     */
    public double[] getRollerSpeedsRadiansPerSecond() {
        return new double[] {
            mUpperShooterMotorInputs.velocityRadiansPerSecond, mLowerShooterMotorInputs.velocityRadiansPerSecond,
        };
    }

    public double getUpperRollerSpeedRadiansPerSecond() {
        return mUpperShooterMotorInputs.velocityRadiansPerSecond;
    }

    public double getLowerRollerSpeedRadiansPerSecond() {
        return mLowerShooterMotorInputs.velocityRadiansPerSecond;
    }

    /**
     * upper, lower
     */
    public void setRollerSpeeds(double[] speeds) {
        setUpperRollerSpeed(speeds.length > 0 ? speeds[0] : Constants.Shooter.kIdleSpeedRadiansPerSecond.get());
        setLowerRollerSpeed(speeds.length > 1 ? speeds[1] : Constants.Shooter.kIdleSpeedRadiansPerSecond.get());
    }

    public void setUnifiedRollerSpeed(double radiansPerSecond) {
        setRollerSpeeds(new double[] {radiansPerSecond, radiansPerSecond});
    }

    public void setUpperRollerSpeed(double radiansPerSecond) {
        var calculatedSlew = mUpperRollerSlewRateLimiter.calculate(radiansPerSecond);
        var velocity = radiansPerSecond == 0 ? 0 : calculatedSlew;
        Logger.recordOutput("Shooter/Motors/Rollers/UpperRightDemandRadiansPerSecond", velocity);
        mUpperRollerMotorIO.setVelocityControl(velocity);
    }

    public void setLowerRollerSpeed(double radiansPerSecond) {
        var calculatedSlew = mLowerRollerSlewRateLimiter.calculate(radiansPerSecond);
        var velocity = radiansPerSecond == 0 ? 0 : calculatedSlew;
        Logger.recordOutput("Shooter/Motors/Rollers/LowerRightDemandRadiansPerSecond", velocity);
        mLowerRollerMotorIO.setVelocityControl(velocity);
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
        mUpperRollerMotorIO.setPercentOutput(0);
        mLowerRollerMotorIO.setPercentOutput(0);
        mUpperRollerMotorIO.stopMotor();
        mLowerRollerMotorIO.stopMotor();
    }

    public void stopRotation() {
        mRotationMotorIO.setPercentOutput(0);
        mRotationMotorIO.stopMotor();
    }

    public void setRotationPercentOutput(double percent) {
        mRotationMotorIO.setPercentOutput(percent);
    }
}
