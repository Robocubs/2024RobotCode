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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final MotorIO mUpperShooterMotorIO;
    private final MotorIO mLowerShooterMotorIO;
    private final MotorIO mRotationShooterMotorIO;
    private final EncoderIO mAngleEncoderIO;

    private final MotorInputsAutoLogged mUpperShooterMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mLowerShooterMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mRotationShooterMotorInputs = new MotorInputsAutoLogged();
    private final EncoderInputsAutoLogged mAngleEncoderInputs = new EncoderInputsAutoLogged();

    private Optional<Rotation2d> mRotationMotorOffset = Optional.empty();

    public Shooter(MotorIO upperMotor, MotorIO lowerMotor, MotorIO rotationMotor, EncoderIO angleEncoder) {
        mUpperShooterMotorIO = upperMotor;
        mLowerShooterMotorIO = lowerMotor;
        mRotationShooterMotorIO = rotationMotor;

        mUpperShooterMotorIO.setBrakeMode(false);
        mLowerShooterMotorIO.setBrakeMode(false);
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

    @Override
    public void periodic() {
        var hash = hashCode();
        mUpperShooterMotorIO.updateInputs(mUpperShooterMotorInputs);
        mLowerShooterMotorIO.updateInputs(mLowerShooterMotorInputs);
        mRotationShooterMotorIO.updateInputs(mRotationShooterMotorInputs);

        mAngleEncoderIO.updateInputs(mAngleEncoderInputs);

        Logger.processInputs("Shooter/Motors/Rollers/Upper", mUpperShooterMotorInputs);
        Logger.processInputs("Shooter/Motors/Rollers/Lower", mLowerShooterMotorInputs);
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
            mRotationMotorOffset = Optional.of(mAngleEncoderInputs
                    .position
                    .minus(Constants.Shooter.kShooterAngleEncoderOffset)
                    .minus(Rotation2d.fromRadians(mRotationShooterMotorInputs.positionRadians)));
        }
    }

    private void setRollerPID(double ff, double p, double i, double d) {
        mUpperShooterMotorIO.setPID(ff, p, i, d);
        mLowerShooterMotorIO.setPID(ff, p, i, d);
    }

    private void setRotationPID(double ff, double p, double i, double d) {
        mRotationShooterMotorIO.setPID(ff, p, i, d);
    }

    @AutoLogOutput
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(mRotationShooterMotorInputs.positionRadians)
                .plus(mRotationMotorOffset.orElse(GeometryUtil.kRotationIdentity));
    }

    public double[] getRollerSpeedsRadiansPerSecond() {
        return new double[] {
            mUpperShooterMotorInputs.velocityRadiansPerSecond, mLowerShooterMotorInputs.velocityRadiansPerSecond
        };
    }

    public void setRollerSpeed(double radiansPerSecond) {
        Logger.recordOutput("Shooter/Motors/Rollers/DemandRadiansPerSecond", radiansPerSecond);
        mUpperShooterMotorIO.setVelocityControl(radiansPerSecond);
        mLowerShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setRotationAngle(Rotation2d rotation) {
        Logger.recordOutput("Shooter/AngleDemand", rotation);
        if (mRotationMotorOffset.isEmpty()) {
            mRotationShooterMotorIO.setPercentOutput(0.0);
            return;
        }

        var motorRotationDemand = rotation.minus(mRotationMotorOffset.get());
        mRotationShooterMotorIO.setPositionControl(motorRotationDemand);

        Logger.recordOutput("Shooter/Motors/Rotation/Demand", motorRotationDemand);
    }

    public void stopRollers() {
        mUpperShooterMotorIO.setPercentOutput(0);
        mLowerShooterMotorIO.setPercentOutput(0);
    }

    public void stopRotation() {
        mRotationShooterMotorIO.setPercentOutput(0);
    }
}
