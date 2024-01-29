package com.team1701.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.team1701.lib.drivers.digitalinputs.DigitalIOSim;
import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOSim;
import com.team1701.lib.drivers.encoders.EncoderInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private MotorIO mUpperShooterMotorIO;
    private MotorIO mLowerShooterMotorIO;
    private MotorIO mRotationShooterMotorIO;

    private EncoderIO mThroughBoreEncoderIO;

    private final double mInitialShooterAngleRadians;

    private final MotorInputsAutoLogged mUpperShooterMotorInputsAutoLogged = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mLowerShooterMotorInputsAutoLogged = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mRotationShooterMotorInputsAutoLogged = new MotorInputsAutoLogged();
    private final EncoderInputsAutoLogged mEncoderInputsAutoLogged = new EncoderInputsAutoLogged();

    @AutoLogOutput(key = "Shooter/HasPiece")
    private TimeLockedBoolean mShooterHasPiece;

    public Shooter(MotorIO upperMotor, MotorIO lowerMotor, MotorIO rotationMotor, EncoderIO throughBoreEncoder) {

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

        mThroughBoreEncoderIO = throughBoreEncoder;

        mInitialShooterAngleRadians = mEncoderInputsAutoLogged.position.getRadians();
    }

    public static MotorIOSim createMotorSim(DCMotor shooterMotor) {
        return new MotorIOSim(shooterMotor, Constants.Shooter.kShooterReduction, 0.025, Constants.kLoopPeriodSeconds);
    }

    public static DigitalIOSim createDigitalSim(Supplier<Boolean> blockedSupplier) {
        return new DigitalIOSim(blockedSupplier);
    }

    public static EncoderIOSim createEncoderSim(Supplier<Rotation2d> rotationSupplier) {
        return new EncoderIOSim(rotationSupplier);
    }

    @Override
    public void periodic() {
        var hash = hashCode();
        mUpperShooterMotorIO.updateInputs(mUpperShooterMotorInputsAutoLogged);
        mLowerShooterMotorIO.updateInputs(mLowerShooterMotorInputsAutoLogged);
        mRotationShooterMotorIO.updateInputs(mRotationShooterMotorInputsAutoLogged);

        mThroughBoreEncoderIO.updateInputs(mEncoderInputsAutoLogged);

        Logger.processInputs("Shooter/Motors/Rollers/Upper", mUpperShooterMotorInputsAutoLogged);
        Logger.processInputs("Shooter/Motors/Rollers/Lower", mLowerShooterMotorInputsAutoLogged);
        Logger.processInputs("Shooter/Motors/Rotation", mRotationShooterMotorInputsAutoLogged);

        Logger.processInputs("Shooter/Encoder", mRotationShooterMotorInputsAutoLogged);

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
    }

    public void setRollerPID(double ff, double p, double i, double d) {
        mUpperShooterMotorIO.setPID(ff, p, i, d);
        mLowerShooterMotorIO.setPID(ff, p, i, d);
    }

    public void setRotationPID(double ff, double p, double i, double d) {
        mRotationShooterMotorIO.setPID(ff, p, i, d);
    }

    public void setRollerSpeed(double radiansPerSecond) {
        Logger.recordOutput("Shooter/Motors/Rollers/DemandRadiansPerSecond", radiansPerSecond);
        mUpperShooterMotorIO.setVelocityControl(radiansPerSecond);
        mLowerShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setRotationAngle(Rotation2d rotation) {
        var motorRotationDemand = rotation.minus(new Rotation2d(mInitialShooterAngleRadians));

        Logger.recordOutput("Shooter/Motors/Rotation/RawDemand", rotation);
        Logger.recordOutput("Shooter/Motors/Rotation/CalculatedDemand", motorRotationDemand);

        mRotationShooterMotorIO.setPositionControl(motorRotationDemand);
    }

    public void stopRollers() {
        mUpperShooterMotorIO.setPercentOutput(0);
    }

    public void setRotationBrake(boolean enable) {
        mRotationShooterMotorIO.setBrakeMode(enable);
    }
}
