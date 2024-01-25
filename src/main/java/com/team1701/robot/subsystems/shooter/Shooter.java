package com.team1701.robot.subsystems.shooter;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
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
    private final MotorInputsAutoLogged mUpperShooterMotorInputsAutoLogged = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mLowerShooterMotorInputsAutoLogged = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mRotationShooterMotorInputsAutoLogged = new MotorInputsAutoLogged();

    @AutoLogOutput(key = "Shooter/Motors/Rollers/Upper/DemandRadiansPerSecond")
    private double mUpperDemandRadiansPerSecond;

    @AutoLogOutput(key = "Shooter/Motors/Rollers/Lower/DemandRadiansPerSecond")
    private double mLowerDemandRadiansPerSecond;

    @AutoLogOutput(key = "Shooter/Motors/Rotation/DemandRadians")
    private double mRotationDemandRadians;

    public Shooter(MotorIO upperMotor, MotorIO lowerMotor, MotorIO rotationMotor) {
        mUpperShooterMotorIO = upperMotor;
        mLowerShooterMotorIO = lowerMotor;
        mRotationShooterMotorIO = rotationMotor;
        setRollerPID(
                Constants.Shooter.kShooterKff.get(),
                Constants.Shooter.kShooterKp.get(),
                0,
                Constants.Shooter.kShooterKd.get());
        mUpperShooterMotorIO.setBrakeMode(false);
    }

    public static MotorIOSim createSim(DCMotor shooterMotor) {
        return new MotorIOSim(shooterMotor, Constants.Shooter.kShooterReduction, 0.025, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        var hash = hashCode();
        mUpperShooterMotorIO.updateInputs(mUpperShooterMotorInputsAutoLogged);
        mLowerShooterMotorIO.updateInputs(mLowerShooterMotorInputsAutoLogged);
        mRotationShooterMotorIO.updateInputs(mRotationShooterMotorInputsAutoLogged);
        Logger.processInputs("Shooter/Motors/Rollers/Upper", mUpperShooterMotorInputsAutoLogged);
        Logger.processInputs("Shooter/Motors/Rollers/Lower", mLowerShooterMotorInputsAutoLogged);
        Logger.processInputs("Shooter/Motors/Rotation", mRotationShooterMotorInputsAutoLogged);
        if (Constants.Shooter.kShooterKff.hasChanged(hash)
                || Constants.Shooter.kShooterKp.hasChanged(hash)
                || Constants.Shooter.kShooterKd.hasChanged(hash)) {
            setRollerPID(
                    Constants.Shooter.kShooterKff.get(),
                    Constants.Shooter.kShooterKp.get(),
                    0,
                    Constants.Shooter.kShooterKd.get());
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
        mUpperShooterMotorIO.setVelocityControl(radiansPerSecond);
        mLowerShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setRotationAngle(Rotation2d rotation) {
        mRotationShooterMotorIO.setPositionControl(rotation);
    }

    public void stopRollers() {
        mUpperShooterMotorIO.setPercentOutput(0);
    }

    public void setRotationBrake(boolean enable) {
        mRotationShooterMotorIO.setBrakeMode(enable);
    }
}
