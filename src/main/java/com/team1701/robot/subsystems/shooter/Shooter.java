package com.team1701.robot.subsystems.shooter;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private MotorIO mShooterMotorIO;
    private final MotorInputsAutoLogged mShooterMotorInputsAutoLogged = new MotorInputsAutoLogged();

    @AutoLogOutput(key = "Shooter/Motor/DemandRadiansPerSecond")
    private double mDemandRadiansPerSecond;

    public Shooter(MotorIO motor) {
        mShooterMotorIO = motor;
        setPID(
                Constants.Shooter.kShooterKff.get(),
                Constants.Shooter.kShooterKp.get(),
                0,
                Constants.Shooter.kShooterKd.get());
        mShooterMotorIO.setBrakeMode(false);
    }

    public static MotorIOSim createSim(DCMotor shooterMotor) {
        return new MotorIOSim(shooterMotor, Constants.Shooter.kShooterReduction, 0.025, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        var hash = hashCode();
        mShooterMotorIO.updateInputs(mShooterMotorInputsAutoLogged);
        Logger.processInputs("Shooter/Motor", mShooterMotorInputsAutoLogged);
        if (Constants.Shooter.kShooterKff.hasChanged(hash)
                || Constants.Shooter.kShooterKp.hasChanged(hash)
                || Constants.Shooter.kShooterKd.hasChanged(hash)) {
            setPID(
                    Constants.Shooter.kShooterKff.get(),
                    Constants.Shooter.kShooterKp.get(),
                    0,
                    Constants.Shooter.kShooterKd.get());
        }
    }

    public void setPID(double ff, double p, double i, double d) {
        mShooterMotorIO.setPID(ff, p, i, d);
    }

    public void setSpeed(double radiansPerSecond) {
        mShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void stop() {
        mShooterMotorIO.setPercentOutput(0);
    }
}
