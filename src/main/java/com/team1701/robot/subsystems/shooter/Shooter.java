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

    @AutoLogOutput(key = "Shooter/DemandRadiansPerSecond")
    private double mLeftDemandRadiansPerSecond;

    public Shooter(MotorIO motor) {
        mShooterMotorIO = motor;
        mShooterMotorIO.setPID(
                Constants.Shooter.kShooterKf.get(),
                Constants.Shooter.kShooterKp.get(),
                0,
                Constants.Shooter.kShooterKd.get());
    }

    public static MotorIOSim createSim(DCMotor shooterMotor) {
        return new MotorIOSim(shooterMotor, Constants.Shooter.kShooterReduction, 0.025, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        // TODO: Update inputs
        mShooterMotorIO.updateInputs(mShooterMotorInputsAutoLogged);
        Logger.processInputs("Shooter/Motor", mShooterMotorInputsAutoLogged);
    }

    public void setSpeed(double radiansPerSecond) {
        mShooterMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void setShooterBrakeMode(boolean enable) {
        mShooterMotorIO.setBrakeMode(enable);
    }

    public void stop() {
        mShooterMotorIO.setPercentOutput(0);
    }
}
