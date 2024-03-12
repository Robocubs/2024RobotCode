package com.team1701.robot.subsystems.climb;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private final MotorIO mLeftWinchIO;
    private final MotorIO mRightWinchIO;

    private final MotorInputsAutoLogged mLeftWinchMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mRightWinchMotorInputs = new MotorInputsAutoLogged();

    public Climb(MotorIO leftWinch, MotorIO rightWinch) {
        mLeftWinchIO = leftWinch;
        mRightWinchIO = rightWinch;

        mLeftWinchIO.setBrakeMode(true);
        mRightWinchIO.setBrakeMode(true);

        setWinchPID(Constants.Climb.kWinchKff.get(), Constants.Climb.kWinchKp.get(), 0, Constants.Climb.kWinchKd.get());
    }

    private void setWinchPID(double ff, double p, double i, double d) {
        mLeftWinchIO.setPID(ff, p, i, d);
        mRightWinchIO.setPID(ff, p, i, d);
    }

    public static MotorIOSim createWinchMotorIOSim(DCMotor winchMotor) {
        return new MotorIOSim(winchMotor, Constants.Climb.kWinchReduction, 0.14, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        var hash = hashCode();

        mLeftWinchIO.updateInputs(mLeftWinchMotorInputs);
        mRightWinchIO.updateInputs(mRightWinchMotorInputs);

        Logger.processInputs("Climb/LeftWinch", mLeftWinchMotorInputs);
        Logger.processInputs("Climb/RightWinch", mRightWinchMotorInputs);

        if (Constants.Climb.kWinchKff.hasChanged(hash)
                || Constants.Climb.kWinchKp.hasChanged(hash)
                || Constants.Climb.kWinchKd.hasChanged(hash)) {
            setWinchPID(
                    Constants.Climb.kWinchKff.get(), Constants.Climb.kWinchKp.get(), 0, Constants.Climb.kWinchKd.get());
        }
    }

    public void retractWinch() {
        setPercentOutput(-.9);
    }

    public void extendWinch() {
        setPercentOutput(.8);
    }

    public void setPercentOutput(double percent) {
        mLeftWinchIO.setPercentOutput(percent);
        mRightWinchIO.setPercentOutput(percent);
    }

    public void stop() {
        mLeftWinchIO.setPercentOutput(0);
        mRightWinchIO.setPercentOutput(0);
    }
}
