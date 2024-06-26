package com.team1701.robot.subsystems.climb;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.tuning.LoggedTunableValue;
import com.team1701.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
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

        setWinchPID();
    }

    private void setWinchPID() {
        mLeftWinchIO.setPID(Constants.Climb.kWinchKp.get(), 0, Constants.Climb.kWinchKd.get());
        mRightWinchIO.setPID(Constants.Climb.kWinchKp.get(), 0, Constants.Climb.kWinchKd.get());
    }

    public static MotorIOSim createWinchMotorIOSim(DCMotor winchMotor) {
        return new MotorIOSim(winchMotor, Constants.Climb.kWinchReduction, 0.14, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        mLeftWinchIO.updateInputs(mLeftWinchMotorInputs);
        mRightWinchIO.updateInputs(mRightWinchMotorInputs);

        Logger.processInputs("Climb/LeftWinch", mLeftWinchMotorInputs);
        Logger.processInputs("Climb/RightWinch", mRightWinchMotorInputs);

        LoggedTunableValue.ifChanged(hashCode(), this::setWinchPID, Constants.Climb.kWinchKp, Constants.Climb.kWinchKd);
    }

    public void retractWinch() {
        setPercentOutput(-.9);
    }

    public void extendWinch() {
        setPercentOutput(.8);
    }

    public void setPosition(Rotation2d rotation) {
        mLeftWinchIO.setPositionControl(rotation);
        mRightWinchIO.setPositionControl(rotation);
    }

    public void setLeftClimbPosition() {
        mLeftWinchIO.setPositionControl(Constants.Climb.kMaxSetpoint);
        mRightWinchIO.setPositionControl(Constants.Climb.kMiddleSetpoint);
    }

    public void setRightClimbPosition() {
        mLeftWinchIO.setPositionControl(Constants.Climb.kMiddleSetpoint);
        mRightWinchIO.setPositionControl(Constants.Climb.kMaxSetpoint);
    }

    public void setMidClimbPosition() {
        setPosition(Constants.Climb.kMiddleSetpoint);
    }

    public void setPercentOutput(double percent) {
        Logger.recordOutput("Climb/PercentOutput", percent);
        mLeftWinchIO.setPercentOutput(percent);
        mRightWinchIO.setPercentOutput(percent);
    }

    public void stop() {
        Logger.recordOutput("Climb/PercentOutput", 0.0);
        mLeftWinchIO.stopMotor();
        mRightWinchIO.stopMotor();
    }
}
