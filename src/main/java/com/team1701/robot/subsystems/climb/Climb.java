package com.team1701.robot.subsystems.climb;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

        setWinchPID(Constants.Winch.kWinchKff.get(), Constants.Winch.kWinchKp.get(), 0, Constants.Winch.kWinchKd.get());
    }

    private void setWinchPID(double ff, double p, double i, double d) {
        mLeftWinchIO.setPID(ff, p, i, d);
        mRightWinchIO.setPID(ff, p, i, d);
    }

    public static MotorIOSim createWinchMotorIOSim(DCMotor winchMotor) {
        return new MotorIOSim(winchMotor, Constants.Winch.kWinchReduction, 0.14, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        var hash = hashCode();

        mLeftWinchIO.updateInputs(mLeftWinchMotorInputs);
        mRightWinchIO.updateInputs(mRightWinchMotorInputs);

        if (Constants.Winch.kWinchKff.hasChanged(hash)
                || Constants.Winch.kWinchKp.hasChanged(hash)
                || Constants.Winch.kWinchKd.hasChanged(hash)) {
            setWinchPID(
                    Constants.Winch.kWinchKff.get(), Constants.Winch.kWinchKp.get(), 0, Constants.Winch.kWinchKd.get());
        }
    }

    public void retractWinch(double distanceMeters) {
        var rotations = distanceMeters / Constants.Winch.kWinchCircumference;
        mLeftWinchIO.setSmoothPositionControl(
                Rotation2d.fromRotations(rotations),
                Constants.Arm.kMaxRotationVelocityRadiansPerSecond.get(),
                Constants.Arm.kMaxRotationAccelerationRadiansPerSecondSquared.get());
        mRightWinchIO.setSmoothPositionControl(
                Rotation2d.fromRotations(rotations),
                Constants.Arm.kMaxRotationVelocityRadiansPerSecond.get(),
                Constants.Arm.kMaxRotationAccelerationRadiansPerSecondSquared.get());
    }

    public void stop() {
        mLeftWinchIO.setPercentOutput(0);
        mRightWinchIO.setPercentOutput(0);
    }
}
