package com.team1701.robot.subsystems.elevator;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {
    private MotorIO mLeftElevatorMotorIO;
    private MotorIO mRightElevatorMotorIO;
    private final MotorInputsAutoLogged mElevatorMotorInputsAutoLogged = new MotorInputsAutoLogged();

    @AutoLogOutput(key = "Elevator/Motors/Left/DemandRadians")
    private double mLeftDemandRadians;

    @AutoLogOutput(key = "Elevator/Motors/Right/DemandRadians")
    private double mRightDemandRadians;

    public Elevator(MotorIO leftMotor, MotorIO rightMotor) {
        mLeftElevatorMotorIO = leftMotor;
        mRightElevatorMotorIO = rightMotor;

        // mLeftElevatorMotorIO.setPID(, , , );
        // mRightElevatorMotorIO.setPID(, , , );
    }
}
