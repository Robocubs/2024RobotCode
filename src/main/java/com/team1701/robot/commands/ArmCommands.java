package com.team1701.robot.commands;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.arm.Arm;
import com.team1701.robot.subsystems.arm.Arm.ArmPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ArmCommands {
    private static final LoggedTunableNumber kArmAngleToleranceRadians =
            new LoggedTunableNumber("Command/PositionArm/ArmAngleToleranceRadians", 0.01);

    public static Command idleArmCommand(Arm arm, RobotState robotState) {
        return Commands.run(
                        () -> {
                            Rotation2d targetRotation =
                                    switch (robotState.getScoringMode()) {
                                        case SPEAKER -> ArmPosition.HOME.armRotation;
                                        case AMP -> robotState.getDistanceToAmp() <= 1
                                                ? ArmPosition.AMP.armRotation
                                                : ArmPosition.HOME.armRotation;
                                        case CLIMB -> ArmPosition.LOW_CLIMB.armRotation;
                                        default -> ArmPosition.HOME.armRotation;
                                    };
                            arm.setRotationAngle(targetRotation);
                        },
                        arm)
                .withName("IdleArmCommand");
    }

    public static Command positionArm(Arm arm, Rotation2d angle) {
        return Commands.run(() -> arm.setRotationAngle(angle), arm)
                .until(() -> GeometryUtil.isNear(
                        arm.getAngle(), angle, Rotation2d.fromRadians(kArmAngleToleranceRadians.get())))
                .withName("PositionArm");
    }
}
