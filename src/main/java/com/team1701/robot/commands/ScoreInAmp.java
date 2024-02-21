package com.team1701.robot.commands;

import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.arm.Arm;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ScoreInAmp extends Command {
    private static final String kLoggingPrefix = "Command/ScoreInAmp/";
    private static final LoggedTunableNumber kShooterAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "ShooterAngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kArmAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "ArmAngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kTargetSpeedRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "TargetSpeedRadiansPerSecond", 600);
    private static final LoggedTunableNumber kShooterAmpAngleDegrees =
            new LoggedTunableNumber(kLoggingPrefix + "Shooter/AmpAngleDegrees", 85);

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final Arm mArm;
    private final RobotState mRobotState;

    private boolean mShooting;
    private boolean mArmAtAngle;

    public ScoreInAmp(Shooter shooter, Indexer indexer, Arm arm, RobotState robotState) {
        mShooter = shooter;
        mIndexer = indexer;
        mArm = arm;
        mRobotState = robotState;

        addRequirements(shooter, indexer, arm);
    }

    @Override
    public void initialize() {
        mShooting = false;
    }

    @Override
    public void execute() {
        mShooter.setRotationAngle(Rotation2d.fromDegrees(kShooterAmpAngleDegrees.get()));
        mArm.rotateToAmpPosition();

        // TODO: Determine linear regression of speeds
        var targetSpeed = kTargetSpeedRadiansPerSecond.get();
        mShooter.setRollerSpeed(targetSpeed);

        var atShooterAngle = GeometryUtil.isNear(
                mShooter.getAngle(),
                Rotation2d.fromDegrees(kShooterAmpAngleDegrees.get()),
                Rotation2d.fromRadians(kShooterAngleToleranceRadians.get()));

        var atArmAngle = GeometryUtil.isNear(
                mShooter.getAngle(),
                Rotation2d.fromDegrees(Constants.Arm.kArmAmpRotationDegrees.get()),
                Rotation2d.fromRadians(kArmAngleToleranceRadians.get()));


        var atSpeed = DoubleStream.of(mShooter.getRollerSpeedsRadiansPerSecond())
                .allMatch(actualSpeed -> MathUtil.isNear(targetSpeed, actualSpeed, 10.0));

        if (atShooterAngle && atArmAngle && atSpeed) {
            mIndexer.setForwardShoot();
            mShooting = true;
        }

        if (!mShooting) {
            if (mIndexer.hasNoteAtExit()) {
                mIndexer.stop();
            } else {
                mIndexer.setForwardLoad();
            }
        }

        Logger.recordOutput(kLoggingPrefix + "Shooting", mShooting);
        Logger.recordOutput(kLoggingPrefix + "AtAngle", atShooterAngle);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", atSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRollers();
        mShooter.stopRotation();
        mIndexer.stop();
        mArm.rotateHome();
    }

    @Override
    public boolean isFinished() {
        return mShooting && !mRobotState.hasNote();
    }


}
