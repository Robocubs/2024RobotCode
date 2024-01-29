package com.team1701.robot.commands;

import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shoot extends Command {
    private final Shooter mShooter;
    private final Drive mDrive;

    private final RobotState mRobotState = new RobotState();

    @AutoLogOutput(key = "Commands/Shoot/calculatedDistanceToTarget")
    private double distanceToTarget;

    @AutoLogOutput(key = "Commands/Shoot/calculatedShooterAngleFromHorizontal")
    private Rotation2d shooterAngleFromHorizontal;

    @AutoLogOutput(key = "Commands/Shoot/robotRelativeRotationDemand")
    private Rotation2d robotRelativeRotationDemand;

    @AutoLogOutput(key = "Commands/Shoot/shotFired")
    private boolean shotFired;

    public Shoot(Shooter shooter, Drive drive, boolean allianceIsBlue) {
        mShooter = shooter;
        mDrive = drive;
        addRequirements(shooter, drive);
    }

    @Override
    public void initialize() {
        shotFired = false;
        distanceToTarget = mRobotState
                .getPose2d()
                .getTranslation()
                .getDistance(mRobotState.getSpeakerPose().toTranslation2d());
        robotRelativeRotationDemand =
                mRobotState.getSpeakerHeading().minus(mRobotState.getPose2d().getRotation());
        shooterAngleFromHorizontal = new Rotation2d(
                distanceToTarget - Constants.Shooter.kShooterAxisOffset,
                FieldConstants.kBlueSpeakerOpeningCenter.getZ() - Constants.Shooter.kShooterAxisHeight);
        mShooter.setRotationBrake(true);
    }

    @Override
    public void execute() {
        if (mRobotState.getPose2d().getRotation() != mRobotState.getSpeakerHeading()) {
            DriveCommands.rotateRelativeToRobot(
                    mDrive, robotRelativeRotationDemand, Constants.Drive.kFastKinematicLimits, true);
        }

        mShooter.setRotationAngle(shooterAngleFromHorizontal);

        mShooter.setRollerSpeed(Units.rotationsPerMinuteToRadiansPerSecond(Constants.Motors.kMaxKrakenRPM));
        // TODO: implement autostart/stop with sensors

        if (true /* TODO !shooterHasPiece */) {
            mShooter.stopRollers();
            shotFired = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return shotFired;
    }
}
