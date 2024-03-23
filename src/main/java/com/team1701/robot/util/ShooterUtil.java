package com.team1701.robot.util;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public final class ShooterUtil {

    public static Rotation2d calculateStationaryDesiredAngle(RobotState robotState) {
        switch (robotState.getScoringMode()) {
            case SPEAKER:
                return Rotation2d.fromRadians(
                        Constants.Shooter.kShooterAngleInterpolator.get(robotState.getDistanceToSpeaker()));
            case AMP:
                return Rotation2d.fromDegrees(Constants.Shooter.kShooterAmpAngleDegrees.get());
            default:
                return Constants.Shooter.kLoadingAngle;
        }
    }

    public static Rotation2d calculateShooterAngleWithMotion(RobotState robotState, Translation2d expectedTranslation) {
        return Rotation2d.fromRadians(Constants.Shooter.kShooterAngleInterpolator.get(
                robotState.getDistanceToSpeaker(GeometryUtil.toTranslation3d(expectedTranslation))));
    }

    public static ShooterSpeeds calculateShooterSpeedsWithMotion(
            RobotState robotState, Translation2d expectedTranslation) {
        return new ShooterSpeeds(Constants.Shooter.kShooterSpeedInterpolator.get(
                robotState.getDistanceToSpeaker(GeometryUtil.toTranslation3d(expectedTranslation))));
    }

    public static ShooterSpeeds calculateStationaryRollerSpeeds(RobotState robotState) {
        switch (robotState.getScoringMode()) {
            case SPEAKER:
                var interpolatedSpeed =
                        Constants.Shooter.kShooterSpeedInterpolator.get(robotState.getDistanceToSpeaker());
                Logger.recordOutput("Shooter/InterpolatedShooterSpeeds", interpolatedSpeed);
                return new ShooterSpeeds(interpolatedSpeed);
            case AMP:
                return new ShooterSpeeds(
                        Constants.Shooter.kUpperAmpSpeed.get(), Constants.Shooter.kLowerAmpSpeed.get());
            default:
                return new ShooterSpeeds(0);
        }
    }

    public static ShooterSpeeds calculatePassingShooterSpeeds(RobotState robotState) {
        var interpolatedSpeed = Constants.Shooter.kPassingSpeedInterpolator.get(robotState.getPassingDistance());
        Logger.recordOutput("Shooter/InterpolatedPassingSpeeds", interpolatedSpeed);
        return new ShooterSpeeds(interpolatedSpeed);
    }

    public static Rotation2d calculatePassingShooterAngle(RobotState robotState) {
        return Rotation2d.fromRadians(Constants.Shooter.kPassingAngleInterpolator.get(robotState.getPassingDistance()));
    }

    public static ShooterSpeeds calculateIdleRollerSpeeds(RobotState robotState) {
        ShooterSpeeds speeds;
        switch (robotState.getScoringMode()) {
            case SPEAKER:
                double speed;

                if (!robotState.inNearHalf()) {
                    speed = 0;
                } else if (robotState.hasNote()) {
                    speed = Constants.Shooter.kShooterSpeedInterpolator.get(robotState.getDistanceToSpeaker());
                } else {
                    speed = Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                }
                speeds = new ShooterSpeeds(speed);

                break;
            case AMP:
                speeds = new ShooterSpeeds(
                        Constants.Shooter.kUpperAmpSpeed.get(), Constants.Shooter.kLowerAmpSpeed.get());
                break;
            case CLIMB:
                speeds = new ShooterSpeeds(0);
                break;
            default:
                speeds = new ShooterSpeeds(Constants.Shooter.kIdleSpeedRadiansPerSecond.get());
        }
        return clampSpeeds(speeds, 0, 440);
    }

    public static ShooterSpeeds clampSpeeds(ShooterSpeeds speeds, double lower, double upper) {
        return new ShooterSpeeds(
                MathUtil.clamp(speeds.upperSpeed(), lower, upper), MathUtil.clamp(speeds.lowerSpeed(), lower, upper));
    }
}
