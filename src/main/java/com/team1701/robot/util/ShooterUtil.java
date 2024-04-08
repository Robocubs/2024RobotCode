package com.team1701.robot.util;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSetpoint;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public final class ShooterUtil {

    public static double calculateTheoreticalAngle(double distance) {
        return Math.asin(Constants.Shooter.kSpeakerToShooterHingeDifference / distance);
    }

    public static double calculateSpeakerSpeed(double distanceToSpeaker) {
        return Constants.Shooter.kUseNewCurves
                ? Constants.Shooter.kSpeedRegression.predict(distanceToSpeaker)
                : Constants.Shooter.kShooterSpeedInterpolator.get(distanceToSpeaker);
    }

    public static double calculateSpeakerAngle(double distanceToSpeaker) {
        return Constants.Shooter.kUseNewCurves
                ? Constants.Shooter.kAngleRegression.predict(calculateTheoreticalAngle(distanceToSpeaker))
                : Constants.Shooter.kShooterAngleInterpolator.get(distanceToSpeaker);
    }

    public static ShooterSetpoint calculateSetpoint(double distanceToSpeaker) {
        return new ShooterSetpoint(
                new ShooterSpeeds(calculateSpeakerSpeed(distanceToSpeaker)),
                GeometryUtil.clampRotation(
                        Rotation2d.fromRadians(calculateSpeakerAngle(distanceToSpeaker)),
                        Constants.Shooter.kShooterLowerLimit,
                        Constants.Shooter.kShooterUpperLimit));
    }

    public static ShooterSetpoint calculateStationarySetpoint(RobotState robotState) {
        return new ShooterSetpoint(
                calculateStationaryRollerSpeeds(robotState), calculateStationaryDesiredAngle(robotState));
    }

    public static ShooterSetpoint calculatePassingSetpoint(RobotState robotState) {
        return new ShooterSetpoint(
                Constants.Shooter.kPassingSpeedInterpolator.get(robotState.getPassingDistance()),
                Rotation2d.fromRadians(
                        Constants.Shooter.kPassingAngleInterpolator.get(robotState.getPassingDistance())));
    }

    public static ShooterSetpoint calculateIdleSetpoint(RobotState robotState) {
        return new ShooterSetpoint(
                calculateIdleRollerSpeeds(robotState),
                robotState.hasLoadedNote()
                        ? calculateStationaryDesiredAngle(robotState)
                        : Constants.Shooter.kLoadingAngle);
    }

    private static Rotation2d calculateStationaryDesiredAngle(RobotState robotState) {
        var rotation =
                switch (robotState.getScoringMode()) {
                    case SPEAKER -> Rotation2d.fromRadians(calculateSpeakerAngle(robotState.getDistanceToSpeaker()));
                    case AMP -> Rotation2d.fromDegrees(Constants.Shooter.kShooterAmpAngleDegrees.get());
                    default -> Constants.Shooter.kLoadingAngle;
                };

        return GeometryUtil.clampRotation(
                rotation, Constants.Shooter.kShooterLowerLimit, Constants.Shooter.kShooterUpperLimit);
    }

    private static ShooterSpeeds calculateStationaryRollerSpeeds(RobotState robotState) {
        switch (robotState.getScoringMode()) {
            case SPEAKER:
                var speed = calculateSpeakerSpeed(robotState.getDistanceToSpeaker());
                return new ShooterSpeeds(speed);
            case AMP:
                return new ShooterSpeeds(
                        Constants.Shooter.kUpperAmpSpeed.get(), Constants.Shooter.kLowerAmpSpeed.get());
            default:
                return ShooterSpeeds.kZero;
        }
    }

    private static ShooterSpeeds calculateIdleRollerSpeeds(RobotState robotState) {
        ShooterSpeeds speeds;
        switch (robotState.getScoringMode()) {
            case SPEAKER:
                double speed;
                var distance = robotState.getDistanceToSpeaker();

                if (robotState.inOpponentWing()) {
                    speed = 0;
                } else if (robotState.hasNote()) {
                    speed = calculateSpeakerSpeed(distance);
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
                speeds = ShooterSpeeds.kZero;
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
