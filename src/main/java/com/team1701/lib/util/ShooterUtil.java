package com.team1701.lib.util;

import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public final class ShooterUtil {

    public static Pose3d getShooterExitPose(RobotState robotState, Shooter shooter) {
        var shooterHingePose = robotState.getPose3d().transformBy(Constants.Robot.kRobotToShooterHinge);
        return new Pose3d(
                shooterHingePose.getTranslation(),
                new Rotation3d(
                        shooterHingePose.getRotation().getX(),
                        shooterHingePose.getRotation().getY()
                                - shooter.getAngle().getRadians(),
                        shooterHingePose.getRotation().getZ()));
    }

    public static Rotation2d calculateStationaryDesiredAngle(RobotState robotState) {
        switch (robotState.getScoringMode()) {
            case SPEAKER:
                return Rotation2d.fromRadians(
                        Constants.Shooter.kShooterAngleInterpolator.get(robotState.getDistanceToSpeaker()));
            case AMP:
                return Rotation2d.fromDegrees(Constants.Shooter.kShooterAmpAngleDegrees.get());
            default:
                return Rotation2d.fromRotations(Constants.Shooter.kShooterLowerLimitRotations);
        }
    }

    public static Rotation2d calculateShooterAngleWithMotion(RobotState robotState, Translation2d expectedTranslation) {
        return Rotation2d.fromRadians(Constants.Shooter.kShooterAngleInterpolator.get(
                robotState.getDistanceToSpeaker(GeometryUtil.toTranslation3d(expectedTranslation))));
    }

    /**
     * @return double[] with upper, then lower speeds
     */
    public static double[] calculateShooterSpeedsWithMotion(RobotState robotState, Translation2d expectedTranslation) {
        var speed = Constants.Shooter.kShooterSpeedInterpolator.get(
                robotState.getDistanceToSpeaker(GeometryUtil.toTranslation3d(expectedTranslation)));
        return new double[] {speed, speed};
    }

    /**
     * @return double[] with upper, then lower speeds
     */
    public static double[] calculateStationaryRollerSpeeds(RobotState robotState) {
        switch (robotState.getScoringMode()) {
            case SPEAKER:
                double[] speeds = new double[2];
                var interpolatedSpeed =
                        Constants.Shooter.kShooterSpeedInterpolator.get(robotState.getDistanceToSpeaker());
                for (int i = 0; i < speeds.length; i++) {
                    speeds[i] = interpolatedSpeed;
                }
                Logger.recordOutput("Shooter/InterpolatedShooterSpeeds", interpolatedSpeed);
                return speeds;
            case AMP:
                return new double[] {
                    Constants.Shooter.kUpperAmpSpeed.get(), Constants.Shooter.kLowerAmpSpeed.get(),
                };
            default:
                return new double[] {0, 0};
        }
    }

    /**
     * @return double[] with upper, then lower speeds
     */
    public static double[] calculateIdleRollerSpeeds(RobotState robotState, Drive drive) {
        double[] speeds = new double[2];
        switch (robotState.getScoringMode()) {
            case SPEAKER:
                double speed;

                if (drive.getKinematicLimits().equals(Constants.Drive.kSlowKinematicLimits)) {
                    speed = Constants.Shooter.kShooterSpeedInterpolator.get(robotState.getDistanceToSpeaker());
                } else {
                    if (robotState.hasNote()) {
                        if (robotState.inNearHalf()) {
                            speed = Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                        } else {
                            speed = robotState.inOpponentWing()
                                    ? 0
                                    : Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                        }
                    } else {
                        speed = robotState.inNearHalf() ? Constants.Shooter.kIdleSpeedRadiansPerSecond.get() : 0;
                    }
                }
                for (int i = 0; i < speeds.length; i++) {
                    speeds[i] = speed;
                }
                break;
            case AMP:
                if (robotState.hasNote()) {
                    speed = robotState.getDistanceToAmp() <= 1
                            ? Constants.Shooter.kAmpRollerSpeedRadiansPerSecond.get()
                            : 250;
                } else {
                    speed = Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                }
                speeds = new double[] {speed, speed};
            case CLIMB:
                speeds = new double[] {0, 0};
            default:
                speeds = new double[] {
                    Constants.Shooter.kIdleSpeedRadiansPerSecond.get(),
                    Constants.Shooter.kIdleSpeedRadiansPerSecond.get(),
                };
        }
        return Util.clampAll(speeds, 0, 440);
    }
}
