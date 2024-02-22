package com.team1701.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team1701.lib.drivers.cameras.config.VisionCameraConfig;
import com.team1701.lib.drivers.cameras.config.VisionConfig;
import com.team1701.lib.swerve.ExtendedSwerveDriveKinematics;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double kLoopPeriodSeconds = 0.02;

    public enum ScoringMode {
        SPEAKER,
        AMP,
        CLIMB
    }

    public static final class Robot {
        public static final double kRobotWidth = Units.inchesToMeters(23);
        public static final double kRobotLength = Units.inchesToMeters(28.5);
        public static final double kRobotWidthWithBumpers = kRobotWidth + Units.inchesToMeters(8);
        public static final double kRobotLengthWithBumpers = kRobotLength + Units.inchesToMeters(8);
        public static final double kRobotFrontToCenter = Units.inchesToMeters(23.0 / 2.0);
        public static final double kRobotBackToCenter = kRobotLength - kRobotFrontToCenter;
        public static final double kRobotSideToCenter = kRobotWidth / 2.0;

        public static final Transform3d kRobotToShooterHinge = new Transform3d(
                new Translation3d(Units.inchesToMeters(-3), Units.inchesToMeters(0), Units.inchesToMeters(7.52)),
                GeometryUtil.kRotation3dIdentity);
        public static final Transform3d kShooterHingeToShooterExit = new Transform3d(
                new Translation3d(Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(1.9)),
                GeometryUtil.kRotation3dIdentity);
    }

    public static final class Vision {
        /*
         * The below system improves upon last season's reliance on the provided standard
         * deviation values for vision in the WPI pose estimation library:
         *
         * In the lab, standard deviations for the XY direction and theta were collected at various distances and angles.
         * We plug these *measured* distances and angles and their corresponding standard deviations into interpolation maps.
         * So there is a key and value system similar to HashMaps.
         * These maps will return these std. dev. values when the exact key (i.e. either distance (m) or angle (radians))
         * is provided. If there is no such key, then it uses math to derive a value for the key using the nearest existing keys.
         * Think about it like the "line of best fit" you found in math class.
         *
         * Remember:
         *
         * For a given distance from the camera to an AprilTag, what is a mostly-accurate std. dev. I can use?
         * Keys are MEASURED distances or angles collected in the lab at KNOWN standard deviations.
         * Values are standard deviations, either calculated or stored.
         */
        public static final boolean kUseInterpolatedVisionStdDevValues = true;

        // TODO: Collect values
        public static final double[][] kMeasuredDistanceToXStdDevValues = {
            {2.13, 0.006},
            {2.286, 0.008},
            {2.4384, 0.009},
            {2.5908, 0.011},
            {2.7432, 0.015},
            {2.9, 0.017},
            {3.048, 0.015},
            {3.2, 0.015},
            {3.66, 0.022},
            {3.96, 0.041},
            {5.18, 0.046}
        };
        public static final double[][] kMeasuredDistanceToYStdDevValues = {
            {2.13, 0.009},
            {2.286, 0.021},
            {2.4384, 0.022},
            {2.5908, 0.012},
            {2.7432, 0.04},
            {2.9, 0.017},
            {3.048, 0.13},
            {3.2, 0.043},
            {3.66, 0.06},
            {3.96, 0.22},
            {5.18, 0.5}
        };
        // I scaled the std. dev by 10 here because we want to trust our Pigeon values way more
        public static final double[][] kMeasuredDistanceToAngleStdDevValues = {
            {2.13, 0.007},
            {2.286, 0.008},
            {2.4384, 0.015},
            {2.5908, 0.006},
            {2.7432, 0.008},
            {2.9, 0.017},
            {3.048, 0.018},
            {3.2, 0.02}
        };

        public static InterpolatingDoubleTreeMap kVisionXStdDevInterpolater = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap kVisionYStdDevInterpolater = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap kVisionThetaStdDevInterpolater = new InterpolatingDoubleTreeMap();

        static {
            if (kUseInterpolatedVisionStdDevValues) {
                for (double[] pair : kMeasuredDistanceToXStdDevValues) {
                    kVisionXStdDevInterpolater.put(pair[0], pair[1]);
                }

                for (double[] pair : kMeasuredDistanceToYStdDevValues) {
                    kVisionYStdDevInterpolater.put(pair[0], pair[1]);
                }

                for (double[] pair : kMeasuredDistanceToAngleStdDevValues) {
                    kVisionThetaStdDevInterpolater.put(pair[0], pair[1]);
                }
            }
        }

        public static final double kAmbiguityThreshold = 0.15;
        public static final double kAprilTagWidth = Units.inchesToMeters(6.5);
        public static final double kMaxPoseAmbiguity = 0.03;
        public static final double kMaxAreaFitInFrame = 0.0;

        public static final VisionConfig kFrontLeftCameraConfig = new VisionConfig(
                "CubVisionFL",
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-1.75), Units.inchesToMeters(2.32), Units.inchesToMeters(24.5)),
                        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(45))),
                0,
                VisionCameraConfig.kStandardArduCamConfig);

        public static final VisionConfig kFrontRightCameraConfig = new VisionConfig(
                "CubVisionFR",
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-1.75), Units.inchesToMeters(-2.32), Units.inchesToMeters(24.5)),
                        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-45))),
                2,
                VisionCameraConfig.kStandardArduCamConfig);

        public static final VisionConfig kBackLeftCameraConfig = new VisionConfig(
                "CubVisionBL",
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-4.44), Units.inchesToMeters(3.46), Units.inchesToMeters(24.5)),
                        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(135))),
                0,
                VisionCameraConfig.kStandardArduCamConfig);

        public static final VisionConfig kBackRightCameraConfig = new VisionConfig(
                "CubVisionBR",
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-4.44), Units.inchesToMeters(-3.46), Units.inchesToMeters(24.5)),
                        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-135))),
                2,
                VisionCameraConfig.kStandardArduCamConfig);

        public static final VisionConfig kSniperCameraConfig = new VisionConfig(
                "CubVisionSniper",
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-0.47), 0, Units.inchesToMeters(24.5)),
                        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0))),
                0,
                VisionCameraConfig.kSniperCamConfig);

        public static final VisionConfig kLimelightConfig = new VisionConfig(
                "limelight",
                new Transform3d(
                        new Translation3d(0, 0.0, Units.inchesToMeters(24.5)),
                        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-180))),
                0,
                VisionCameraConfig.kLimelightConfig);
    }

    public static final class Controls {
        public static final double kDriverDeadband = 0.09;
    }

    public static final class Motors {
        public static final double kMaxNeoRPM = 5676;
        public static final double kMaxKrakenRPM = 6000;
    }

    public static final class Drive {
        // TODO: update these values for 2024 bot
        protected static final double kL1DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
        protected static final double kL2DriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        protected static final double kL3DriveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        protected static final double k16ToothKitReduction = (16.0 / 14.0);
        protected static final double kMk4SteerReduction = 1.0 / 12.8;
        protected static final double kMk4iSteerReduction = 7.0 / 150.0;

        public static final double kOdometryFrequency = 250.0;
        public static final double kTrackWidthMeters;
        public static final double kWheelbaseMeters;
        public static final double kModuleRadius;
        public static final double kWheelRadiusMeters;
        public static final double kMaxVelocityMetersPerSecond;
        public static final double kMaxAngularVelocityRadiansPerSecond;
        public static final double kMaxSteerVelocityRadiansPerSecond;
        public static final double kMinLockVelocityMetersPerSecond = 0.2;
        public static final boolean kDriveMotorsInverted;
        public static final boolean kSteerMotorsInverted;
        public static final double kDriveReduction;
        public static final double kSteerReduction;

        public static final int kNumModules;
        public static final ExtendedSwerveDriveKinematics kKinematics;
        public static final KinematicLimits kUncappedKinematicLimits;
        public static final KinematicLimits kFastKinematicLimits;
        public static final KinematicLimits kSlowKinematicLimits;
        public static final KinematicLimits kFastTrapezoidalKinematicLimits;
        public static final KinematicLimits kSlowTrapezoidalKinematicLimits;

        public static final LoggedTunableNumber kDriveKff = new LoggedTunableNumber("Drive/Module/DriveKff");
        public static final LoggedTunableNumber kDriveKp = new LoggedTunableNumber("Drive/Module/DriveKp");
        public static final LoggedTunableNumber kDriveKd = new LoggedTunableNumber("Drive/Module/DriveKd");
        public static final LoggedTunableNumber kSteerKp = new LoggedTunableNumber("Drive/Module/SteerKp");
        public static final LoggedTunableNumber kSteerKd = new LoggedTunableNumber("Drive/Module/SteerKd");

        // TODO: determine PID values
        public static final double kPathTranslationKp = 4.0;
        public static final double kPathRotationKp = 2.0;

        public static final HolonomicPathFollowerConfig kPathFollowerConfig;

        static {
            double driveMotorMaxRPM;
            double turnMotorMaxRPM;

            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    kWheelRadiusMeters = Units.inchesToMeters(1.95379394);
                    driveMotorMaxRPM = Constants.Motors.kMaxKrakenRPM;
                    turnMotorMaxRPM = Constants.Motors.kMaxKrakenRPM;
                    kDriveReduction = k16ToothKitReduction * kL3DriveReduction;
                    kSteerReduction = kMk4iSteerReduction;
                    kDriveMotorsInverted = true;
                    kSteerMotorsInverted = true;
                    /* TODO: Update values for 2024 bot */
                    kTrackWidthMeters = 0.465;
                    kWheelbaseMeters = 0.465;
                    kDriveKff.initDefault(0.06);
                    kDriveKp.initDefault(0.06);
                    kDriveKd.initDefault(0);
                    kSteerKp.initDefault(16.0);
                    kSteerKd.initDefault(0);
                    break;
                case SIMULATION_BOT:
                    kWheelRadiusMeters = Units.inchesToMeters(2);
                    driveMotorMaxRPM = Constants.Motors.kMaxKrakenRPM;
                    turnMotorMaxRPM = Constants.Motors.kMaxKrakenRPM;
                    kDriveReduction = kL3DriveReduction * k16ToothKitReduction;
                    kSteerReduction = kMk4iSteerReduction;
                    kDriveMotorsInverted = true;
                    kSteerMotorsInverted = true;
                    /* TODO: Update values for 2024 bot */
                    kTrackWidthMeters = 0.5;
                    kWheelbaseMeters = 0.5;
                    kDriveKff.initDefault(0.1);
                    kDriveKp.initDefault(0.6);
                    kDriveKd.initDefault(0);
                    kSteerKp.initDefault(16.0);
                    kSteerKd.initDefault(0);
                    break;
                default:
                    throw new UnsupportedOperationException("No drive configuration for " + Configuration.getRobot());
            }

            kModuleRadius = Math.hypot(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0);
            kMaxVelocityMetersPerSecond =
                    Units.rotationsPerMinuteToRadiansPerSecond(driveMotorMaxRPM) * kDriveReduction * kWheelRadiusMeters;
            kMaxAngularVelocityRadiansPerSecond =
                    kMaxVelocityMetersPerSecond / Math.hypot(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0);
            kMaxSteerVelocityRadiansPerSecond =
                    Units.rotationsPerMinuteToRadiansPerSecond(turnMotorMaxRPM) * kSteerReduction;

            kKinematics = new ExtendedSwerveDriveKinematics(
                    // Front left
                    new Translation2d(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0),
                    // Front right
                    new Translation2d(kTrackWidthMeters / 2.0, -kWheelbaseMeters / 2.0),
                    // Back left
                    new Translation2d(-kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0),
                    // Back right
                    new Translation2d(-kTrackWidthMeters / 2.0, -kWheelbaseMeters / 2.0));

            kNumModules = kKinematics.getNumModules();
            kUncappedKinematicLimits =
                    new KinematicLimits(kMaxVelocityMetersPerSecond, Double.MAX_VALUE, Double.MAX_VALUE);
            kFastKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond, kMaxVelocityMetersPerSecond / 0.2, Units.degreesToRadians(1000.0));
            kSlowKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * 0.5,
                    kMaxVelocityMetersPerSecond * 0.5 / 0.2,
                    Units.degreesToRadians(750.0));
            kFastTrapezoidalKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * 0.8,
                    kMaxVelocityMetersPerSecond * 0.8 / 1.5,
                    kFastKinematicLimits.maxSteeringVelocity());
            kSlowTrapezoidalKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * 0.4,
                    kMaxVelocityMetersPerSecond * 0.4 / 2.0,
                    kFastKinematicLimits.maxSteeringVelocity());

            kPathFollowerConfig = new HolonomicPathFollowerConfig(
                    new PIDConstants(kPathTranslationKp),
                    new PIDConstants(kPathRotationKp),
                    kMaxVelocityMetersPerSecond * 0.95,
                    kModuleRadius,
                    new ReplanningConfig(),
                    kLoopPeriodSeconds);
        }
    }

    public static final class Shooter {
        // TODO: Update values
        public static final double kRollerReduction = 1.0;
        public static final double kEncoderToShooterReduction = 24.0 / 42.0;
        public static final double kAngleReduction = 1.0 / 105.0;
        public static final int kShooterRightUpperRollerMotorId = 23;
        public static final int kShooterRightLowerRollerMotorId = 25;
        public static final int kShooterLeftLowerRollerMotorId = 24;
        public static final int kShooterLeftUpperRollerMotorId = 22;
        public static final int kShooterRotationMotorId = 26;

        public static final double kShooterUpperLimitRotations = Units.degreesToRotations(110);
        public static final double kShooterLowerLimitRotations = Units.degreesToRotations(18);

        public static final double kShooterAxisHeight = Units.inchesToMeters(7.52);

        public static final LoggedTunableNumber kRollerKff = new LoggedTunableNumber("Shooter/Motor/Roller/Kff");
        public static final LoggedTunableNumber kRollerKp = new LoggedTunableNumber("Shooter/Motor/Roller/Kp");
        public static final LoggedTunableNumber kRollerKd = new LoggedTunableNumber("Shooter/Motor/Roller/Kd");

        public static final LoggedTunableNumber kRotationKff = new LoggedTunableNumber("Shooter/Motor/Rotation/Kff");
        public static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber("Shooter/Motor/Rotation/Kp");
        public static final LoggedTunableNumber kMaxRotationVelocityRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Motor/Rotation/MaxVelocity");
        public static final LoggedTunableNumber kMaxRotationAccelerationRadiansPerSecondSquared =
                new LoggedTunableNumber("Shooter/Motor/Rotation/MaxAcceleration");

        public static final LoggedTunableNumber kIdleSpeedRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Rollers/IdleSpeedRadiansPerSecond", 300);
        public static final LoggedTunableNumber kShooterAmpAngleDegrees =
                new LoggedTunableNumber("Shooter/Rotation/AmpAngleDegrees", 85);
        public static final LoggedTunableNumber kAmpRollerSpeedRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Roller/AmpSpeedRadiansPerSecond", 400);
        public static final LoggedTunableNumber kTrapRollerSpeedRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Roller/TrapRollerSpeedRadiansPerSecond", 200);
        public static final LoggedTunableNumber kTargetShootSpeedRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Roller/TargetShootSpeedRadiansPerSecond", 600);

        public static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber("Shooter/Motor/Rotation/Kd");

        public static final Rotation2d kShooterAngleEncoderOffset;

        public static int kShooterEntranceSensorId;
        public static int kShooterExitSensorId;
        public static int kShooterThroughBoreEncoderId = 4;

        public static double kThroughBoreEncoderDistancePerRotation;

        static {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    kRollerKff.initDefault(0.0);
                    kRollerKp.initDefault(0.0);
                    kRollerKd.initDefault(0.0);

                    kRotationKff.initDefault(0.0);
                    kRotationKp.initDefault(0.0);
                    kRotationKd.initDefault(0.0);

                    kShooterAngleEncoderOffset = Rotation2d.fromRotations(0); // TODO: Update value

                    break;
                case SIMULATION_BOT:
                    kRollerKff.initDefault(0.017);
                    kRollerKp.initDefault(0.1);
                    kRollerKd.initDefault(0.0);

                    kRotationKff.initDefault(0.0);
                    kRotationKp.initDefault(2.0);
                    kRotationKd.initDefault(0.0);

                    kShooterAngleEncoderOffset = Rotation2d.fromRotations(Math.random());

                    break;
                default:
                    throw new UnsupportedOperationException("No shooter configuration for " + Configuration.getRobot());
            }
        }
    }

    public static final class Climb {
        public static final double kWinchReduction = 1.0 / 25.0;

        public static final int kLeftWinchId = 40;
        public static final int kRightWinchId = 41;

        public static final LoggedTunableNumber kWinchKff = new LoggedTunableNumber("Arm/Winch/Kff");
        public static final LoggedTunableNumber kWinchKp = new LoggedTunableNumber("Arm/Winch/Kp");
        public static final LoggedTunableNumber kWinchKd = new LoggedTunableNumber("Arm/Winch/Kd");

        public static final double kWinchCircumference =
                Math.PI * Units.inchesToMeters(1.5); // idk, TODO: update diameter

        static {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    kWinchKff.initDefault(0.0);
                    kWinchKp.initDefault(0.0);
                    kWinchKd.initDefault(0.0);
                    break;
                case SIMULATION_BOT:
                    kWinchKff.initDefault(0.0);
                    kWinchKp.initDefault(2.0);
                    kWinchKd.initDefault(0.0);

                    break;
                default:
            }
        }
    }

    public static final class Arm {
        // TODO: Update values
        public static final double kRotationReduction = 1.0 / 20.0;

        public static final double kArmUpperLimitRotations = Units.degreesToRotations(135);
        public static final double kArmLowerLimitRotations = Units.degreesToRotations(0);

        public static final double kEncoderToArmReduction = 1; // TODO: get value (prob 1 but check)

        // TODO: add IDs

        public static final int kRotationMotorId = 42;
        public static final int kEncoderId = 43;

        public static final LoggedTunableNumber kArmRotationKff = new LoggedTunableNumber("Arm/RotationMotor/Kff");
        public static final LoggedTunableNumber kArmRotationKp = new LoggedTunableNumber("Arm/RotationMotor/Kp");
        public static final LoggedTunableNumber kArmRotationKd = new LoggedTunableNumber("Arm/RotationMotor/Kd");

        // TODO: update values
        public static final LoggedTunableNumber kMaxRotationVelocityRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Motor/Rotation/MaxVelocity");
        public static final LoggedTunableNumber kMaxRotationAccelerationRadiansPerSecondSquared =
                new LoggedTunableNumber("Shooter/Motor/Rotation/MaxAcceleration");

        public static final Rotation2d kArmAngleEncoderOffset;

        public static final LoggedTunableNumber kArmAmpRotationDegrees =
                new LoggedTunableNumber("Arm/AmpRotationDegrees");
        ; // TODO: update

        static {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    kArmRotationKff.initDefault(0.0);
                    kArmRotationKp.initDefault(0.0);
                    kArmRotationKd.initDefault(0.0);

                    kArmAngleEncoderOffset = Rotation2d.fromRotations(0); // TODO: Update value
                    break;
                case SIMULATION_BOT:
                    kArmRotationKff.initDefault(0.0);
                    kArmRotationKp.initDefault(0.0);
                    kArmRotationKd.initDefault(0.0);

                    kArmAngleEncoderOffset = Rotation2d.fromRotations(Math.random());
                    break;
                default:
                    kArmAngleEncoderOffset = Rotation2d.fromRotations(0);
            }
        }
    }

    public static final class Indexer {
        // TODO: Update values and set names
        public static final int kIndexerMotorId = 21;
        public static final double kIndexerReduction = 1;

        public static final int kIndexerEntranceSensorId = 1;
        public static final int kIndexerExitSensorId = 3;

        public static final double kIndexerLoadPercent = .25;
        public static final double kIndexerFeedPercent = 1;
        public static final double kReduction = 1.0 / 1.0;

        public static final LoggedTunableNumber kIndexerKff = new LoggedTunableNumber("Indexer/Motor/Kff");
        public static final LoggedTunableNumber kIndexerKp = new LoggedTunableNumber("Indexer/Motor/Kp");
        public static final LoggedTunableNumber kIndexerKd = new LoggedTunableNumber("Indexer/Motor/Kd");
        public static double kIntakeReduction;
    }

    public class Intake {
        public static final int kIntakeMotorId = 20;

        // TODO: Add sensor Ids
        public static final double kIntakeSpeed = 0.5;
        public static final double kOuttakeSpeed = -0.5;
        public static final int kIntakeEntranceSensorId = 4;
        public static final int kIntakeExitSensorId = 5;
        public static final double kReduction = 1.0 / 9.0;
    }
}
