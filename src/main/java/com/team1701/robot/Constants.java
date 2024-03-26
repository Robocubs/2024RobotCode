package com.team1701.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team1701.lib.drivers.cameras.config.VisionCameraConfig;
import com.team1701.lib.drivers.cameras.config.VisionConfig;
import com.team1701.lib.swerve.ExtendedSwerveDriveKinematics;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.PolynomialRegression;
import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double kLoopPeriodSeconds = 0.02;

    public static final class Robot {
        public static final double kRobotWidth = Units.inchesToMeters(23);
        public static final double kRobotLength = Units.inchesToMeters(28.5);
        public static final double kRobotWidthWithBumpers = kRobotWidth + Units.inchesToMeters(8);
        public static final double kRobotLengthWithBumpers = kRobotLength + Units.inchesToMeters(8);
        public static final double kRobotFrontToCenter = Units.inchesToMeters(23.0 / 2.0);
        public static final double kRobotBackToCenter = kRobotLength - kRobotFrontToCenter;
        public static final double kRobotSideToCenter = kRobotWidth / 2.0;
        public static final double kRobotFrontToCenterWithBumpers = kRobotWidthWithBumpers / 2.0;

        public static final Transform2d kRobotToIntake =
                new Transform2d(-kRobotWidth / 2, 0.0, GeometryUtil.kRotationIdentity);
        public static final Transform2d kIntakeToRobot = kRobotToIntake.inverse();
        public static final Transform3d kRobotToShooterHinge = new Transform3d(
                new Translation3d(Units.inchesToMeters(-3), Units.inchesToMeters(0), Units.inchesToMeters(7.52)),
                GeometryUtil.kRotation3dIdentity);
        public static final Transform3d kShooterHingeToShooterExit = new Transform3d(
                new Translation3d(Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(1.9)),
                GeometryUtil.kRotation3dIdentity);
        public static final double kLongDistanceFromDriveCenterToCorner =
                Math.hypot(kRobotWidthWithBumpers / 2.0, kRobotLength - (kRobotWidth / 2.0));
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
                double scalar = 1.5;
                for (double[] pair : kMeasuredDistanceToXStdDevValues) {
                    kVisionXStdDevInterpolater.put(pair[0], pair[1] * scalar);
                }

                for (double[] pair : kMeasuredDistanceToYStdDevValues) {
                    kVisionYStdDevInterpolater.put(pair[0], pair[1] * scalar);
                }

                for (double[] pair : kMeasuredDistanceToAngleStdDevValues) {
                    kVisionThetaStdDevInterpolater.put(pair[0], pair[1] * scalar);
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
                        new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(-180))),
                0,
                VisionCameraConfig.kLimelightConfig);
    }

    public static final class Controls {
        public static final double kDriverDeadband =
                Configuration.getMode() == Configuration.Mode.SIMULATION ? 0.15 : 0.09;
    }

    public static final class Motors {
        public static final double kMaxNeoRPM = 5676;
        public static final double kMaxKrakenRPM = 6000;
    }

    public static final class Drive {
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
        public static final KinematicLimits kFastSmoothKinematicLimits;
        public static final KinematicLimits kShootMoveKinematicLimits;
        public static final KinematicLimits kMediumTrapezoidalKinematicLimits;

        public static final LoggedTunableNumber kDriveKs = new LoggedTunableNumber("Drive/Module/DriveKs");
        public static final LoggedTunableNumber kDriveKv = new LoggedTunableNumber("Drive/Module/DriveKv");
        public static final LoggedTunableNumber kDriveKa = new LoggedTunableNumber("Drive/Module/DriveKa");
        public static final LoggedTunableNumber kDriveKp = new LoggedTunableNumber("Drive/Module/DriveKp");
        public static final LoggedTunableNumber kDriveKd = new LoggedTunableNumber("Drive/Module/DriveKd");
        public static final LoggedTunableNumber kSteerKp = new LoggedTunableNumber("Drive/Module/SteerKp");
        public static final LoggedTunableNumber kSteerKd = new LoggedTunableNumber("Drive/Module/SteerKd");

        public static final double kPathTranslationKp = 4.0;
        public static final double kPathRotationKp = 2.0;

        public static final HolonomicPathFollowerConfig kPathFollowerConfig;

        static {
            double driveMotorMaxRPM;
            double turnMotorMaxRPM;

            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    kWheelRadiusMeters = Units.inchesToMeters(1.95379394);
                    driveMotorMaxRPM = 5500; // Measured + 10%
                    turnMotorMaxRPM = Constants.Motors.kMaxKrakenRPM;
                    kDriveReduction = k16ToothKitReduction * kL3DriveReduction;
                    kSteerReduction = kMk4iSteerReduction;
                    kDriveMotorsInverted = true;
                    kSteerMotorsInverted = true;
                    kTrackWidthMeters = 0.451;
                    kWheelbaseMeters = 0.451;
                    kDriveKs.initDefault(3.78855);
                    kDriveKv.initDefault(0.11668); // 0.06
                    kDriveKa.initDefault(1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp);
                    kDriveKp.initDefault(35);
                    kDriveKd.initDefault(0);
                    kSteerKp.initDefault(4000); // 16.0
                    kSteerKd.initDefault(50);
                    break;
                case SIMULATION_BOT:
                    kWheelRadiusMeters = Units.inchesToMeters(2);
                    driveMotorMaxRPM = Constants.Motors.kMaxKrakenRPM;
                    turnMotorMaxRPM = Constants.Motors.kMaxKrakenRPM;
                    kDriveReduction = kL3DriveReduction * k16ToothKitReduction;
                    kSteerReduction = kMk4iSteerReduction;
                    kDriveMotorsInverted = true;
                    kSteerMotorsInverted = true;
                    kTrackWidthMeters = 0.5;
                    kWheelbaseMeters = 0.5;
                    kDriveKs.initDefault(0.32651);
                    kDriveKv.initDefault(0.1);
                    kDriveKa.initDefault(0);
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
            kShootMoveKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * .8,
                    kMaxVelocityMetersPerSecond / 0.2,
                    kFastKinematicLimits.maxSteeringVelocity());
            kFastSmoothKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond / 2.0,
                    kMaxVelocityMetersPerSecond / 0.4,
                    kFastKinematicLimits.maxSteeringVelocity());
            kSlowKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * 0.5,
                    kMaxVelocityMetersPerSecond * 0.5 / 0.2,
                    Units.degreesToRadians(750.0));
            kFastTrapezoidalKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * 0.8,
                    kMaxVelocityMetersPerSecond / 1.0,
                    kFastKinematicLimits.maxSteeringVelocity());
            kMediumTrapezoidalKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * 0.8,
                    kMaxVelocityMetersPerSecond / 1.5,
                    kFastKinematicLimits.maxSteeringVelocity());
            kSlowTrapezoidalKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * 0.4,
                    kMaxVelocityMetersPerSecond / 2.0,
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
        public static final double kRollerReduction = 1.0 / 1.0; // 32.0 /18.0
        public static final double kEncoderToShooterReduction = 30.0 / 50.0;
        public static final double kAngleReduction = (1.0 / 4.0) * (1.0 / 5.0) * (20.0 / 93.0);

        public static final int kShooterRightUpperRollerMotorId = 25;
        public static final int kShooterRightLowerRollerMotorId = 23;
        public static final int kShooterLeftLowerRollerMotorId = 22;
        public static final int kShooterLeftUpperRollerMotorId = 24;

        public static final int kShooterRotationMotorId = 26;

        public static final Rotation2d kShooterUpperLimit = Rotation2d.fromDegrees(58);
        public static final Rotation2d kShooterLowerLimit = Rotation2d.fromDegrees(16);

        public static final Rotation2d kPassingHeadingTolerance = Rotation2d.fromRadians(0.1);

        public static final double kShooterAxisHeight = Units.inchesToMeters(7.52);
        public static final Rotation2d kShooterReleaseAngle = Rotation2d.fromDegrees(-2);
        public static final double kSpeakerToShooterHingeDifference =
                FieldConstants.kSpeakerHeight - kShooterAxisHeight;
        public static final LoggedTunableNumber kShooterReleaseAngleDegrees =
                new LoggedTunableNumber("Shooter/kShooterReleaseAngleDegrees", -7);

        public static final Rotation2d kShooterAngleEncoderOffset;

        public static final int kShooterThroughBoreEncoderId = 4;

        public static final LoggedTunableNumber kUpperRollerKs =
                new LoggedTunableNumber("Shooter/Motor/UpperRoller/Ks");
        public static final LoggedTunableNumber kUpperRollerKv =
                new LoggedTunableNumber("Shooter/Motor/UpperRoller/Kv");
        public static final LoggedTunableNumber kLowerRollerKs =
                new LoggedTunableNumber("Shooter/Motor/LowerRoller/Ks");
        public static final LoggedTunableNumber kLowerRollerKv =
                new LoggedTunableNumber("Shooter/Motor/LowerRoller/Kv");
        public static final LoggedTunableNumber kRollerKp = new LoggedTunableNumber("Shooter/Motor/Roller/Kp");
        public static final LoggedTunableNumber kRollerKd = new LoggedTunableNumber("Shooter/Motor/Roller/Kd");

        public static final LoggedTunableNumber kMaxAngleDegrees =
                new LoggedTunableNumber("Shooter/Motor/Rotation/MaxAngleDegrees", 58);

        public static final LoggedTunableNumber kMaxRotationVelocityRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Motor/Rotation/MaxVelocity", 1.0);
        public static final LoggedTunableNumber kMaxRotationAccelerationRadiansPerSecondSquared =
                new LoggedTunableNumber("Shooter/Motor/Rotation/MaxAcceleration", 1.0);

        public static final Rotation2d kLoadingAngle = Rotation2d.fromDegrees(28);
        public static final LoggedTunableNumber kIdleSpeedRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Roller/IdleSpeedRadiansPerSecond", 200);
        public static final LoggedTunableNumber kShooterAmpAngleDegrees =
                new LoggedTunableNumber("Shooter/Rotation/AmpAngleDegrees", 55);
        public static final LoggedTunableNumber kAmpRollerSpeedRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Roller/AmpSpeedRadiansPerSecond", 150);
        public static final LoggedTunableNumber kTrapRollerSpeedRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Roller/TrapRollerSpeedRadiansPerSecond", 200);
        public static final LoggedTunableNumber kTunableShooterSpeedRadiansPerSecond =
                new LoggedTunableNumber("Shooter/Roller/TunableSpeedRadiansPerSecond", 300);

        public static final LoggedTunableNumber kTunableShooterAngleRadians =
                new LoggedTunableNumber("Shooter/Rotation/TunableAngleRadians", 1.0);

        public static final LoggedTunableNumber kLowerAmpSpeed =
                new LoggedTunableNumber("Shooter/Roller/Amp/LowerRollerSpeed", 95);

        public static final LoggedTunableNumber kUpperAmpSpeed =
                new LoggedTunableNumber("Shooter/Roller/Amp/UpperRollerSpeed", 60);

        public static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber("Shooter/Motor/Rotation/Kp");
        public static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber("Shooter/Motor/Rotation/Kd");

        public static final InterpolatingDoubleTreeMap kShooterAngleInterpolator =
                new InterpolatingDoubleTreeMap(); // Radians
        public static final InterpolatingDoubleTreeMap kShooterSpeedInterpolator =
                new InterpolatingDoubleTreeMap(); // Radians/sec

        public static final InterpolatingDoubleTreeMap kPassingAngleInterpolator =
                new InterpolatingDoubleTreeMap(); // Radians
        public static final InterpolatingDoubleTreeMap kPassingSpeedInterpolator =
                new InterpolatingDoubleTreeMap(); // Radians/sec

        public static final InterpolatingDoubleTreeMap kShooterHeadingOffsetInterpolator =
                new InterpolatingDoubleTreeMap();

        public static final double kRollerRampRate = 450;

        public static final boolean kUseNewCurves = true;

        // public static final double[][] kShooterDistanceToAngleValues = {
        //     {3.47, 0.535},
        //     {2.75, 0.7},
        //     {2.298, 0.9},
        //     {4.25, 0.49},
        //     {4.75, 0.442},
        //     {5.5, 0.385},
        //     {6.46, .3785}
        // };

        // public static final double[][] kShooterDistanceToSpeedValues = {
        //     {2.3, 200},
        //     {3.5, 400},
        //     {3.78, 400},
        //     {2.75, 300},
        //     {2.298, 300},
        //     {4.25, 410},
        //     {4.75, 410},
        //     {5.5, 480},
        //     {6.46, 520}
        // };

        public static final double[][] kShooterDistanceToAngleValues = {
            // {2.3, 1.01},
            // {2.75, 0.83},
            // {3.125, .715},
            // {3.5, 0.62},
            // {3.78, 0.56},
            // {4.25, 0.5},
            // {4.89, 0.47},
            // {5.49, 0.43},
            // {6, 0.4},
            // {6.4, 0.36},
            // {8.3, 0.31},
            // delete above
            {2.3, 1},
            {2.7, .8},
            {3.5, .63},
            {3.8, .58},
            {4.1, .55},
            {4.7, .50},
            {5.1, .47},
            {5.9, .44} // -9
        };

        public static final double[][] kShooterDistanceToSpeedValues = {
            // {2.3, 275},
            // {2.75, 320},
            // {3.5, 385},
            // {3.78, 425},
            // {4.25, 450},
            // {4.89, 500},
            // {5.49, 550},
            // {6, 600},
            // {6.4, 620},
            // {8.3, 660},
            // delete above
            {2.3, 410}, // -7
            {2.7, 410}, // -9
            {3.5, 440}, // -9
            {3.8, 460}, // -9
            {4.1, 470}, // -9
            {4.7, 510}, // -9
            {5.1, 550}, // -5
            {5.9, 590} // -5
        };

        public static final double[][] kShooterDistanceToHeadingOffset = {
            {460, -9},
            {540, -5}
        };

        public static final double[][] kPassingDistanceToAngleValues = {
            {11.53, .7}, {10.3, .75}, {9.02, .8}, {7.15, .9}, {5, .9}, {0, 1}
        };

        public static final double[][] kPassingDistanceToSpeedValues = {
            {11.53, 325}, {10.3, 315}, {9.02, 300}, {7.15, 275}, {5, 200}, {0, 100}
        };

        // Regression of Collected (a.k.a used angle) vs Calculated Angle
        public static final PolynomialRegression kAngleRegression;
        // Regression of Speed vs Distance to speaker
        public static final PolynomialRegression kSpeedRegression;

        static {
            for (double[] pair : kShooterDistanceToAngleValues) {
                kShooterAngleInterpolator.put(pair[0], pair[1]);
            }

            for (double[] pair : kShooterDistanceToSpeedValues) {
                kShooterSpeedInterpolator.put(pair[0], pair[1]);
            }

            for (double[] pair : kPassingDistanceToAngleValues) {
                kPassingAngleInterpolator.put(pair[0], pair[1]);
            }

            for (double[] pair : kPassingDistanceToSpeedValues) {
                kPassingSpeedInterpolator.put(pair[0], pair[1]);
            }

            if (kUseNewCurves) {
                // Angle
                var collectedDistanceToAngles = kShooterDistanceToAngleValues;
                double[] theoreticalAngles = new double[collectedDistanceToAngles.length];
                double[] collectedAngles = new double[collectedDistanceToAngles.length];
                for (int i = 0; i < collectedDistanceToAngles.length; ++i) {
                    var d = collectedDistanceToAngles[i][0];
                    theoreticalAngles[i] = ShooterUtil.calculateTheoreticalAngle(d);
                    collectedAngles[i] = collectedDistanceToAngles[i][1];
                }

                kAngleRegression = new PolynomialRegression(theoreticalAngles, collectedAngles, 1);

                // Speed
                var collectedDistanceToSpeeds = kShooterDistanceToSpeedValues;
                double[] distances = new double[collectedDistanceToSpeeds.length];
                double[] collectedSpeeds = new double[collectedDistanceToSpeeds.length];
                for (int i = 0; i < collectedDistanceToSpeeds.length; ++i) {
                    distances[i] = collectedDistanceToSpeeds[i][0];
                    collectedSpeeds[i] = collectedDistanceToSpeeds[i][1];
                }

                kSpeedRegression = new PolynomialRegression(distances, collectedSpeeds, 2);
            }
            for (double[] pair : kShooterDistanceToHeadingOffset) {
                kShooterHeadingOffsetInterpolator.put(pair[0], Units.degreesToRadians(pair[1]));
            }
        }

        static {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    kUpperRollerKs.initDefault(0.14414);
                    kUpperRollerKv.initDefault(0.01684);
                    kLowerRollerKs.initDefault(0.13441);
                    kLowerRollerKv.initDefault(0.01675);
                    kRollerKp.initDefault(0.0002);
                    kRollerKd.initDefault(0.0);

                    kRotationKp.initDefault(0.3);
                    kRotationKd.initDefault(0.0);

                    kTunableShooterSpeedRadiansPerSecond.initDefault(450);

                    kShooterAngleEncoderOffset = Rotation2d.fromRadians(1.011)
                            .plus(Rotation2d.fromDegrees(12.3 / kEncoderToShooterReduction)); // hard stop is 12.3º

                    kMaxRotationVelocityRadiansPerSecond.initDefault(0);
                    kMaxRotationAccelerationRadiansPerSecondSquared.initDefault(0);

                    break;
                case SIMULATION_BOT:
                    kUpperRollerKs.initDefault(0.8548);
                    kUpperRollerKv.initDefault(0.01576);
                    kLowerRollerKs.initDefault(0.8548);
                    kLowerRollerKv.initDefault(0.01576);
                    kRollerKp.initDefault(0.2);
                    kRollerKd.initDefault(0.0);

                    kRotationKp.initDefault(50.0);
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

        public static final Rotation2d kMaxSetpoint = Rotation2d.fromRadians(32);
        public static final Rotation2d kMiddleSetpoint = Rotation2d.fromRadians(25);

        public static final double upperLimitRotations = Units.radiansToRotations(32) / kWinchReduction;
        public static final double lowerLimitRotations = Units.radiansToRotations(-2) / kWinchReduction;

        public static final LoggedTunableNumber kWinchKp = new LoggedTunableNumber("Climb/Winch/Kp");
        public static final LoggedTunableNumber kWinchKd = new LoggedTunableNumber("Climb/Winch/Kd");

        public static final double kWinchCircumference = Math.PI * Units.inchesToMeters(1.5);

        static {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    kWinchKp.initDefault(0.1);
                    kWinchKd.initDefault(0.0);
                    break;
                case SIMULATION_BOT:
                    kWinchKp.initDefault(2.0);
                    kWinchKd.initDefault(0.0);
                    break;
                default:
            }
        }
    }

    public static final class Arm {
        public static final double kRotationReduction = 1.0 / 20.0;

        public static final double kArmUpperLimitRotations = Units.degreesToRotations(95) / kRotationReduction;
        public static final double kArmLowerLimitRotations = Units.degreesToRotations(20) / kRotationReduction;

        public static final double kAngleReduction = 1;

        public static final int kRotationMotorId = 42;
        public static final int kEncoderId = 5;

        public static final LoggedTunableNumber kArmRotationKp = new LoggedTunableNumber("Arm/RotationMotor/Kp");
        public static final LoggedTunableNumber kArmRotationKd = new LoggedTunableNumber("Arm/RotationMotor/Kd");

        public static final LoggedTunableNumber kMaxRotationVelocityRadiansPerSecond =
                new LoggedTunableNumber("Arm/Motor/Rotation/MaxVelocity", Math.PI);
        public static final LoggedTunableNumber kMaxRotationAccelerationRadiansPerSecondSquared =
                new LoggedTunableNumber("Arm/Motor/Rotation/MaxAcceleration", Math.PI);

        public static final Rotation2d kArmAngleEncoderOffset;

        public static final LoggedTunableNumber kArmAmpRotationDegrees =
                new LoggedTunableNumber("Arm/AmpRotationDegrees", 95);

        static {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    kArmRotationKp.initDefault(0.2);
                    kArmRotationKd.initDefault(0.0);

                    kArmAngleEncoderOffset = Rotation2d.fromRadians(-0.6).plus(GeometryUtil.kRotationHalfPi);
                    break;
                case SIMULATION_BOT:
                    kArmRotationKp.initDefault(0.2);
                    kArmRotationKd.initDefault(0.0);

                    kArmAngleEncoderOffset = Rotation2d.fromRotations(Math.random());
                    break;
                default:
                    kArmAngleEncoderOffset = Rotation2d.fromRotations(0);
            }
        }
    }

    public static final class Indexer {
        public static final int kIndexerMotorId = 21;
        public static final double kIndexerReduction = 1.0 / 9.0;

        public static final int kIndexerEntranceSensorId = 0;
        public static final int kIndexerExitSensorId = 2;

        public static final double kIndexerShootPercent = 1;
        public static final double kIndexerLoadPercent = 0.8;
        public static final double kIndexerSlowPercent = 0.5;
        public static final double kIndexerReversePercent = -1;
        public static final double kIndexerReverseBumpPercent = -0.2;
    }

    public class Intake {
        public static final int kIntakeMotorId = 20;

        public static final int kIntakeEntranceSensorId = 3;
        public static final int kIntakeExitSensorId = 1;

        public static final double kIntakeSpeed = 0.8;
        public static final double kOuttakeSpeed = -0.75;
        public static final double kReduction = 1.0 / 9.0;
    }
}
