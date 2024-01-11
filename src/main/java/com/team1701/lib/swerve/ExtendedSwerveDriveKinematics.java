package com.team1701.lib.swerve;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ejml.simple.SimpleMatrix;

public class ExtendedSwerveDriveKinematics extends SwerveDriveKinematics {

    private final int mNumModules;
    private final Translation2d[] mModules;
    private final Rotation2d[] mRotations;

    public ExtendedSwerveDriveKinematics(Translation2d... moduleLocations) {
        super(moduleLocations);
        mNumModules = moduleLocations.length;
        mModules = Arrays.copyOf(moduleLocations, mNumModules);
        mRotations = new Rotation2d[mNumModules];

        for (var i = 0; i < mNumModules; i++) {
            mRotations[i] = new Rotation2d(mModules[i].getX(), mModules[i].getY());
        }
    }

    public ChassisSpeeds toChassisSpeedWheelConstraints(SwerveModuleState... wheelStates) {
        if (wheelStates.length != mNumModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in constructor");
        }

        var constraintsMatrix = new SimpleMatrix(mNumModules * 2, 3);
        for (var i = 0; i < mNumModules; i++) {
            var module = wheelStates[i];
            var beta =
                    module.angle.rotateBy(mRotations[i].unaryMinus()).rotateBy(Rotation2d.fromRadians(Math.PI / 2.0));

            constraintsMatrix.setRow(
                    i * 2, 0, module.angle.getCos(), module.angle.getSin(), -mModules[i].getNorm() * beta.getCos());
            constraintsMatrix.setRow(
                    i * 2 + 1, 0, -module.angle.getSin(), module.angle.getCos(), mModules[i].getNorm() * beta.getSin());
        }

        var enforcedConstraints = new SimpleMatrix(mNumModules * 2, 1);
        for (var i = 0; i < mNumModules; i++) {
            enforcedConstraints.setRow(i * 2, 0, wheelStates[i].speedMetersPerSecond);
            enforcedConstraints.setRow(i * 2 + 1, 0, 0);
        }

        var chassisSpeedsVector = constraintsMatrix.pseudoInverse().mult(enforcedConstraints);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0), chassisSpeedsVector.get(1, 0), chassisSpeedsVector.get(2, 0));
    }

    public int getNumModules() {
        return mNumModules;
    }
}
