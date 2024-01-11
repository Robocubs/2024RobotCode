package com.team1701.lib.swerve;

import java.util.Arrays;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public ChassisSpeeds chassisSpeeds;
    public SwerveModuleState[] moduleStates;

    public SwerveSetpoint(int moduleCount) {
        this.chassisSpeeds = new ChassisSpeeds();
        this.moduleStates = new SwerveModuleState[moduleCount];
        Arrays.setAll(moduleStates, i -> new SwerveModuleState());
    }

    public SwerveSetpoint(ExtendedSwerveDriveKinematics kinematics, SwerveModuleState[] initialStates) {
        this.chassisSpeeds = kinematics.toChassisSpeedWheelConstraints(initialStates);
        this.moduleStates = initialStates;
    }

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.chassisSpeeds = chassisSpeeds;
        this.moduleStates = initialStates;
    }

    @Override
    public String toString() {
        var ret = chassisSpeeds.toString() + "\n";
        for (var i = 0; i < moduleStates.length; ++i) {
            ret += "  " + moduleStates[i].toString() + "\n";
        }
        return ret;
    }
}
