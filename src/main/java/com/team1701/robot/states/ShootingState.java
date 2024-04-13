package com.team1701.robot.states;

import com.team1701.robot.subsystems.shooter.Shooter.ShooterSetpoint;
import org.littletonrobotics.junction.Logger;

public class ShootingState {
    public static final ShootingState kDefault =
            new ShootingState(ShooterSetpoint.kDefault, false, false, false, false, false, false);

    public final ShooterSetpoint setpoint;
    public final boolean isActive;
    public final boolean atHeading;
    public final boolean atAngle;
    public final boolean atSpeed;
    public final boolean canShootWhileMove;
    public final boolean isShooting;

    public ShootingState(
            ShooterSetpoint setpoint,
            boolean isActive,
            boolean atAngle,
            boolean atSpeed,
            boolean atHeading,
            boolean canShootWhileMove,
            boolean isShooting) {
        this.setpoint = setpoint;
        this.isActive = isActive;
        this.atHeading = atHeading;
        this.atAngle = atAngle;
        this.atSpeed = atSpeed;
        this.canShootWhileMove = canShootWhileMove;
        this.isShooting = isShooting;
    }

    public boolean canShoot() {
        return atHeading && atAngle && atSpeed;
    }

    public void log(String prefix) {
        Logger.recordOutput(prefix + "/Active", isActive);
        Logger.recordOutput(prefix + "/AtHeading", atHeading);
        Logger.recordOutput(prefix + "/AtAngle", atAngle);
        Logger.recordOutput(prefix + "/AtSpeed", atSpeed);
        Logger.recordOutput(prefix + "/Shooting", isShooting);
    }
}
