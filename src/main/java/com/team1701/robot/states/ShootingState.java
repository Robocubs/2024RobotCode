package com.team1701.robot.states;

public class ShootingState {
    public final boolean isActive;
    public final boolean atHeading;
    public final boolean atAngle;
    public final boolean atSpeed;
    public final boolean isShooting;

    public ShootingState() {
        this(false, false, false, false, false);
    }

    public ShootingState(boolean isActive, boolean atAngle, boolean atSpeed, boolean atHeading, boolean isShooting) {
        this.isActive = isActive;
        this.atHeading = atHeading;
        this.atAngle = atAngle;
        this.atSpeed = atSpeed;
        this.isShooting = isShooting;
    }

    public boolean canShoot() {
        return atHeading && atAngle && atSpeed;
    }
}
