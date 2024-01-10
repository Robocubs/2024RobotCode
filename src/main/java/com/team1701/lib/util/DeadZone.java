package com.team1701.lib.util;

public class DeadZone {
    private double minimumValue;
    private double maximumValue;

    public DeadZone(double value) {
        minimumValue = -value;
        maximumValue = value;
    }

    public DeadZone(double minimumValue, double maximumValue) {
        this.minimumValue = -minimumValue;
        this.maximumValue = maximumValue;
    }

    public double apply(double value) {
        if (value > maximumValue) {
            return (value - maximumValue) / (1 - maximumValue);
        }

        if (value < minimumValue) {
            return (value - minimumValue) / (1 + minimumValue);
        }

        return 0;
    }
}
