package com.team1701.lib.drivers.cameras.neural;

import com.team1701.lib.drivers.cameras.neural.proto.DetectedObjectProto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

public class DetectedObject implements ProtobufSerializable {
    public final String detectedClass;
    public final int detectedClassId;
    public final double confidence;
    public final double area;
    public final Rotation2d pitch;
    public final Rotation2d yaw;
    public final Translation2d positionPixels;

    public DetectedObject(
            String detectedClass,
            int detectedClassId,
            double confidence,
            double area,
            Rotation2d pitch,
            Rotation2d yaw,
            Translation2d positionPixels) {
        this.detectedClass = detectedClass;
        this.detectedClassId = detectedClassId;
        this.confidence = confidence;
        this.area = area;
        this.pitch = pitch;
        this.yaw = yaw;
        this.positionPixels = positionPixels;
    }

    public static final DetectedObjectProto proto = new DetectedObjectProto();
}
