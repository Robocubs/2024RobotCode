package com.team1701.lib.drivers.cameras.apriltag;

import com.team1701.lib.drivers.cameras.apriltag.proto.AprilTagTargetProto;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

public class AprilTagTarget implements ProtobufSerializable {
    public final int id;
    public final Transform3d bestCameraToTarget;
    public final Transform3d altCameraToTarget;
    public final double ambiguity;

    public AprilTagTarget(int id, Transform3d bestCameraToTarget, Transform3d altCameraToTarget, double ambiguity) {
        this.id = id;
        this.bestCameraToTarget = bestCameraToTarget;
        this.altCameraToTarget = altCameraToTarget;
        this.ambiguity = ambiguity;
    }

    public static final AprilTagTargetProto proto = new AprilTagTargetProto();
}
