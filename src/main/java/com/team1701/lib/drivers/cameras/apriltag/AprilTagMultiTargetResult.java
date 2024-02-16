package com.team1701.lib.drivers.cameras.apriltag;

import com.team1701.lib.drivers.cameras.apriltag.proto.AprilTagMultiTargetResultProto;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

public class AprilTagMultiTargetResult implements ProtobufSerializable {
    public final int[] targetIds;
    public final Pose3d cameraPose;
    public final double reprojectionError;

    public AprilTagMultiTargetResult(int[] targetIds, Pose3d cameraPose, double reprojectionError) {
        this.targetIds = targetIds;
        this.cameraPose = cameraPose;
        this.reprojectionError = reprojectionError;
    }

    public static final AprilTagMultiTargetResultProto proto = new AprilTagMultiTargetResultProto();
}
