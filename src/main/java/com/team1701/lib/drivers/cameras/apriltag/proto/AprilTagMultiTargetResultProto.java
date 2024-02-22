package com.team1701.lib.drivers.cameras.apriltag.proto;

import java.util.Arrays;

import com.team1701.lib.drivers.cameras.apriltag.AprilTagMultiTargetResult;
import com.team1701.proto.Apriltag.ProtobufAprilTagMultiTargetResult;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class AprilTagMultiTargetResultProto
        implements Protobuf<AprilTagMultiTargetResult, ProtobufAprilTagMultiTargetResult> {

    @Override
    public Class<AprilTagMultiTargetResult> getTypeClass() {
        return AprilTagMultiTargetResult.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufAprilTagMultiTargetResult.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {Pose3d.proto};
    }

    @Override
    public ProtobufAprilTagMultiTargetResult createMessage() {
        return ProtobufAprilTagMultiTargetResult.newInstance();
    }

    @Override
    public AprilTagMultiTargetResult unpack(ProtobufAprilTagMultiTargetResult msg) {
        var msgTargetIds = msg.getTargetIds();
        var targetIds = new int[msgTargetIds.length()];
        Arrays.setAll(targetIds, i -> msgTargetIds.get(i));

        var pose = Pose3d.proto.unpack(msg.getCameraPose());
        var reprojectionError = msg.getReprojectionError();

        return new AprilTagMultiTargetResult(targetIds, pose, reprojectionError);
    }

    @Override
    public void pack(ProtobufAprilTagMultiTargetResult msg, AprilTagMultiTargetResult value) {
        var targetIds = msg.getMutableTargetIds();
        for (var id : value.targetIds) {
            targetIds.add(id);
        }

        Pose3d.proto.pack(msg.getMutableCameraPose(), value.cameraPose);
        msg.setReprojectionError(value.reprojectionError);
    }
}
