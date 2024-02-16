package com.team1701.lib.drivers.cameras.apriltag.proto;

import com.team1701.lib.drivers.cameras.apriltag.AprilTagTarget;
import com.team1701.proto.Apriltag.ProtobufAprilTagTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;
import us.hebi.quickbuf.RepeatedMessage;

public class AprilTagTargetProto implements Protobuf<AprilTagTarget, ProtobufAprilTagTarget> {
    @Override
    public Class<AprilTagTarget> getTypeClass() {
        return AprilTagTarget.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufAprilTagTarget.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {Transform3d.proto};
    }

    @Override
    public ProtobufAprilTagTarget createMessage() {
        return ProtobufAprilTagTarget.newInstance();
    }

    @Override
    public AprilTagTarget unpack(ProtobufAprilTagTarget msg) {
        return new AprilTagTarget(
                msg.getId(),
                Transform3d.proto.unpack(msg.getBestCameraToTarget()),
                Transform3d.proto.unpack(msg.getAltCameraToTarget()),
                msg.getAmbiguity());
    }

    public AprilTagTarget[] unpack(RepeatedMessage<ProtobufAprilTagTarget> msg) {
        var targets = new AprilTagTarget[msg.length()];
        for (int i = 0; i < targets.length; i++) {
            targets[i] = unpack(msg.get(i));
        }
        return targets;
    }

    @Override
    public void pack(ProtobufAprilTagTarget msg, AprilTagTarget value) {
        msg.setId(value.id);
        Transform3d.proto.pack(msg.getMutableBestCameraToTarget(), value.bestCameraToTarget);
        Transform3d.proto.pack(msg.getMutableAltCameraToTarget(), value.altCameraToTarget);
        msg.setAmbiguity(value.ambiguity);
    }

    public void pack(RepeatedMessage<ProtobufAprilTagTarget> msg, AprilTagTarget[] values) {
        var targets = msg.reserve(values.length);
        for (var target : values) {
            pack(targets.next(), target);
        }
    }
}
