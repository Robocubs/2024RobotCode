package com.team1701.lib.drivers.cameras.apriltag.proto;

import java.util.Optional;

import com.team1701.lib.drivers.cameras.apriltag.AprilTagMultiTargetResult;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagPipelineResult;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagTarget;
import com.team1701.proto.Apriltag.ProtobufAprilTagPipelineResult;
import edu.wpi.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;
import us.hebi.quickbuf.RepeatedMessage;

public class AprilTagPipelineResultProto implements Protobuf<AprilTagPipelineResult, ProtobufAprilTagPipelineResult> {
    @Override
    public Class<AprilTagPipelineResult> getTypeClass() {
        return AprilTagPipelineResult.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufAprilTagPipelineResult.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {AprilTagTarget.proto, AprilTagMultiTargetResult.proto};
    }

    @Override
    public ProtobufAprilTagPipelineResult createMessage() {
        return ProtobufAprilTagPipelineResult.newInstance();
    }

    @Override
    public AprilTagPipelineResult unpack(ProtobufAprilTagPipelineResult msg) {
        return new AprilTagPipelineResult(
                msg.getLatency(),
                msg.getTimestamp(),
                AprilTagTarget.proto.unpack(msg.getTargets()),
                msg.hasMultiTargetResult()
                        ? Optional.of(AprilTagMultiTargetResult.proto.unpack(msg.getMultiTargetResult()))
                        : Optional.empty());
    }

    public AprilTagPipelineResult[] unpack(RepeatedMessage<ProtobufAprilTagPipelineResult> msg) {
        var results = new AprilTagPipelineResult[msg.length()];
        for (int i = 0; i < results.length; i++) {
            results[i] = unpack(msg.get(i));
        }

        return results;
    }

    @Override
    public void pack(ProtobufAprilTagPipelineResult msg, AprilTagPipelineResult value) {
        msg.setLatency(value.latencyMilliseconds);
        msg.setTimestamp(value.timestamp);
        AprilTagTarget.proto.pack(msg.getMutableTargets(), value.targets);
        value.multiTargetResult.ifPresent(
                result -> AprilTagMultiTargetResult.proto.pack(msg.getMutableMultiTargetResult(), result));
    }

    public void pack(RepeatedMessage<ProtobufAprilTagPipelineResult> msg, AprilTagPipelineResult[] values) {
        var targets = msg.reserve(values.length);
        for (var target : values) {
            pack(targets.next(), target);
        }
    }
}
