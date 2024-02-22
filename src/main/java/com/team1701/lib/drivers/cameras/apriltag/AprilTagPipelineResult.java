package com.team1701.lib.drivers.cameras.apriltag;

import java.util.Optional;

import com.team1701.lib.drivers.cameras.apriltag.proto.AprilTagPipelineResultProto;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

public class AprilTagPipelineResult implements ProtobufSerializable {
    public static final AprilTagPipelineResult kEmpty =
            new AprilTagPipelineResult(0, 0, new AprilTagTarget[] {}, Optional.empty());

    public final double latencyMilliseconds;
    public final double timestamp;
    public final AprilTagTarget[] targets;
    public final Optional<AprilTagMultiTargetResult> multiTargetResult;

    public AprilTagPipelineResult(
            double latencyMilliseconds,
            double timestamp,
            AprilTagTarget[] targets,
            Optional<AprilTagMultiTargetResult> multiTargetResult) {
        this.latencyMilliseconds = latencyMilliseconds;
        this.timestamp = timestamp;
        this.targets = targets;
        this.multiTargetResult = multiTargetResult;
    }

    public static final AprilTagPipelineResultProto proto = new AprilTagPipelineResultProto();
}
