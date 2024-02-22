package com.team1701.lib.drivers.cameras.apriltag;

import com.team1701.lib.drivers.cameras.apriltag.proto.AprilTagPipelineResultsProto;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

public class AprilTagPipelineResults implements ProtobufSerializable {
    public final AprilTagPipelineResult[] results;

    public AprilTagPipelineResults() {
        results = new AprilTagPipelineResult[] {};
    }

    public AprilTagPipelineResults(AprilTagPipelineResult[] results) {
        this.results = results;
    }

    public static final AprilTagPipelineResultsProto proto = new AprilTagPipelineResultsProto();
}
