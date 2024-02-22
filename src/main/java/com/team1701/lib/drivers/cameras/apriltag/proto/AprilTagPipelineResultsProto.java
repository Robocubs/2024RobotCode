package com.team1701.lib.drivers.cameras.apriltag.proto;

import com.team1701.lib.drivers.cameras.apriltag.AprilTagPipelineResult;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagPipelineResults;
import com.team1701.proto.Apriltag.ProtobufAprilTagPipelineResults;
import edu.wpi.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class AprilTagPipelineResultsProto
        implements Protobuf<AprilTagPipelineResults, ProtobufAprilTagPipelineResults> {

    @Override
    public Class<AprilTagPipelineResults> getTypeClass() {
        return AprilTagPipelineResults.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufAprilTagPipelineResults.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {AprilTagPipelineResult.proto};
    }

    @Override
    public ProtobufAprilTagPipelineResults createMessage() {
        return ProtobufAprilTagPipelineResults.newInstance();
    }

    @Override
    public AprilTagPipelineResults unpack(ProtobufAprilTagPipelineResults msg) {
        return new AprilTagPipelineResults(AprilTagPipelineResult.proto.unpack(msg.getResults()));
    }

    @Override
    public void pack(ProtobufAprilTagPipelineResults msg, AprilTagPipelineResults value) {
        AprilTagPipelineResult.proto.pack(msg.getMutableResults(), value.results);
    }
}
