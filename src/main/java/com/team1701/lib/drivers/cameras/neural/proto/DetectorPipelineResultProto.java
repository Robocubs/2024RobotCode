package com.team1701.lib.drivers.cameras.neural.proto;

import com.team1701.lib.drivers.cameras.neural.DetectedObject;
import com.team1701.lib.drivers.cameras.neural.DetectorPipelineResult;
import com.team1701.proto.Detector.ProtobufDetectorPipelineResult;
import edu.wpi.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class DetectorPipelineResultProto implements Protobuf<DetectorPipelineResult, ProtobufDetectorPipelineResult> {

    @Override
    public Class<DetectorPipelineResult> getTypeClass() {
        return DetectorPipelineResult.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufDetectorPipelineResult.getDescriptor();
    }

    @Override
    public ProtobufDetectorPipelineResult createMessage() {
        return ProtobufDetectorPipelineResult.newInstance();
    }

    @Override
    public DetectorPipelineResult unpack(ProtobufDetectorPipelineResult msg) {
        return new DetectorPipelineResult(
                msg.getLatency(), msg.getTimestamp(), DetectedObject.proto.unpack(msg.getDetectedObjects()));
    }

    @Override
    public void pack(ProtobufDetectorPipelineResult msg, DetectorPipelineResult value) {
        msg.setLatency(value.latency);
        msg.setTimestamp(value.timestamp);
        DetectedObject.proto.pack(msg.getMutableDetectedObjects(), value.detectedObjects);
    }
}
