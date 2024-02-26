package com.team1701.lib.drivers.cameras.neural;

import com.team1701.lib.drivers.cameras.neural.proto.DetectorPipelineResultProto;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

public class DetectorPipelineResult implements ProtobufSerializable {
    public final double latency;
    public final double timestamp;
    public final DetectedObject[] detectedObjects;

    public DetectorPipelineResult() {
        this(0, 0, new DetectedObject[] {});
    }

    public DetectorPipelineResult(double latency, double timestamp) {
        this(latency, timestamp, new DetectedObject[] {});
    }

    public DetectorPipelineResult(double latency, double timestamp, DetectedObject[] detectedObjects) {
        this.latency = latency;
        this.timestamp = timestamp;
        this.detectedObjects = detectedObjects;
    }

    public static final DetectorPipelineResultProto proto = new DetectorPipelineResultProto();
}
