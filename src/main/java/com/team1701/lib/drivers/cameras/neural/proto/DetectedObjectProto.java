package com.team1701.lib.drivers.cameras.neural.proto;

import com.team1701.lib.drivers.cameras.neural.DetectedObject;
import com.team1701.proto.Detector.ProtobufDetectedObject;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;
import us.hebi.quickbuf.RepeatedMessage;

public class DetectedObjectProto implements Protobuf<DetectedObject, ProtobufDetectedObject> {

    @Override
    public Class<DetectedObject> getTypeClass() {
        return DetectedObject.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufDetectedObject.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {Rotation2d.proto, Translation2d.proto};
    }

    @Override
    public ProtobufDetectedObject createMessage() {
        return ProtobufDetectedObject.newInstance();
    }

    @Override
    public DetectedObject unpack(ProtobufDetectedObject msg) {
        return new DetectedObject(
                msg.getDetectedClass(),
                msg.getDetectedClassId(),
                msg.getConfidence(),
                msg.getArea(),
                Rotation2d.proto.unpack(msg.getPitch()),
                Rotation2d.proto.unpack(msg.getYaw()),
                Translation2d.proto.unpack(msg.getPositionPixels()));
    }

    public DetectedObject[] unpack(RepeatedMessage<ProtobufDetectedObject> msg) {
        var results = new DetectedObject[msg.length()];
        for (int i = 0; i < results.length; i++) {
            results[i] = unpack(msg.get(i));
        }

        return results;
    }

    @Override
    public void pack(ProtobufDetectedObject msg, DetectedObject value) {
        msg.setDetectedClass(value.detectedClass);
        msg.setDetectedClassId(value.detectedClassId);
        msg.setConfidence(value.confidence);
        msg.setArea(value.area);
        Rotation2d.proto.pack(msg.getMutablePitch(), value.pitch);
        Rotation2d.proto.pack(msg.getMutableYaw(), value.yaw);
        Translation2d.proto.pack(msg.getMutablePositionPixels(), value.positionPixels);
    }

    public void pack(RepeatedMessage<ProtobufDetectedObject> msg, DetectedObject[] values) {
        var targets = msg.reserve(values.length);
        for (var target : values) {
            pack(targets.next(), target);
        }
    }
}
