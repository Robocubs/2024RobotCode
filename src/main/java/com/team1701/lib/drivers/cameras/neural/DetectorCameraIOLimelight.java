package com.team1701.lib.drivers.cameras.neural;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Timer;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

/*
 * Subsystem for interacting with the Limelight 3
 */
public class DetectorCameraIOLimelight implements DetectorCameraIO {
    public static final double kHorizontalFOV = 63.3; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));

    private final StringSubscriber mJsonSub;
    private final String mName;
    private final JSONParser mParser = new JSONParser();
    private final VisionConfig mConfig;

    public DetectorCameraIOLimelight(VisionConfig config) {
        mConfig = config;
        mName = config.cameraName;
        var table = NetworkTableInstance.getDefault().getTable(mName);
        mJsonSub = table.getStringTopic("json").subscribe("");
    }

    @Override
    public void updateInputs(DetectorCameraInputs inputs) {
        var jsonAtomic = mJsonSub.getAtomic();
        var jsonTimestampSeconds = jsonAtomic.timestamp / 1000000.0;
        var isConnected = !"".equals(jsonAtomic.value) && Timer.getFPGATimestamp() - jsonTimestampSeconds < 1.0;
        inputs.isConnected = isConnected;
        if (!isConnected) {
            inputs.pipelineResult = new DetectorPipelineResult();
            return;
        }

        var latency = 0.0;
        var timestamp = jsonTimestampSeconds;
        try {
            var parser = (JSONObject) mParser.parse(jsonAtomic.value);
            var results = (JSONObject) parser.get("Results");

            var captureLatency = (long) results.get("cl");
            var targetingLatency = (long) results.get("tl");
            latency = (captureLatency + targetingLatency) / 1000.0;
            timestamp = jsonTimestampSeconds + latency;

            var seesTarget = (long) results.get("v") != 0;
            if (!seesTarget) {
                inputs.pipelineResult = new DetectorPipelineResult(latency, timestamp);
                return;
            }

            var detectorResults = (JSONArray) results.get("Detector");
            var detectedObjects = new DetectedObject[detectorResults.size()];
            for (int i = 0; i < detectedObjects.length; i++) {
                var obj = (JSONObject) detectorResults.get(i);
                detectedObjects[i] = new DetectedObject(
                        (String) obj.get("class"),
                        (int) obj.get("classID"),
                        (double) obj.get("conf"),
                        (double) obj.get("ta"),
                        Rotation2d.fromDegrees((double) obj.get("ty")),
                        Rotation2d.fromDegrees((double) obj.get("tx")),
                        new Translation2d((double) obj.get("txp"), (double) obj.get("typ")));
            }

            inputs.pipelineResult = new DetectorPipelineResult(latency, timestamp, detectedObjects);
        } catch (ParseException e) {
            e.printStackTrace();
            inputs.pipelineResult = new DetectorPipelineResult(latency, timestamp);
        }
    }

    @Override
    public VisionConfig getVisionConfig() {
        return mConfig;
    }
}
