package com.team1701.lib.drivers.cameras.neural;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    private final NetworkTable mNetworkTable;
    private final String mName;
    private final JSONParser mParser = new JSONParser();
    private final VisionConfig mConfig;

    public DetectorCameraIOLimelight(VisionConfig config) {
        mConfig = config;
        mName = config.cameraName;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(mName);
    }

    @Override
    public void updateInputs(DetectorCameraInputs inputs) {
        // Adding capture latency (photons -> beginning of pipeline) to pipeline latency (beginning of pipeline -> data
        // send)
        var cl = mNetworkTable.getEntry("cl").getDouble(0.0);
        var latency = inputs.latency = (mNetworkTable.getEntry("tl").getDouble(0.0) + cl) / 1000.0;
        inputs.givenPipeline = (long) mNetworkTable.getEntry("pipeline").getInteger(0);
        inputs.isConnected = cl != 0.0;
        var seesTarget = inputs.seesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        if (seesTarget) {
            inputs.captureTimestamp = Timer.getFPGATimestamp() - latency;
            var multiTargetJSON = mNetworkTable.getEntry("json").getString("");
            try {
                var p = (JSONObject) mParser.parse(multiTargetJSON);
                var results = (JSONObject) p.get("Results");

                var detectorResults = (JSONArray) results.get("Detector");
                var numberOfDetectedObjects = inputs.numberOfDetectedObjects = detectorResults.size();
                JSONObject obj;

                inputs.constructEmptyInputArrays(numberOfDetectedObjects);

                for (int i = 0; i < numberOfDetectedObjects; i++) {
                    obj = (JSONObject) detectorResults.get(i);
                    inputs.detectedClasses[i] = (String) obj.get("class");
                    inputs.detectedClassIDs[i] = (long) obj.get("classID");
                    inputs.confidences[i] = (double) obj.get("conf");
                    inputs.areas[i] = (double) obj.get("ta");
                    inputs.txs[i] = (double) obj.get("tx");
                    inputs.txps[i] = (double) obj.get("txp");
                    inputs.tys[i] = (double) obj.get("ty");
                    inputs.typs[i] = (double) obj.get("typ");
                }

            } catch (ParseException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public VisionConfig getVisionConfig() {
        return mConfig;
    }
}
