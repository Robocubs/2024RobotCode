package com.team1701.lib.util;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.CommandUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public record ChoreoEventMarker(double timestamp, Command command) {
    public static List<ChoreoEventMarker> loadFromFile(String trajectoryName) {
        try (var fileReader =
                new FileReader(new File(Filesystem.getDeployDirectory(), "choreo/" + trajectoryName + ".traj"))) {
            var json = (JSONObject) new JSONParser().parse(fileReader);

            if (!json.containsKey("eventMarkers")) {
                return List.of();
            }

            var eventMarkers = ((JSONArray) json.get("eventMarkers"));
            var eventCommands = new ArrayList<ChoreoEventMarker>(eventMarkers.size());
            for (var m : eventMarkers) {
                var marker = (JSONObject) m;
                var timestamp = ((Number) marker.get("timestamp")).doubleValue();
                var command = CommandUtil.commandFromJson((JSONObject) marker.get("command"), false);

                eventCommands.add(new ChoreoEventMarker(timestamp, command));
            }

            eventCommands.sort((m1, m2) -> Double.compare(m1.timestamp, m2.timestamp));

            return eventCommands;
        } catch (Exception e) {
            e.printStackTrace();
            return List.of();
        }
    }
}
