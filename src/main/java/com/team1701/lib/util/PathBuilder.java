package com.team1701.lib.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class PathBuilder {
    private final List<Pose2d> mPath = new ArrayList<>();

    public Pose2d[] build() {
        return mPath.toArray(new Pose2d[mPath.size()]);
    }

    public Pose2d[] buildAndClear() {
        var path = build();
        clear();
        return path;
    }

    public void clear() {
        mPath.clear();
    }

    public PathBuilder addPose(Pose2d pose) {
        // TODO: Add to mPath
        return this;
    }

    public PathBuilder addPath(Pose2d[] path) {
        Collections.addAll(mPath, path);
        return this;
    }

    public PathBuilder addPath(PathPlannerPath path) {
        // TODO: Add to mPath
        return this;
    }
}
