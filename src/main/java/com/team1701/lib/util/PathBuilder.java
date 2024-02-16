package com.team1701.lib.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
        mPath.add(pose);
        return this;
    }

    public PathBuilder addPath(Pose2d[] path) {
        Collections.addAll(mPath, path);
        return this;
    }

    public PathBuilder addPath(PathPlannerPath path) {
        path.getAllPathPoints().forEach(pathPoint -> {
            Rotation2d rotation;
            if (pathPoint.rotationTarget != null) {
                rotation = pathPoint.rotationTarget.getTarget();
            } else if (!mPath.isEmpty()) {
                rotation = mPath.get(mPath.size() - 1).getRotation();
            } else {
                rotation = GeometryUtil.kRotationIdentity;
            }
            mPath.add(new Pose2d(pathPoint.position, rotation));
        });

        return this;
    }
}
