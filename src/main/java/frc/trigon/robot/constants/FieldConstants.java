package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import lib.utilities.FilesHandler;
import lib.utilities.flippable.FlippablePose2d;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

public class FieldConstants {
    public static final double
            FIELD_WIDTH_METERS = FlippingUtil.fieldSizeY,
            FIELD_LENGTH_METERS = FlippingUtil.fieldSizeX;
    private static final List<Integer> I_HATE_YOU = List.of(
            //Tags to ignore
    );

    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIDToPoseMap();

    private static final double
            AUTO_FIND_CORAL_POSE_X = 3.3,
            AUTO_FIND_CORAL_POSE_LEFT_Y = 5.5;
    private static final Rotation2d AUTO_FIND_CORAL_POSE_LEFT_ROTATION = Rotation2d.fromDegrees(130);
    public static final FlippablePose2d
            AUTO_FIND_CORAL_POSE_LEFT = new FlippablePose2d(AUTO_FIND_CORAL_POSE_X, AUTO_FIND_CORAL_POSE_LEFT_Y, AUTO_FIND_CORAL_POSE_LEFT_ROTATION, true),
            AUTO_FIND_CORAL_POSE_RIGHT = new FlippablePose2d(AUTO_FIND_CORAL_POSE_X, FieldConstants.FIELD_WIDTH_METERS - AUTO_FIND_CORAL_POSE_LEFT_Y, AUTO_FIND_CORAL_POSE_LEFT_ROTATION.unaryMinus(), true);

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "field_calibration.json") :
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIDToPoseMap() {
        final HashMap<Integer, Pose3d> tagIDToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            if (!I_HATE_YOU.contains(aprilTag.ID))
                tagIDToPose.put(aprilTag.ID, aprilTag.pose.transformBy(TAG_OFFSET));

        return tagIDToPose;
    }
}
