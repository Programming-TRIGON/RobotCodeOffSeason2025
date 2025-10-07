package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.poseestimator.StandardDeviations;

public class CameraConstants {
    private static final StandardDeviations
            REEF_TAG_CAMERA_STANDARD_DEVIATIONS = new StandardDeviations(
            0.015,
            0.01
    );
    private static final Transform3d
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA = new Transform3d(
            new Translation3d(0.204, -0.170, 0.972),
            new Rotation3d(0, Units.degreesToRadians(42), Units.degreesToRadians(15))
    ),
            ROBOT_CENTER_TO_FRONT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(0.221, -0.165, 1.017),
                    new Rotation3d(0, Units.degreesToRadians(42), Units.degreesToRadians(15))
            ),
            ROBOT_CENTER_TO_LEFT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.2032, 0.129, 0.1258),
                    new Rotation3d(0, Units.degreesToRadians(60), Units.degreesToRadians(180))
            ),
            ROBOT_CENTER_TO_RIGHT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.2032, -0.129, 0.1258),
                    new Rotation3d(0, Units.degreesToRadians(60), Units.degreesToRadians(180))
            );

    public static final double OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 1;
    public static final ObjectDetectionCamera OBJECT_DETECTION_CAMERA = new ObjectDetectionCamera(
            "ObjectDetectionCamera",
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA
    );
    public static final AprilTagCamera
            FRONT_REEF_TAG_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "FrontReefTagCamera",
            ROBOT_CENTER_TO_FRONT_REEF_TAG_CAMERA,
            REEF_TAG_CAMERA_STANDARD_DEVIATIONS
    ),
            LEFT_REEF_TAG_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "LeftReefTagCamera",
                    ROBOT_CENTER_TO_LEFT_REEF_TAG_CAMERA,
                    REEF_TAG_CAMERA_STANDARD_DEVIATIONS
            ),
            RIGHT_REEF_TAG_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "RightReefTagCamera",
                    ROBOT_CENTER_TO_RIGHT_REEF_TAG_CAMERA,
                    REEF_TAG_CAMERA_STANDARD_DEVIATIONS
            );
}