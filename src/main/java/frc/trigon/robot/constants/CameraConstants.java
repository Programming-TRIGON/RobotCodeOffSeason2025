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
            new Translation3d(0.2015, 0.195, 0.7156),
            new Rotation3d(0, Units.degreesToRadians(30), 0)
    ),
            ROBOT_CENTER_TO_INTAKE_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(0.2247, 0.195, 0.7498),
                    new Rotation3d(0, Units.degreesToRadians(90 - 51), 0)
            ),
            ROBOT_CENTER_TO_LEFT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.129, 0.2032, 0.1258),
                    new Rotation3d(0, Units.degreesToRadians(60), 0)
            ),
            ROBOT_CENTER_TO_RIGHT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.129, -0.2032, 0.1258),
                    new Rotation3d(0, Units.degreesToRadians(60), 0)
            );

    public static final double OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 1;
    public static final ObjectDetectionCamera OBJECT_DETECTION_CAMERA = new ObjectDetectionCamera(
            "ObjectDetectionCamera",
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA
    );
    public static final AprilTagCamera
            INTAKE_SIDE_REEF_TAG_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "IntakeSideReefTagCameraCamera",
            ROBOT_CENTER_TO_INTAKE_REEF_TAG_CAMERA,
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