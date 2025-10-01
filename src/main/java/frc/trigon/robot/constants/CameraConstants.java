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
    private static final Transform3d ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA = new Transform3d(
            new Translation3d(0.2015, 0.195, 0.7156),
            new Rotation3d(0, Units.degreesToRadians(30), 0)
    );

    public static final double OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 0;
    public static final ObjectDetectionCamera OBJECT_DETECTION_CAMERA = new ObjectDetectionCamera(
            "ObjectDetectionCamera",
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA
    );
    public static final AprilTagCamera
            INTAKE_SIDE_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "IntakeSideCamera",
            new Transform3d(
                    new Translation3d(0.2247, 0.195, 0.7498),
                    new Rotation3d(0, Units.degreesToRadians(51), 0)
            ),
            new StandardDeviations(0, 0) //TOD0: Measure
    );
}