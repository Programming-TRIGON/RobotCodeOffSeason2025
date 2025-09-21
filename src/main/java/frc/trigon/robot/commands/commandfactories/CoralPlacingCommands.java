package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.ReefChooser;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.utilities.flippable.FlippablePose2d;
import lib.utilities.flippable.FlippableTranslation2d;

public class CoralPlacingCommands {
    public static boolean SHOULD_SCORE_AUTONOMOUSLY = true;
    private static final ReefChooser REEF_CHOOSER = OperatorConstants.REEF_CHOOSER;

    public static Command getScoreCommand(boolean shouldScoreRight) {
        return new SequentialCommandGroup(
                getAutonomouslyPrepareScoreCommand(shouldScoreRight).until(OperatorConstants.CONTINUE_TRIGGER),
                new ParallelCommandGroup(
                        ElevatorCommands.getSetTargetStateCommand(REEF_CHOOSER::getElevatorState),
                        ArmCommands.getSetTargetStateCommand(REEF_CHOOSER::getArmState, CoralPlacingCommands::shouldReverseScore)
                )
        );
    }

    private static Command getAutonomouslyPrepareScoreCommand(boolean shouldScoreRight) {
        return new ParallelCommandGroup(
                ElevatorCommands.getPrepareStateCommand(REEF_CHOOSER::getElevatorState),
                ArmCommands.getPrepareForStateCommand(REEF_CHOOSER::getArmState, CoralPlacingCommands::shouldReverseScore),
                getAutonomousDriveToReefThenManualDriveCommand(shouldScoreRight)
        );
    }

    private static Command getAutonomousDriveToReefThenManualDriveCommand(boolean shouldScoreRight) {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        () -> CoralPlacingCommands.calculateClosestScoringPose(shouldScoreRight),
                        AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS
                ),
                GeneralCommands.getFieldRelativeDriveCommand()
        );
    }

    private static double calculateDistanceToTargetScoringPose(boolean shouldScoreRight) {
        final Translation2d currentTranslation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d targetTranslation = calculateClosestScoringPose(shouldScoreRight).get().getTranslation();
        return currentTranslation.getDistance(targetTranslation);
    }

    public static FlippablePose2d calculateClosestScoringPose(boolean shouldScoreRight) {
        final Translation2d robotPositionOnField = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Rotation2d robotOrientationOnField = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;
        final Transform2d
                reefCenterToScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d()),
                scoringPoseToRightBranch = new Transform2d(0, FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, new Rotation2d());

        double distanceFromClosestScoringPoseMeters = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = new Pose2d();
        for (final Rotation2d targetRotation : reefClockAngles) {
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, shouldReverseScore() ? targetRotation.unaryMinus() : targetRotation);


            final Pose2d currentScoringPose = reefCenterAtTargetRotation.transformBy(reefCenterToScoringPose);
            final double distanceFromCurrentScoringPoseMeters = currentScoringPose.getTranslation().getDistance(robotPositionOnField);
            if (distanceFromCurrentScoringPoseMeters < distanceFromClosestScoringPoseMeters) {
                distanceFromClosestScoringPoseMeters = distanceFromCurrentScoringPoseMeters;
                closestScoringPose = currentScoringPose;
            }
        }

        return new FlippablePose2d(closestScoringPose.transformBy(shouldScoreRight ? scoringPoseToRightBranch : scoringPoseToRightBranch.inverse()), false);
    }

    private static boolean shouldReverseScore() {
        final Rotation2d robotRotation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation();
        final Translation2d robotTranslation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterTranslation = FieldConstants.FLIPPABLE_REEF_CENTER_TRANSLATION.get();
        final double reefCenterToRobotX = robotTranslation.getX() - reefCenterTranslation.getX();
        final double reefCenterToRobotY = robotTranslation.getY() - reefCenterTranslation.getY();
        final Rotation2d robotRotationRelativeToReef = Rotation2d.fromRadians(Math.atan2(reefCenterToRobotX, reefCenterToRobotY));
        final Rotation2d robotRotationFacingReef = robotRotation.minus(robotRotationRelativeToReef);
        return robotRotationFacingReef.getDegrees() > Rotation2d.kCW_90deg.getDegrees() && robotRotationFacingReef.getDegrees() < Rotation2d.kCCW_90deg.getDegrees();
    }

    /**
     * An enum that represents the different levels of scoring in the reef.
     * Each level has a different x and y transform from the reef center,
     * as well as a different elevator and gripper state.
     * The x and y transform are used to calculate the target placing position from the middle of the reef.
     */
    public enum ScoringLevel {
        L1(Double.NaN, Double.NaN, null),
        L2(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, Rotation2d.fromDegrees(0)),
        L3(L2.xTransformMeters, L2.positiveYTransformMeters, Rotation2d.fromDegrees(0)),
        L4(L2.xTransformMeters, L2.positiveYTransformMeters, Rotation2d.fromDegrees(0));

        public final ElevatorConstants.ElevatorState elevatorState;
        public final ArmConstants.ArmState armState;
        public final int level = calculateLevel();
        final double xTransformMeters, positiveYTransformMeters;
        final Rotation2d rotationTransform;

        /**
         * Constructs a scoring level with the given x and y transform.
         * The elevator and gripper state are determined automatically based on the scoring level.
         *
         * @param xTransformMeters         the x transform from the middle of the reef to the target placing position
         * @param positiveYTransformMeters the y transform from the middle of the reef to the target placing position.
         *                                 This must be positive (to account for flipping later on), and might be flipped depending on operator input (left or right reef side)
         * @param rotationTransform        the angle to be facing the reef with the robot. Might change when scoring from the coral intake
         */
        ScoringLevel(double xTransformMeters, double positiveYTransformMeters, Rotation2d rotationTransform) {
            this.xTransformMeters = xTransformMeters;
            this.positiveYTransformMeters = positiveYTransformMeters;
            this.rotationTransform = rotationTransform;
            this.elevatorState = determineElevatorState();
            this.armState = determineArmState();
        }

        /**
         * Calculates the target placing position using the clock position and the target reef side.
         * The reef side transform will be flipped depending on operator input.
         * To make it more intuitive for the operator to input the reef side,
         * left will always correspond to the physical left side in the driver station,
         * as opposed to "reef relative" left.
         *
         * @param reefClockPosition the desired clock position of the reef
         * @param reefSide          the desired side of the reef, left or right (as seen from the driver station)
         * @return the target placing position
         */
        public FlippablePose2d calculateTargetPlacingPosition(FieldConstants.ReefClockPosition reefClockPosition, FieldConstants.ReefSide reefSide) {
            final Pose2d reefCenterPose = new Pose2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, reefClockPosition.clockAngle);
            final double yTransform = reefSide.shouldFlipYTransform(reefClockPosition) ? -positiveYTransformMeters : positiveYTransformMeters;
            final Transform2d transform = new Transform2d(xTransformMeters, yTransform, rotationTransform);

            return new FlippablePose2d(reefCenterPose.plus(transform), true);
        }

        private ElevatorConstants.ElevatorState determineElevatorState() {
            return switch (level) {
                case 1 -> ElevatorConstants.ElevatorState.SCORE_L1;
                case 2 -> ElevatorConstants.ElevatorState.SCORE_L2;
                case 3 -> ElevatorConstants.ElevatorState.SCORE_L3;
                case 4 -> ElevatorConstants.ElevatorState.SCORE_L4;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private ArmConstants.ArmState determineArmState() {
            return switch (level) {
                case 1 -> ArmConstants.ArmState.SCORE_L1;
                case 2 -> ArmConstants.ArmState.SCORE_L2;
                case 3 -> ArmConstants.ArmState.SCORE_L3;
                case 4 -> ArmConstants.ArmState.SCORE_L4;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private int calculateLevel() {
            return ordinal() + 1;
        }
    }
}