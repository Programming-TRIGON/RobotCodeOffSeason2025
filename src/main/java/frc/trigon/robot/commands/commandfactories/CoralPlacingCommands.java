package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.ReefChooser;
import frc.trigon.robot.subsystems.armelevator.ArmElevatorConstants;
import frc.trigon.robot.subsystems.endeffector.EndEffectorCommands;
import frc.trigon.robot.subsystems.endeffector.EndEffectorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.utilities.flippable.FlippablePose2d;
import lib.utilities.flippable.FlippableTranslation2d;

public class CoralPlacingCommands {
    public static boolean SHOULD_SCORE_AUTONOMOUSLY = true;
    static final ReefChooser REEF_CHOOSER = OperatorConstants.REEF_CHOOSER;

    public static Command getScoreInReefCommand(boolean shouldScoreRight) {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getLoadCoralCommand(),
                new ConditionalCommand(
                        getAutonomouslyScoreCommand(shouldScoreRight),
                        getScoreCommand(shouldScoreRight),
                        () -> SHOULD_SCORE_AUTONOMOUSLY && REEF_CHOOSER.getScoringLevel() != ScoringLevel.L1
                )
        ).onlyIf(() -> RobotContainer.END_EFFECTOR.hasGamePiece() || RobotContainer.TRANSPORTER.hasCoral());
    }

    private static Command getAutonomouslyScoreCommand(boolean shouldScoreRight) {
        return new SequentialCommandGroup(
                getAutonomouslyPrepareScoreCommand(shouldScoreRight).until(() -> isReadyToScore(shouldScoreRight)),
                new ParallelCommandGroup(
                        GeneralCommands.getFlippableOverridableArmCommand(REEF_CHOOSER::getArmElevatorState, false, CoralPlacingCommands::shouldReverseScore),
                        EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.SCORE_CORAL)
                )
        );
    }

    private static Command getScoreCommand(boolean shouldScoreRight) {
        return new SequentialCommandGroup(
                getAutonomouslyPrepareScoreCommand(shouldScoreRight).until(OperatorConstants.CONTINUE_TRIGGER),
                new ParallelCommandGroup(
                        GeneralCommands.getFlippableOverridableArmCommand(REEF_CHOOSER::getArmElevatorState, false, CoralPlacingCommands::shouldReverseScore),
                        EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.SCORE_CORAL)
                )
        );
    }

    private static Command getAutonomouslyPrepareScoreCommand(boolean shouldScoreRight) {
        return new ParallelCommandGroup(
                getPrepareArmElevatorIfWontHitReef(shouldScoreRight),
                new SequentialCommandGroup(
                        getAutonomousDriveToNoHitReefPose(shouldScoreRight).asProxy().onlyWhile(() -> !isPrepareArmAngleAboveCurrentArmAngle()),
                        getAutonomousDriveToReef(shouldScoreRight).asProxy()
                ).unless(() -> REEF_CHOOSER.getScoringLevel() == ScoringLevel.L1)
        );
    }

    private static Command getAutonomousDriveToReef(boolean shouldScoreRight) {
        return SwerveCommands.getDriveToPoseCommand(
                () -> CoralPlacingCommands.calculateClosestScoringPose(shouldScoreRight),
                AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS
        );
    }

    private static Command getAutonomousDriveToNoHitReefPose(boolean shouldScoreRight) {
        return SwerveCommands.getDriveToPoseCommand(
                () -> CoralPlacingCommands.calculateClosestNoHitReefPose(shouldScoreRight),
                AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS
        );
    }

    private static Command getPrepareArmElevatorIfWontHitReef(boolean shouldScoreRight) {
        return GeneralCommands.runWhen(
                GeneralCommands.getFlippableOverridableArmCommand(REEF_CHOOSER::getArmElevatorState, true, CoralPlacingCommands::shouldReverseScore),
                () -> CoralPlacingCommands.isPrepareArmAngleAboveCurrentArmAngle() || calculateDistanceToTargetScoringPose(shouldScoreRight) > FieldConstants.SAFE_DISTANCE_FROM_SCORING_POSE_METERS
        );
    }

    private static double calculateDistanceToTargetScoringPose(boolean shouldScoreRight) {
        final Translation2d currentTranslation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d targetTranslation = calculateClosestScoringPose(shouldScoreRight).get().getTranslation();
        return currentTranslation.getDistance(targetTranslation);
    }

    private static FlippablePose2d calculateClosestScoringPose(boolean shouldScoreRight) {
        final Translation2d robotPositionOnField = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;
        final Transform2d reefCenterToScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d());

        double distanceFromClosestScoringPoseMeters = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = new Pose2d();
        for (final Rotation2d targetRotation : reefClockAngles) {
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, targetRotation);


            final Pose2d currentScoringPose = reefCenterAtTargetRotation.transformBy(reefCenterToScoringPose);
            final double distanceFromCurrentScoringPoseMeters = currentScoringPose.getTranslation().getDistance(robotPositionOnField);
            if (distanceFromCurrentScoringPoseMeters < distanceFromClosestScoringPoseMeters) {
                distanceFromClosestScoringPoseMeters = distanceFromCurrentScoringPoseMeters;
                closestScoringPose = currentScoringPose;
            }
        }

        final Transform2d scoringPoseToBranch = new Transform2d(
                0,
                shouldScoreRight ? FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS : -FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS,
                shouldReverseScore() ? Rotation2d.k180deg : new Rotation2d()
        );

        return new FlippablePose2d(closestScoringPose.transformBy(scoringPoseToBranch), false);
    }

    private static FlippablePose2d calculateClosestNoHitReefPose(boolean shouldScoreRight) {
        final Translation2d robotPositionOnField = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;
        final Transform2d reefCenterToScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_NO_HIT_REEF_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d());

        double distanceFromClosestScoringPoseMeters = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = new Pose2d();
        for (final Rotation2d targetRotation : reefClockAngles) {
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, targetRotation);


            final Pose2d currentScoringPose = reefCenterAtTargetRotation.transformBy(reefCenterToScoringPose);
            final double distanceFromCurrentScoringPoseMeters = currentScoringPose.getTranslation().getDistance(robotPositionOnField);
            if (distanceFromCurrentScoringPoseMeters < distanceFromClosestScoringPoseMeters) {
                distanceFromClosestScoringPoseMeters = distanceFromCurrentScoringPoseMeters;
                closestScoringPose = currentScoringPose;
            }
        }

        final Transform2d scoringPoseToBranch = new Transform2d(
                0,
                shouldScoreRight ? FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS : -FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS,
                shouldReverseScore() ? Rotation2d.k180deg : new Rotation2d()
        );

        return new FlippablePose2d(closestScoringPose.transformBy(scoringPoseToBranch), false);
    }

    private static boolean isPrepareArmAngleAboveCurrentArmAngle() {
        ArmElevatorConstants.ArmElevatorState targetState = REEF_CHOOSER.getArmElevatorState();
        final Rotation2d targetAngle = targetState.prepareState == null
                ? targetState.targetAngle
                : targetState.prepareState.targetAngle;
        return RobotContainer.ARM_ELEVATOR.armAboveAngle(targetAngle) || RobotContainer.ARM_ELEVATOR.armAtAngle(targetAngle);
    }

    private static boolean isReadyToScore(boolean shouldScoreRight) {
        return RobotContainer.ARM_ELEVATOR.atState(REEF_CHOOSER.getArmElevatorState().prepareState, shouldReverseScore())
                && RobotContainer.SWERVE.atPose(calculateClosestScoringPose(shouldScoreRight));
    }

    static boolean shouldReverseScore() {
        final Rotation2d robotRotation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation();
        final Translation2d robotTranslation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterTranslation = FieldConstants.FLIPPABLE_REEF_CENTER_TRANSLATION.get();
        final Translation2d difference = reefCenterTranslation.minus(robotTranslation);
        final Rotation2d robotRotationRelativeToReef = difference.getAngle();
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

        public final ArmElevatorConstants.ArmElevatorState
                armElevatorState,
                armElevatorAlgaeCollectionState;
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
            this.armElevatorState = determineArmElevatorState();
            this.armElevatorAlgaeCollectionState = determineArmElevatorAlgaeCollectionState();
        }


        private ArmElevatorConstants.ArmElevatorState determineArmElevatorState() {
            return switch (level) {
                case 1 -> ArmElevatorConstants.ArmElevatorState.SCORE_L1;
                case 2 -> ArmElevatorConstants.ArmElevatorState.SCORE_L2;
                case 3 -> ArmElevatorConstants.ArmElevatorState.SCORE_L3;
                case 4 -> ArmElevatorConstants.ArmElevatorState.SCORE_L4;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private ArmElevatorConstants.ArmElevatorState determineArmElevatorAlgaeCollectionState() {
            return switch (level) {
                case 1, 2 -> ArmElevatorConstants.ArmElevatorState.COLLECT_ALGAE_L2;
                case 3, 4 -> ArmElevatorConstants.ArmElevatorState.COLLECT_ALGAE_L3;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private int calculateLevel() {
            return ordinal() + 1;
        }
    }
}