package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.WaitUntilChangeCommand;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.ReefChooser;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.utilities.flippable.FlippablePose2d;
import org.trigon.utilities.flippable.FlippableTranslation2d;

public class CoralPlacingCommands {
    public static boolean SHOULD_SCORE_AUTONOMOUSLY = true;
    private static final ReefChooser REEF_CHOOSER = OperatorConstants.REEF_CHOOSER;

    public static Command getScoreInReefCommand(boolean shouldScoreRight) {
        return new ConditionalCommand(
                getScoreInReefFromCoralIntakeCommand().asProxy(),
                getScoreInReefFromGripperCommand(shouldScoreRight).asProxy(),
                () -> REEF_CHOOSER.getScoringLevel() == ScoringLevel.L1_CORAL_INTAKE
        );
    }

    private static Command getScoreInReefFromGripperCommand(boolean shouldScoreRight) {
        return GeneralCommands.getContinuousConditionalCommand(
                getAutonomouslyScoreInReefFromGripperCommand(shouldScoreRight).asProxy(),
                getManuallyScoreInReefFromGripperCommand().asProxy(),
                () -> SHOULD_SCORE_AUTONOMOUSLY && REEF_CHOOSER.getScoringLevel() != ScoringLevel.L1_GRIPPER
        ).until(() -> REEF_CHOOSER.getScoringLevel() == ScoringLevel.L1_CORAL_INTAKE);
    }

    private static Command getScoreInReefFromCoralIntakeCommand() {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getUnloadCoralCommand().onlyIf(RobotContainer.GRIPPER::hasGamePiece),
                CoralIntakeCommands.getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.SCORE_L1).until(CoralPlacingCommands::canContinueScoringFromCoralIntake),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.SCORE_L1_BOOST).withTimeout(0.05),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.SCORE_L1)
        ).until(() -> REEF_CHOOSER.getScoringLevel() != ScoringLevel.L1_CORAL_INTAKE);
    }

    private static Command getManuallyScoreInReefFromGripperCommand() {
        return CoralCollectionCommands.getLoadCoralCommand().asProxy().andThen(
                new ParallelCommandGroup(
                        ElevatorCommands.getSetTargetStateCommand(REEF_CHOOSER::getElevatorState).raceWith(new WaitUntilChangeCommand<>(REEF_CHOOSER::getElevatorState)).repeatedly(),
                        getGripperScoringSequenceCommand()
                ).asProxy()
        );
    }

    private static Command getAutonomouslyScoreInReefFromGripperCommand(boolean shouldScoreRight) {
        return new ParallelRaceGroup(
                CoralCollectionCommands.getLoadCoralCommand().asProxy().andThen(
                        new ParallelCommandGroup(
                                getOpenElevatorWhenCloseToReefCommand(shouldScoreRight),
                                getAutoGripperScoringSequenceCommand(shouldScoreRight)
                        ).asProxy().raceWith(new WaitUntilChangeCommand<>(REEF_CHOOSER::getElevatorState)).repeatedly()
                ),
                getAutonomousDriveToReefThenManualDriveCommand(shouldScoreRight)
        ).andThen();
    }

    private static Command getGripperScoringSequenceCommand() {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.OPEN_FOR_NOT_HITTING_REEF)
                        .unless(() -> RobotContainer.ELEVATOR.atState(REEF_CHOOSER.getElevatorState()) || REEF_CHOOSER.getScoringLevel() == ScoringLevel.L2 || REEF_CHOOSER.getScoringLevel() == ScoringLevel.L1_GRIPPER)
                        .until(RobotContainer.ELEVATOR::isCloseEnoughToOpenGripper),
                scoreFromGripperReefChooserCommand()
        );
    }

    private static Command getAutoGripperScoringSequenceCommand(boolean shouldScoreRight) {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.OPEN_FOR_NOT_HITTING_REEF)
                        .unless(() -> RobotContainer.ELEVATOR.atState(REEF_CHOOSER.getElevatorState()) || REEF_CHOOSER.getScoringLevel() == ScoringLevel.L2 || REEF_CHOOSER.getScoringLevel() == ScoringLevel.L1_GRIPPER)
                        .until(RobotContainer.ELEVATOR::isCloseEnoughToOpenGripper),
                new ConditionalCommand(
                        scoreFromGripperInL4Command(shouldScoreRight),
                        scoreFromGripperReefChooserCommand(shouldScoreRight),
                        () -> REEF_CHOOSER.getScoringLevel() == ScoringLevel.L4
                )
        );
    }

    private static Command scoreFromGripperReefChooserCommand(boolean shouldScoreRight) {
        return new SequentialCommandGroup(
                GripperCommands.getPrepareForStateCommand(REEF_CHOOSER::getGripperState).raceWith(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> canAutonomouslyReleaseFromGripper(shouldScoreRight)),
                                new WaitUntilChangeCommand<>(RobotContainer.ROBOT_POSE_ESTIMATOR::getEstimatedRobotPose)
                        )
                ).until(OperatorConstants.CONTINUE_TRIGGER),
                GripperCommands.getSetTargetStateCommand(REEF_CHOOSER::getGripperState).finallyDo(OperatorConstants.REEF_CHOOSER::switchReefSide)
        );
    }

    private static Command scoreFromGripperReefChooserCommand() {
        return new SequentialCommandGroup(
                GripperCommands.getPrepareForStateCommand(REEF_CHOOSER::getGripperState).until(OperatorConstants.CONTINUE_TRIGGER),
                GripperCommands.getSetTargetStateCommand(REEF_CHOOSER::getGripperState).finallyDo(OperatorConstants.REEF_CHOOSER::switchReefSide)
        );
    }

    private static Command scoreFromGripperInL4Command(boolean shouldScoreRight) {
        return new SequentialCommandGroup(
                GripperCommands.getPrepareForScoringInL4Command(REEF_CHOOSER::calculateTargetScoringPose).raceWith(
                        new WaitUntilCommand(() -> canAutonomouslyReleaseFromGripper(shouldScoreRight))
                ).until(OperatorConstants.CONTINUE_TRIGGER),
                GripperCommands.getScoreInL4Command(REEF_CHOOSER::calculateTargetScoringPose).finallyDo(OperatorConstants.REEF_CHOOSER::switchReefSide)
        );
    }

    private static Command getOpenElevatorWhenCloseToReefCommand(boolean shouldScoreRight) {
        return GeneralCommands.runWhen(
                ElevatorCommands.getSetTargetStateCommand(REEF_CHOOSER::getElevatorState),
                () -> calculateDistanceToTargetScoringPose(shouldScoreRight) < AutonomousConstants.MINIMUM_DISTANCE_FROM_REEF_TO_OPEN_ELEVATOR
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
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;
        final Transform2d
                reefCenterToScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d()),
                scoringPoseToRightBranch = new Transform2d(0, FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, new Rotation2d());

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

        return new FlippablePose2d(closestScoringPose.transformBy(shouldScoreRight ? scoringPoseToRightBranch : scoringPoseToRightBranch.inverse()), false);
    }

    private static boolean canContinueScoringFromCoralIntake() {
        return OperatorConstants.CONTINUE_TRIGGER.getAsBoolean();
    }

    private static boolean canAutonomouslyReleaseFromGripper(boolean shouldScoreRight) {
        return RobotContainer.ELEVATOR.atTargetState() &&
                RobotContainer.GRIPPER.atTargetAngle() &&
                RobotContainer.SWERVE.atPose(calculateClosestScoringPose(shouldScoreRight));
    }

    /**
     * An enum that represents the different levels of scoring in the reef.
     * Each level has a different x and y transform from the reef center,
     * as well as a different elevator and gripper state.
     * The x and y transform are used to calculate the target placing position from the middle of the reef.
     */
    public enum ScoringLevel {
        L1_CORAL_INTAKE(Double.NaN, Double.NaN, null),
        L1_GRIPPER(Double.NaN, Double.NaN, null),
        L2(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, Rotation2d.fromDegrees(0)),
        L3(L2.xTransformMeters, L2.positiveYTransformMeters, Rotation2d.fromDegrees(0)),
        L4(L2.xTransformMeters, L2.positiveYTransformMeters, Rotation2d.fromDegrees(0));

        public final ElevatorConstants.ElevatorState elevatorState;
        public final GripperConstants.GripperState gripperState;
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
            this.gripperState = determineGripperState();
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
            return switch (ordinal()) {
                case 0 -> ElevatorConstants.ElevatorState.REST;
                case 1 -> ElevatorConstants.ElevatorState.SCORE_L1;
                case 2 -> ElevatorConstants.ElevatorState.SCORE_L2;
                case 3 -> ElevatorConstants.ElevatorState.SCORE_L3;
                case 4 -> ElevatorConstants.ElevatorState.SCORE_L4;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private GripperConstants.GripperState determineGripperState() {
            return switch (ordinal()) {
                case 0 -> null;
                case 1 -> GripperConstants.GripperState.SCORE_L1;
                case 2, 3 -> GripperConstants.GripperState.SCORE_L3_OR_L2;
                case 4 -> GripperConstants.GripperState.SCORE_L4_CLOSE;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private int calculateLevel() {
            if (ordinal() == 0)
                return 1;
            return ordinal();
        }
    }
}