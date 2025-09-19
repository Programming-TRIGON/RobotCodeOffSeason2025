package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;
import lib.utilities.flippable.FlippablePose2d;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.io.IOException;
import java.util.function.Supplier;

import static frc.trigon.robot.RobotContainer.CORAL_POSE_ESTIMATOR;

/**
 * A class that contains command factories for preparation commands and commands used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    public static final LoggedNetworkBoolean[] SCORED_L4S = getEmptyL4LoggedNetworkBooleanArray();
    private static FlippablePose2d TARGET_SCORING_POSE = null;

    public static Command getFloorAutonomousCommand(boolean isRight) {
        return getCycleCoralCommand(isRight).repeatedly().withName("FloorAutonomous" + (isRight ? "Right" : "Left"));
    }

    public static Command getCycleCoralCommand(boolean isRight) {
        return new SequentialCommandGroup(
                getDriveToReefAndScoreCommand(),
                getCollectCoralCommand(isRight)
        );
    }

    public static Command getFindCoralCommand(boolean isRight) {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(() -> isRight ? FieldConstants.AUTO_FIND_CORAL_POSE_RIGHT : FieldConstants.AUTO_FIND_CORAL_POSE_LEFT, AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS, 2.3),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> 0,
                        () -> 0,
                        () -> 0.2
                )
        );
    }

    public static Command getDriveToReefAndScoreCommand() {
        return new ParallelRaceGroup(
                getDriveToReefCommand(),
                getCoralSequenceCommand()
        );
    }

    public static Command getCollectCoralCommand(boolean isRight) {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.COLLECT),
                getDriveToCoralCommand(isRight)
        )
                .until(RobotContainer.INTAKE::hasCoral)
                .unless(() -> RobotContainer.INTAKE.hasCoral() || RobotContainer.ARM.hasGamePiece());
    }

    public static Command getDriveToReefCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> TARGET_SCORING_POSE = calculateClosestOpenScoringPose()),
                new WaitUntilCommand(() -> TARGET_SCORING_POSE != null).raceWith(SwerveCommands.getClosedLoopSelfRelativeDriveCommand(() -> 0, () -> 0, () -> 0)),
                SwerveCommands.getDriveToPoseCommand(() -> TARGET_SCORING_POSE, AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS).repeatedly()
        );
    }

    public static Command getCoralSequenceCommand() {
        return new SequentialCommandGroup(
                //CoralPlacementCommands.getLoadCoralCommand(),TODO
                new WaitUntilCommand(() -> TARGET_SCORING_POSE != null),
                getScoreCommand()
        );
    }

    public static Command getDriveToCoralCommand(boolean isRight) {
        return new SequentialCommandGroup(
                getFindCoralCommand(isRight).unless(() -> CORAL_POSE_ESTIMATOR.getClosestObjectToRobot() != null).until(() -> CORAL_POSE_ESTIMATOR.getClosestObjectToRobot() != null),
                IntakeAssistCommand.getAssistIntakeCommand(IntakeAssistCommand.AssistMode.FULL_ASSIST, IntakeAssistCommand::calculateDistanceFromTrackedCGamePiece).withTimeout(1.5)
        ).repeatedly();
    }

    public static Command getScoreCommand() {
        return new SequentialCommandGroup(
                getPrepareForScoreCommand().until(AutonomousCommands::canScore),
                getPlaceCoralCommand()
        );
    }

    public static Command getPrepareForScoreCommand() {
        return new ParallelCommandGroup(
                getOpenElevatorWhenCloseToReefCommand(),
                ArmCommands.getSetTargetStateCommand(ArmConstants.ArmState.PREPARE_L4)
        );
    }

    private static boolean canScore() {
        return RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.PREPARE_L4) &&
                RobotContainer.ARM.atState(ArmConstants.ArmState.PREPARE_L4) &&
                TARGET_SCORING_POSE != null &&
                Math.abs(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().relativeTo(TARGET_SCORING_POSE.get()).getX()) < AutonomousConstants.REEF_RELATIVE_X_TOLERANCE_METERS &&
                Math.abs(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().relativeTo(TARGET_SCORING_POSE.get()).getY()) < AutonomousConstants.REEF_RELATIVE_Y_TOLERANCE_METERS;
    }

    public static Command getPlaceCoralCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_L4),
                ArmCommands.getSetTargetStateCommand(ArmConstants.ArmState.SCORE_L4),
                getAddCurrentScoringBranchToScoredBranchesCommand()
        ).withTimeout(0.25);
    }

    private static Command getOpenElevatorWhenCloseToReefCommand() {
        return GeneralCommands.runWhen(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.PREPARE_L4),
                () -> calculateDistanceToTargetScoringPose() < AutonomousConstants.MINIMUM_DISTANCE_FROM_REEF_TO_OPEN_ELEVATOR
        );
    }

    private static double calculateDistanceToTargetScoringPose() {
        final Translation2d currentTranslation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d targetTranslation = TARGET_SCORING_POSE.get().getTranslation();
        return currentTranslation.getDistance(targetTranslation);
    }

    public static FlippablePose2d calculateClosestOpenScoringPose() {
        final boolean[] scoredBranchesAtL4 = getScoredBranchesAtL4();
        final Pose2d currentRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();

        double closestDistance = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = null; //TODO: this stuff
//        for (FieldConstants.ReefClockPosition currentClockPosition : reefClockPositions) {
//            for (FieldConstants.ReefSide currentSide : FieldConstants.ReefSide.values()) {
//                if (shouldOnlyCheckOpenBranches && scoredBranchesAtL4[currentClockPosition.ordinal() * 2 + currentSide.ordinal()])
//                    continue;
//                final Pose2d reefSideScoringPose = CoralPlacingCommands.ScoringLevel.L4.calculateTargetPlacingPosition(currentClockPosition, currentSide).get();
//                final double distance = currentRobotPose.getTranslation().getDistance(reefSideScoringPose.getTranslation());
//                if (distance < closestDistance) {
//                    closestDistance = distance;
//                    if (shouldStayBehindAlgae)
//                        closestScoringPose = reefSideScoringPose.transformBy(new Transform2d(new Translation2d(0.1, 0), new Rotation2d()));
//                    else
//                        closestScoringPose = reefSideScoringPose;
//                }
//            }
//        }

        return closestScoringPose == null ? null : new FlippablePose2d(closestScoringPose, false);
    }

    private static Command getAddCurrentScoringBranchToScoredBranchesCommand() {
        return new InstantCommand(
                () -> {
                    final int branchNumber = getBranchNumberFromScoringPose(TARGET_SCORING_POSE.get());
                    SCORED_L4S[branchNumber].set(true);
                }
        );
    }

    private static int getBranchNumberFromScoringPose(Pose2d scoringPose) { //TODO: this stuff
//        for (FieldConstants.ReefClockPosition currentClockPosition : FieldConstants.ReefClockPosition.values()) {
//            for (FieldConstants.ReefSide currentSide : FieldConstants.ReefSide.values()) {
//                final Pose2d reefSideScoringPose = CoralPlacingCommands.ScoringLevel.L4.calculateTargetPlacingPosition(currentClockPosition, currentSide).get();
//                if (reefSideScoringPose.getTranslation().getDistance(scoringPose.getTranslation()) < 0.01)
//                    return currentClockPosition.ordinal() * 2 + currentSide.ordinal();
//            }
//        }

        return 0;
    }

    private static LoggedNetworkBoolean[] getEmptyL4LoggedNetworkBooleanArray() {
        final LoggedNetworkBoolean[] array = new LoggedNetworkBoolean[12];
        for (int i = 0; i < array.length; i++)
            array[i] = new LoggedNetworkBoolean("ScoredL4s/" + i, false);
        return array;
    }

    private static boolean[] getScoredBranchesAtL4() {
        final boolean[] booleanArray = new boolean[SCORED_L4S.length];

        for (int i = 0; i < booleanArray.length; i++)
            booleanArray[i] = SCORED_L4S[i].get();

        return booleanArray;
    }

    /**
     * Creates a command that resets the pose estimator's pose to the starting pose of the given autonomous as long as the robot is not enabled.
     *
     * @param autoName the name of the autonomous
     * @return a command that resets the robot's pose estimator pose to the start position of the given autonomous
     */
    public static Command getResetPoseToAutoPoseCommand(Supplier<String> autoName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    RobotContainer.ROBOT_POSE_ESTIMATOR.resetPose(getAutoStartPose(autoName.get()));
                }
        ).ignoringDisable(true);
    }

    /**
     * Gets the starting position of the target PathPlanner autonomous.
     *
     * @param autoName the name of the autonomous group
     * @return the staring pose of the autonomous
     */
    public static Pose2d getAutoStartPose(String autoName) {
        try {
            final Pose2d nonFlippedAutoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
            final FlippablePose2d flippedAutoStartPose = new FlippablePose2d(nonFlippedAutoStartPose, true);
            return flippedAutoStartPose.get();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            return new Pose2d();
        }
    }
}