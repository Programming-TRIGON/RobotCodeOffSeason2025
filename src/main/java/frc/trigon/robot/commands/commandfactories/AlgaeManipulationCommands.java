package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.WaitUntilChangeCommand;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.ArmElevatorCommands;
import frc.trigon.robot.subsystems.arm.ArmElevatorConstants;
import frc.trigon.robot.subsystems.endeffector.EndEffectorCommands;
import frc.trigon.robot.subsystems.endeffector.EndEffectorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.hardware.RobotHardwareStats;
import lib.hardware.misc.leds.LEDCommands;
import lib.utilities.flippable.Flippable;
import lib.utilities.flippable.FlippablePose2d;
import lib.utilities.flippable.FlippableRotation2d;
import lib.utilities.flippable.FlippableTranslation2d;

import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class AlgaeManipulationCommands {
    private static boolean
            IS_HOLDING_ALGAE = false,
            SHOULD_COLLECT_FROM_LOLLIPOP = false;
    private static final PIDController SMALL_POSITION_ALIGN_PID_CONTROLLER =
            RobotHardwareStats.isSimulation() ?
                    new PIDController(3, 0, 0) :
                    new PIDController(3, 0, 0);

    public static boolean isHoldingAlgae() {
        return IS_HOLDING_ALGAE;
    }

    public static void toggleLollipopCollection() {
        SHOULD_COLLECT_FROM_LOLLIPOP = !SHOULD_COLLECT_FROM_LOLLIPOP;
    }

    public static Command getFloorAlgaeCollectionCommand() {
        return new SequentialCommandGroup(
                GeneralCommands.getResetFlipArmOverrideCommand(),
                new InstantCommand(() -> {
                    IS_HOLDING_ALGAE = true;
                    SHOULD_COLLECT_FROM_LOLLIPOP = false;
                }),
                CoralCollectionCommands.getUnloadCoralCommand().onlyIf(RobotContainer.END_EFFECTOR::hasGamePiece),
                getInitiateFloorAlgaeCollectionCommand().until(RobotContainer.END_EFFECTOR::hasGamePiece),
                GeneralCommands.getResetFlipArmOverrideCommand(),
                getScoreAlgaeCommand().alongWith(getAlgaeCollectionConfirmationCommand())
                        .until(() -> !RobotContainer.END_EFFECTOR.hasGamePiece() && !isScoreAlgaeButtonPressed())
        ).finallyDo(() -> IS_HOLDING_ALGAE = false);
    }

    public static Command getReefAlgaeCollectionCommand() {
        return new SequentialCommandGroup(
                GeneralCommands.getResetFlipArmOverrideCommand(),
                new InstantCommand(() -> IS_HOLDING_ALGAE = true),
                CoralCollectionCommands.getUnloadCoralCommand().onlyIf(RobotContainer.END_EFFECTOR::hasGamePiece),
                getInitiateReefAlgaeCollectionCommand().until(RobotContainer.END_EFFECTOR::hasGamePiece),
                GeneralCommands.getResetFlipArmOverrideCommand(),
                getScoreAlgaeCommand().alongWith(getAlgaeCollectionConfirmationCommand())
                        .until(() -> !RobotContainer.END_EFFECTOR.hasGamePiece() && !isScoreAlgaeButtonPressed())
        )
                .alongWith(getAlignToReefCommand())
                .finallyDo(() -> IS_HOLDING_ALGAE = false);
    }

    private static Command getAlignToReefCommand() {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        AlgaeManipulationCommands::calculateClosestAlgaeCollectionPose,
                        AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS
                ),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> fieldRelativePowersToSelfRelativeXPower(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> SMALL_POSITION_ALIGN_PID_CONTROLLER.calculate(calculateReefRelativeYOffset(), 0),
                        () -> calculateClosestAlgaeCollectionPose().getRotation()
                )
        ).raceWith(
                new WaitUntilCommand(RobotContainer.END_EFFECTOR::hasGamePiece),
                new WaitUntilCommand(OperatorConstants.STOP_REEF_ALGAE_ALIGN_TRIGGER)
        ).onlyIf(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY).asProxy();
    }

    private static Command getScoreAlgaeCommand() {
        return new SelectCommand<>(
                Map.of(
                        0, getHoldAlgaeCommand(),
                        1, getScoreInNetCommand(),
                        2, getScoreInProcessorCommand()
                ),
                AlgaeManipulationCommands::getAlgaeScoreMethodSelector
        ).raceWith(new WaitUntilChangeCommand<>(AlgaeManipulationCommands::isScoreAlgaeButtonPressed)).repeatedly();
    }

    private static Command getHoldAlgaeCommand() {
        return new ParallelCommandGroup(
                ArmElevatorCommands.getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.REST_WITH_ALGAE),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.HOLD_ALGAE)
        );
    }

    private static Command getScoreInNetCommand() {
        return new ParallelRaceGroup(
                getArmNetSequenceCommand(AlgaeManipulationCommands::shouldReverseNetScore),
                getDriveToNetCommand()
        );
    }

    private static Command getScoreInProcessorCommand() {
        return new ParallelCommandGroup(
                getArmProcessorSequenceCommand(),
                getDriveToProcessorCommand()
        ).finallyDo(GeneralCommands.getFieldRelativeDriveCommand()::schedule);
    }

    private static Command getArmNetSequenceCommand(BooleanSupplier shouldReverseScore) {
        return new SequentialCommandGroup(
                GeneralCommands.getFlippableOverridableArmCommand(ArmElevatorConstants.ArmElevatorState.SCORE_NET, true, shouldReverseScore).until(OperatorConstants.CONTINUE_TRIGGER),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.SCORE_ALGAE)
        );
    }

    private static Command getArmProcessorSequenceCommand() {
        return new SequentialCommandGroup(
                GeneralCommands.getFlippableOverridableArmCommand(ArmElevatorConstants.ArmElevatorState.SCORE_PROCESSOR, true).until(OperatorConstants.CONTINUE_TRIGGER),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.SCORE_ALGAE)
        );
    }

    private static Command getDriveToNetCommand() {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        AlgaeManipulationCommands::calculateClosestNetScoringPose,
                        AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS
                ).until(() -> RobotContainer.SWERVE.atPose(calculateClosestNetScoringPose())),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> -SMALL_POSITION_ALIGN_PID_CONTROLLER.calculate(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getX(), calculateClosestNetScoringPose().get().getX()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> new FlippableRotation2d(shouldReverseNetScore() ? Rotation2d.kZero : Rotation2d.k180deg, true)
                )
        ).asProxy().onlyWhile(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY);
    }

    private static Command getDriveToProcessorCommand() {
        return SwerveCommands.getDriveToPoseCommand(
                        () -> FieldConstants.FLIPPABLE_PROCESSOR_SCORE_POSE,
                        AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS
                ).asProxy()
                .until(() -> RobotContainer.SWERVE.atPose(FieldConstants.FLIPPABLE_PROCESSOR_SCORE_POSE))
                .onlyWhile(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY);
    }

    private static int getAlgaeScoreMethodSelector() {
        if (OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER.getAsBoolean())
            return 1;
        if (OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean())
            return 2;
        return 0;
    }

    private static boolean shouldReverseNetScore() {
        final Rotation2d swerveAngle = RobotContainer.SWERVE.getDriveRelativeAngle();
        return swerveAngle.getDegrees() < 90 && swerveAngle.getDegrees() > -90;
    }

    private static Command getInitiateFloorAlgaeCollectionCommand() {
        return new ConditionalCommand(
                getCollectAlgaeFromLollipopSequenceCommand(),
                getCollectAlgaeFromFloorSequenceCommand(),
                () -> SHOULD_COLLECT_FROM_LOLLIPOP
        ).raceWith(
                new WaitUntilChangeCommand<>(() -> SHOULD_COLLECT_FROM_LOLLIPOP)
        ).repeatedly();
    }

    private static Command getCollectAlgaeFromLollipopSequenceCommand() {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        GeneralCommands.getFlippableOverridableArmCommand(ArmElevatorConstants.ArmElevatorState.COLLECT_ALGAE_LOLLIPOP, false),
                        EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.COLLECT_ALGAE)
                ),
                getIntakeCoralFromLollipopCommand()
                        .onlyIf(() -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE)
                        .until(() -> !OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE)
                        .repeatedly()
        );
    }

    private static Command getCollectAlgaeFromFloorSequenceCommand() {
        return new ParallelCommandGroup(
                ArmElevatorCommands.getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.COLLECT_ALGAE_FLOOR),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.COLLECT_ALGAE)
        );
    }

    private static Command getIntakeCoralFromLollipopCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.OPEN_REST),
                CoralCollectionCommands.getCoralCollectionCommand(),
                RobotContainer.TRANSPORTER::hasCoral
        ).asProxy();
    }

    private static Command getInitiateReefAlgaeCollectionCommand() {
        return new ParallelCommandGroup(
                GeneralCommands.getFlippableOverridableArmCommand(OperatorConstants.REEF_CHOOSER::getArmElevatorAlgaeCollectionState, false, CoralPlacingCommands::shouldReverseScore),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.COLLECT_ALGAE)
        ).raceWith(
                new WaitUntilChangeCommand<>(OperatorConstants.REEF_CHOOSER::getArmElevatorAlgaeCollectionState),
                new WaitUntilChangeCommand<>(OperatorConstants.REEF_CHOOSER::getArmElevatorAlgaeCollectionState)
        ).repeatedly();
    }

    private static Command getAlgaeCollectionConfirmationCommand() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(OperatorConstants.RUMBLE_DURATION_SECONDS, OperatorConstants.RUMBLE_POWER)),
                LEDCommands.getAnimateCommand(LEDConstants.ALGAE_COLLECTION_CONFIRMATION_ANIMATION_SETTINGS)
        );
    }

    private static FlippablePose2d calculateClosestNetScoringPose() {
        final Translation2d robotTranslation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        if (Flippable.isRedAlliance() ? robotTranslation.getY() < FieldConstants.FLIPPABLE_NET_SCORE_POSE.get().getY() : robotTranslation.getY() > FieldConstants.FLIPPABLE_NET_SCORE_POSE.get().getY())
            return new FlippablePose2d(new Pose2d(
                    FieldConstants.FLIPPABLE_NET_SCORE_POSE.get().getX(),
                    robotTranslation.getY(),
                    shouldReverseNetScore() ? Rotation2d.k180deg : Rotation2d.kZero
            ), false);
        return FieldConstants.FLIPPABLE_NET_SCORE_POSE;
    }

    private static FlippablePose2d calculateClosestAlgaeCollectionPose() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;
        final Transform2d reefCenterToBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_ALGAE_COLLECTION_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d());
        final List<Pose2d> scoringPoses = new java.util.ArrayList<>(List.of());

        for (Rotation2d reefClockAngle : reefClockAngles) {
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, reefClockAngle);
            scoringPoses.add(reefCenterAtTargetRotation.transformBy(reefCenterToBranchScoringPose));
        }

        final Pose2d closestScoringPose = robotPose.nearest(scoringPoses);
        return new FlippablePose2d(closestScoringPose.getTranslation(), closestScoringPose.getRotation().plus(CoralPlacingCommands.shouldReverseScore() ? Rotation2d.k180deg : new Rotation2d()).getRadians(), false);
    }

    private static boolean isScoreAlgaeButtonPressed() {
        return OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER.getAsBoolean() ||
                OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean();
    }

    private static double fieldRelativePowersToSelfRelativeXPower(double xPower, double yPower) {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();
        final double xValue = CommandConstants.calculateDriveStickAxisValue(xPower);
        final double yValue = CommandConstants.calculateDriveStickAxisValue(yPower);

        return (xValue * robotHeading.getCos()) + (yValue * robotHeading.getSin());
    }

    private static double calculateReefRelativeYOffset() {
        final Pose2d target = calculateClosestAlgaeCollectionPose().get();
        final Pose2d robot = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return robot.relativeTo(target).getY();
    }
}