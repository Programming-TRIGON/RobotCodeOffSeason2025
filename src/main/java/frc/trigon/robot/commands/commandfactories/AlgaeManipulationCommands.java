package frc.trigon.robot.commands.commandfactories;

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
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.hardware.misc.leds.LEDCommands;
import lib.utilities.flippable.FlippablePose2d;
import lib.utilities.flippable.FlippableRotation2d;
import lib.utilities.flippable.FlippableTranslation2d;

import java.util.Map;

public class AlgaeManipulationCommands {
    public static Command getAlgaeCollectionCommandCommand() {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getUnloadCoralCommand().onlyIf(RobotContainer.ARM::hasGamePiece),
                getInitiateAlgaeCollectionCommand().until(RobotContainer.ARM::hasGamePiece),
                new InstantCommand(() -> getScoreAlgaeCommand().schedule()).alongWith(getAlgaeCollectionConfirmationCommand())
        ).alongWith(getAlignToReefCommand());
    }

    private static Command getAlignToReefCommand() {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        AlgaeManipulationCommands::calculateClosestAlgaeCollectionPose,
                        AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS
                ),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> fieldRelativePowersToSelfRelativeXPower(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> 0,
                        () -> calculateClosestAlgaeCollectionPose().getRotation()
                )
        ).raceWith(
                new WaitCommand(1).andThen(new WaitUntilCommand(RobotContainer.ARM::hasGamePiece)),
                new WaitUntilCommand(OperatorConstants.STOP_ALGAE_AUTO_ALIGN_OVERRIDE_TRIGGER)
        ).onlyIf(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY).asProxy();
    }

    private static Command getScoreAlgaeCommand() {
        return new SelectCommand<>(
                Map.of(
                        0, getHoldAlgaeCommand(),
                        1, getScoreInNetCommand(),
                        2, getScoreInProcessorCommand(),
                        3, getEjectAlgaeCommand()
                ),
                AlgaeManipulationCommands::getAlgaeScoreMethodSelector
        ).raceWith(new WaitUntilChangeCommand<>(AlgaeManipulationCommands::isScoreAlgaeButtonPressed)).repeatedly().until(() -> !RobotContainer.ARM.hasGamePiece());
    }

    private static Command getScoreInNetCommand() {
        return new ParallelRaceGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_NET),
                getArmScoringSequenceCommand(ArmConstants.ArmState.SCORE_NET),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> new FlippableRotation2d(Rotation2d.k180deg, true)
                ).asProxy().onlyWhile(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY)
        );
    }

    private static Command getScoreInProcessorCommand() {
        return new ParallelRaceGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_PROCESSOR),
                getArmScoringSequenceCommand(ArmConstants.ArmState.SCORE_PROCESSOR),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> new FlippableRotation2d(Rotation2d.kCW_90deg, true)
                ).asProxy().onlyWhile(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY)
        );
    }

    private static Command getEjectAlgaeCommand() {
        return new ParallelCommandGroup(
                ArmCommands.getSetTargetStateCommand(ArmConstants.ArmState.EJECT_ALGAE),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST)
        ).until(() -> !RobotContainer.ARM.hasGamePiece());
    }

    private static Command getArmScoringSequenceCommand(ArmConstants.ArmState scoreState) {
        return new SequentialCommandGroup(
                ArmCommands.getPrepareForStateCommand(scoreState, CoralPlacingCommands.shouldReverseScore()).until(OperatorConstants.CONTINUE_TRIGGER),
                ArmCommands.getSetTargetStateCommand(scoreState, CoralPlacingCommands.shouldReverseScore())
        );
    }

    private static int getAlgaeScoreMethodSelector() {
        if (OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER.getAsBoolean())
            return 1;
        if (OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean())
            return 2;
        if (OperatorConstants.EJECT_ALGAE_TRIGGER.getAsBoolean())
            return 3;
        return 0;
    }

    private static Command getInitiateAlgaeCollectionCommand() {
        return new ParallelCommandGroup(
                ArmCommands.getSetTargetStateCommand(CoralPlacingCommands.REEF_CHOOSER.getArmAlgaeCollectionState()),
                ElevatorCommands.getSetTargetStateCommand(CoralPlacingCommands.REEF_CHOOSER.getElevatorAlgaeCollectionState())
        );
    }

    private static Command getHoldAlgaeCommand() {
        return new ParallelCommandGroup(
                ArmCommands.getSetTargetStateCommand(ArmConstants.ArmState.HOLD_ALGAE),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST_WITH_ALGAE)
        );
    }

    private static Command getAlgaeCollectionConfirmationCommand() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(OperatorConstants.RUMBLE_DURATION_SECONDS, OperatorConstants.RUMBLE_POWER)),
                LEDCommands.getAnimateCommand(LEDConstants.ALGAE_COLLECTION_CONFIRMATION_ANIMATION_SETTINGS) //TODO: add LEDs
        );
    }

    private static FlippablePose2d calculateClosestAlgaeCollectionPose() {
        final Translation2d robotPositionOnField = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;
        final Transform2d reefCenterToBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_ALGAE_COLLECTION_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d());

        double distanceFromClosestScoringPoseMeters = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = new Pose2d();
        for (final Rotation2d targetRotation : reefClockAngles) {
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, targetRotation);
            final Pose2d currentScoringPose = reefCenterAtTargetRotation.transformBy(reefCenterToBranchScoringPose);
            final double distanceFromCurrentScoringPoseMeters = currentScoringPose.getTranslation().getDistance(robotPositionOnField);
            if (distanceFromCurrentScoringPoseMeters < distanceFromClosestScoringPoseMeters) {
                distanceFromClosestScoringPoseMeters = distanceFromCurrentScoringPoseMeters;
                closestScoringPose = currentScoringPose;
            }
        }

        return new FlippablePose2d(closestScoringPose.rotateBy(CoralPlacingCommands.shouldReverseScore() ? Rotation2d.k180deg : new Rotation2d()), false);
    }

    private static boolean isScoreAlgaeButtonPressed() {
        return OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER.getAsBoolean() ||
                OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean() ||
                OperatorConstants.EJECT_ALGAE_TRIGGER.getAsBoolean();
    }

    private static double fieldRelativePowersToSelfRelativeXPower(double xPower, double yPower) {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();
        final double xValue = CommandConstants.calculateDriveStickAxisValue(xPower);
        final double yValue = CommandConstants.calculateDriveStickAxisValue(yPower);

        return (xValue * robotHeading.getCos()) + (yValue * robotHeading.getSin());
    }
}