package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.armelevator.ArmElevatorCommands;
import frc.trigon.robot.subsystems.armelevator.ArmElevatorConstants;
import frc.trigon.robot.subsystems.endeffector.EndEffectorCommands;
import frc.trigon.robot.subsystems.endeffector.EndEffectorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;

public class CoralCollectionCommands {

    public static Command getCoralCollectionCommand() {
        return new SequentialCommandGroup(
                getIntakeSequenceCommand(),
                new InstantCommand(
                        () -> {
                            if (!AlgaeManipulationCommands.isHoldingAlgae())
                                getLoadCoralCommand().schedule();
                        }
                )
        );
        // new IntakeAssistCommand(OperatorConstants.DEFAULT_INTAKE_ASSIST_MODE)
    }

    public static Command getLoadCoralCommand() {
        return new ParallelCommandGroup(
                ArmElevatorCommands.getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.LOAD_CORAL),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.LOAD_CORAL)
        ).until(RobotContainer.END_EFFECTOR::hasGamePiece).onlyWhile(() -> !RobotContainer.END_EFFECTOR.hasGamePiece());
    }

    public static Command getUnloadCoralCommand() {
        return new ParallelCommandGroup(
                ArmElevatorCommands.getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.UNLOAD_CORAL),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.UNLOAD_CORAL)
        ).until(() -> !RobotContainer.END_EFFECTOR.hasGamePiece() && RobotContainer.INTAKE.hasCoral());
    }

    private static Command getIntakeSequenceCommand() {
        return getInitiateCollectionCommand().until(RobotContainer.TRANSPORTER::hasCoral);
    }

    private static Command getInitiateCollectionCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT),
                TransporterCommands.getSetTargetStateWithPulsesCommand(TransporterConstants.TransporterState.COLLECT)
        );
    }

    private static Command getAlignCoralCommand() {
        return new SequentialCommandGroup(
                TransporterCommands.getSetTargetStateWithPulsesCommand(TransporterConstants.TransporterState.ALIGN_CORAL).until(RobotContainer.TRANSPORTER::hasCoral),
                TransporterCommands.getSetTargetStateWithPulsesCommand(TransporterConstants.TransporterState.HOLD_CORAL)
        );
    }

    private static Command getCollectionConfirmationCommand() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(OperatorConstants.RUMBLE_DURATION_SECONDS, OperatorConstants.RUMBLE_POWER))
        );
    }
}