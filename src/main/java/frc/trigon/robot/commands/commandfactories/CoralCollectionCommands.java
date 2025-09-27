package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;

public class CoralCollectionCommands {

    public static Command getCoralCollectionCommand() {
        return new SequentialCommandGroup(
                getIntakeSequenceCommand(),
                new InstantCommand(
                        () -> getLoadCoralCommand().schedule()
                )
        );
        // new IntakeAssistCommand(OperatorConstants.DEFAULT_INTAKE_ASSIST_MODE)
    }

    private static Command getLoadCoralCommand() {
        return new ParallelCommandGroup(
                ArmCommands.getSetTargetStateCommand(ArmConstants.ArmState.LOAD_CORAL),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.LOAD_CORAL)
        ).until(RobotContainer.ARM::hasGamePiece).asProxy();
    }

    private static Command getIntakeSequenceCommand() {
        return new SequentialCommandGroup(
                getInitiateCollectionCommand().until(RobotContainer.INTAKE::hasCoral),
                new InstantCommand(() -> getAlignCoralCommand().schedule()).alongWith(getCollectionConfirmationCommand())
        ).until(RobotContainer.INTAKE::hasCoral);
    }

    private static Command getInitiateCollectionCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.COLLECT)
        );
    }

    private static Command getAlignCoralCommand() {
        return new SequentialCommandGroup(
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.ALIGN_CORAL).until(RobotContainer.TRANSPORTER::hasCoral),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.HOLD_CORAL)
        );
    }

    private static Command getCollectionConfirmationCommand() {
        return new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(OperatorConstants.RUMBLE_DURATION_SECONDS, OperatorConstants.RUMBLE_POWER));
    }
}