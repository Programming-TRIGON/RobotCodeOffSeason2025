package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;

public class CollectionCommand {
    public static Command getCollectCommand() {
        return new SequentialCommandGroup(
                getInitiateCollectionCommand(),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.ALIGN_CORAL).until(RobotContainer.TRANSPORTER::hasCoral),
                new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(OperatorConstants.RUMBLE_DURATION_SECONDS, OperatorConstants.RUMBLE_POWER))
        );
    }

    private static Command getInitiateCollectionCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.COLLECT)
        ).until(RobotContainer.INTAKE::hasGamePiece);
    }
}