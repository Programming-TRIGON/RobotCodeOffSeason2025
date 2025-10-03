package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
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
                getIntakeCoralCommand().until(RobotContainer.TRANSPORTER::hasCoral),
                getCollectionConfirmationCommand(),
                new InstantCommand(
                        () -> getLoadCoralCommand().schedule()
                )
        ).alongWith(new IntakeAssistCommand(OperatorConstants.DEFAULT_INTAKE_ASSIST_MODE));
    }

    public static Command getLoadCoralCommand() {
        return new ParallelCommandGroup(
                ArmElevatorCommands.getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.LOAD_CORAL),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.LOAD_CORAL)
        ).until(RobotContainer.END_EFFECTOR::hasGamePiece).unless(RobotContainer.END_EFFECTOR::hasGamePiece);
    }


    private static Command getIntakeCoralCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.COLLECT)
        );
    }


    private static Command getCollectionConfirmationCommand() {
        return new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(OperatorConstants.RUMBLE_DURATION_SECONDS, OperatorConstants.RUMBLE_POWER));
    }
}