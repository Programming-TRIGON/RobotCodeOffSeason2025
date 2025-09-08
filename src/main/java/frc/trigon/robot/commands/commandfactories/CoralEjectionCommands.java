package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;

public class CoralEjectionCommands {
    public static Command getCoralEjectionCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getEjectCoralFromIntakeCommand(),
                getEjectCoralFromArmCommand(),
                () -> RobotContainer.TRANSPORTER.hasCoral() || RobotContainer.INTAKE.hasCoral()
        );
    }

    private static Command getEjectCoralFromIntakeCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.EJECT),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.EJECT)
        );
    }

    private static Command getEjectCoralFromArmCommand() {
        return new PrintCommand("Ejecting coral from arm - Not implemented yet"); //TODO: Shamya do this
    }
}