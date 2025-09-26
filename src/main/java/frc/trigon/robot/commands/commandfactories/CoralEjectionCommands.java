package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import frc.trigon.robot.subsystems.arm.ArmElevatorCommands;
import frc.trigon.robot.subsystems.arm.ArmElevatorConstants;
import frc.trigon.robot.subsystems.endeffector.EndEffectorCommands;
import frc.trigon.robot.subsystems.endeffector.EndEffectorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;

public class CoralEjectionCommands {
    public static Command getCoralEjectionCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getEjectCoralFromIntakeCommand(),
                getEjectCoralFromEndEffectorCommand(),
                () -> RobotContainer.TRANSPORTER.hasCoral() || RobotContainer.INTAKE.hasCoral()
        ).onlyIf(SimulationFieldHandler::isHoldingCoral);
    }

    private static Command getEjectCoralFromIntakeCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.EJECT),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.EJECT)
        );
    }

    private static Command getEjectCoralFromEndEffectorCommand() {
        return new SequentialCommandGroup(
                ArmElevatorCommands.getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.EJECT).until(RobotContainer.ARM_ELEVATOR::atTargetState),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.EJECT)
        );
    }
}