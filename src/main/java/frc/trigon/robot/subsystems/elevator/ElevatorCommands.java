package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import lib.commands.ExecuteEndCommand;
import lib.commands.NetworkTablesCommand;

import java.util.Set;

public class ElevatorCommands {
    public static Command getDebbugingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.ELEVATOR::setTargetPositionRotations,
                false,
                Set.of(RobotContainer.ELEVATOR),
                "Debugging/ElevatorTargetPositionRotations"
        );
    }

    public static Command getPrepareForLoadCoralCommand() {
        return new StartEndCommand(
                RobotContainer.ELEVATOR::prepareForLoadCoral,
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        ).onlyIf(() -> RobotContainer.ARM.atState(ArmConstants.ArmState.REST));
    }

    public static Command getSetTargetStateCommand(ElevatorConstants.ElevatorState targetState) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetState(targetState),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }

    public static Command getSetTargetStateCommand(double targetPositionRotations) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetPositionRotations(targetPositionRotations),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }
}