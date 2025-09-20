package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import lib.commands.ExecuteEndCommand;
import lib.commands.NetworkTablesCommand;

import java.util.Set;
import java.util.function.Supplier;

public class ElevatorCommands {
    public static Command getDebbugingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.ELEVATOR::setTargetPositionRotations,
                false,
                Set.of(RobotContainer.ELEVATOR),
                "Debugging/ElevatorTargetPositionRotations"
        );
    }

    public static Command getSetTargetStateCommand(Supplier<ElevatorConstants.ElevatorState> targetState) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetState(targetState.get()),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
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

    public static Command getPrepareStateCommand(Supplier<ElevatorConstants.ElevatorState> targetState) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ELEVATOR.prepareState(targetState.get()),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }

    public static Command getPrepareStateCommand(ElevatorConstants.ElevatorState targetState) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ELEVATOR.prepareState(targetState),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR);
    }
}