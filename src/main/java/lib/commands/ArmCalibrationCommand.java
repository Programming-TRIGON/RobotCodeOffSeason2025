package lib.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class ArmCalibrationCommand extends Command {
    private static final double
            STARTING_VOLTAGE = 0.002,
            VOLTAGE_INCREMENT = 0.0001,
            POSITION_DEADBAND_ROTATIONS = 0.00000000001;
    private final Supplier<Double> positionSupplier;
    private final Consumer<Double> voltageConsumer;
    private double
            currentVoltage,
            provisionalKGMinimum,
            kGMinimum,
            kGMaximum;
    private Rotation2d
            previousPosition,
            gravityOffset;
    private boolean isStationary = true;

    public ArmCalibrationCommand(Supplier<Double> positionSupplier, Consumer<Double> voltageConsumer, Subsystem Requirement) {
        addRequirements(Requirement);
        this.positionSupplier = positionSupplier;
        this.voltageConsumer = voltageConsumer;
        this.currentVoltage = STARTING_VOLTAGE;
        this.provisionalKGMinimum = 0;
        this.kGMinimum = 0;
        this.kGMaximum = 0;
        this.previousPosition = Rotation2d.fromRotations(positionSupplier.get());
        this.gravityOffset = Rotation2d.fromRotations(positionSupplier.get());
    }

    @Override
    public void initialize() {
        voltageConsumer.accept(STARTING_VOLTAGE);
    }

    @Override
    public void execute() {
        if (!isMoving() && !isStationary) {
            provisionalKGMinimum = currentVoltage;
            System.out.println(provisionalKGMinimum + " provisionalKGMinimum");
            increaseVoltage();
            isStationary = true;
            gravityOffset = Rotation2d.fromRotations(positionSupplier.get());
        } else if (!isMoving()) {
            kGMaximum = currentVoltage;
            kGMinimum = provisionalKGMinimum;
            System.out.println(kGMaximum + " kGMaximum");
            increaseVoltage();
        } else {
            System.out.println(isMoving());
            isStationary = false;
        }
        previousPosition = Rotation2d.fromRotations(positionSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        final double kG = calculateKG();
        final double kS = calculateKS();
        printResults(kG, kS);
        logResults(kG, kS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(positionSupplier.get() - gravityOffset.getRotations()) > Rotation2d.k180deg.getRotations();
    }

    private void increaseVoltage() {
        currentVoltage += VOLTAGE_INCREMENT;
        voltageConsumer.accept(currentVoltage);
    }

    private boolean isMoving() {
        return Math.abs(previousPosition.getRotations() - positionSupplier.get()) > POSITION_DEADBAND_ROTATIONS;
    }

    private double calculateKG() {
        return (kGMinimum + kGMaximum) / 2;
    }

    private double calculateKS() {
        return (kGMaximum - kGMinimum) / 2;
    }

    private void printResults(double kG, double kS) {
        System.out.println("Gravity Offset (rotations): " + gravityOffset.getRotations());
        System.out.println("Minimum kG: " + kGMinimum);
        System.out.println("Maximum kG: " + kGMaximum);
        System.out.println("kG: " + kG);
        System.out.println("kS: " + kS);
    }

    private void logResults(double kG, double kS) {
        Logger.recordOutput("ArmCalibrationV2Command/GravityOffset", gravityOffset);
        Logger.recordOutput("ArmCalibrationV2Command/kG", kG);
        Logger.recordOutput("ArmCalibrationV2Command/kS", kS);
    }
}