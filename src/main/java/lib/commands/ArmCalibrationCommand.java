package lib.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class ArmCalibrationCommand extends Command {
    private static final LoggedNetworkNumber STARTING_VOLTAGE = new LoggedNetworkNumber("ArmCalibrationV2Command/ArmCalibrationStartingVoltage", 0.01);
    private static final double
            VOLTAGE_INCREMENT = 0.0001,
            POSITION_DEADBAND_ROTATIONS = 0.00000000001;
    private final Supplier<Double> positionSupplier;
    private final Consumer<Double> voltageConsumer;
    private double
            currentVoltage,
            provisionalKGMinimum,
            kGMinimum,
            kGMaximum,
            bestKGMinimum,
            bestKGMaximum;
    private Rotation2d
            previousPosition,
            bestGravityOffset;
    private boolean
            startedMoving = false,
            isStationary = true;

    public ArmCalibrationCommand(Supplier<Double> positionSupplier, Consumer<Double> voltageConsumer, Subsystem requirement) {
        this.positionSupplier = positionSupplier;
        this.voltageConsumer = voltageConsumer;

        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        this.currentVoltage = STARTING_VOLTAGE.get();
        this.provisionalKGMinimum = 0;
        this.kGMinimum = 0;
        this.kGMaximum = 0;
        this.bestKGMaximum = 0;
        this.bestKGMinimum = 0;
        this.previousPosition = Rotation2d.fromRotations(positionSupplier.get());
        this.bestGravityOffset = Rotation2d.fromRotations(positionSupplier.get());

        voltageConsumer.accept(STARTING_VOLTAGE.get());
    }

    @Override
    public void execute() {
        if (!startedMoving && isMoving())
            startedMoving = true;
        if (!isMoving() && !isStationary) {
            provisionalKGMinimum = currentVoltage;
            System.out.println(provisionalKGMinimum + " provisionalKGMinimum");
            increaseVoltage();
            isStationary = true;
        } else if (!isMoving()) {
            if (startedMoving)
                setKGValues();
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
        return Math.abs(positionSupplier.get() - bestGravityOffset.getRotations()) > Rotation2d.k180deg.getRotations();
    }

    private void setKGValues() {
        kGMaximum = currentVoltage;
        kGMinimum = provisionalKGMinimum;

        if (kGMaximum - kGMinimum > bestKGMaximum - bestKGMinimum) {
            bestKGMaximum = kGMaximum;
            bestKGMinimum = kGMinimum;
            bestGravityOffset = Rotation2d.fromRotations(positionSupplier.get());
        }
    }

    private void increaseVoltage() {
        currentVoltage += VOLTAGE_INCREMENT;
        voltageConsumer.accept(currentVoltage);
    }

    private boolean isMoving() {
        return Math.abs(previousPosition.getRotations() - positionSupplier.get()) > POSITION_DEADBAND_ROTATIONS;
    }

    private double calculateKG() {
        return (bestKGMinimum + bestKGMaximum) / 2;
    }

    private double calculateKS() {
        return (bestKGMaximum - bestKGMinimum) / 2;
    }

    private void printResults(double kG, double kS) {
        System.out.println("Gravity Offset (rotations): " + bestGravityOffset.getRotations());
        System.out.println("Minimum kG: " + bestKGMinimum);
        System.out.println("Maximum kG: " + bestKGMaximum);
        System.out.println("kG: " + kG);
        System.out.println("kS: " + kS);
    }

    private void logResults(double kG, double kS) {
        Logger.recordOutput("ArmCalibrationV2Command/GravityOffset", bestGravityOffset);
        Logger.recordOutput("ArmCalibrationV2Command/kG", kG);
        Logger.recordOutput("ArmCalibrationV2Command/kS", kS);
    }
}