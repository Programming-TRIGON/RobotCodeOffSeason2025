package frc.trigon.robot.subsystems.transporter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import trigon.hardware.misc.simplesensor.SimpleSensor;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import trigon.hardware.simulation.SimpleMotorSimulation;
import trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class TransporterConstants {
    private static final int
            RIGHT_MOTOR_ID = 11,
            LEFT_MOTOR_ID = 12,
            BEAM_BREAK_CHANNEL = 3;
    private static final String
            RIGHT_MOTOR_NAME = "TransporterRightMotor",
            LEFT_MOTOR_NAME = "TransporterLeftMotor",
            BEAM_BREAK_NAME = "TransporterBeamBreak";
    static final TalonFXMotor
            RIGHT_MOTOR = new TalonFXMotor(RIGHT_MOTOR_ID, RIGHT_MOTOR_NAME),
            LEFT_MOTOR = new TalonFXMotor(LEFT_MOTOR_ID, LEFT_MOTOR_NAME);
    static final SimpleSensor BEAM_BREAK = SimpleSensor.createDigitalSensor(BEAM_BREAK_CHANNEL, BEAM_BREAK_NAME);

    private static double GEAR_RATIO = 3;
    static boolean FOC_ENABLED = true;
    private static int MOTOR_AMOUNT = 1;
    private static DCMotor GEARBOX = DCMotor.getKrakenX60(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation RIGHT_MOTOR_SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    ),
            LEFT_MOTOR_SIMULATION = new SimpleMotorSimulation(
                    GEARBOX,
                    GEAR_RATIO,
                    MOMENT_OF_INERTIA
            );
    private static final DoubleSupplier BEAM_BREAK_SIMULATION_SUPPLIER = () -> 0; //TODO: implement

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d RIGHT_MOTOR_MECHANSIM = new SpeedMechanism2d(
            "TranporterRightMotorMechanism",
            MAXIMUM_DISPLAYABLE_VELOCITY
    ),
            LEFT_MOTOR_MECHANSIM = new SpeedMechanism2d(
                    "TranporterLeftMotorMechanism",
                    MAXIMUM_DISPLAYABLE_VELOCITY
            );

    private static final double HAS_CORAL_DEBOUNCE_TIME_SECONDS = 0.2;
    static final BooleanEvent HAS_CORAL_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            BEAM_BREAK::getBinaryValue
    ).debounce(HAS_CORAL_DEBOUNCE_TIME_SECONDS);

    static {
        configureMotor(RIGHT_MOTOR, RIGHT_MOTOR_SIMULATION);
        configureMotor(LEFT_MOTOR, LEFT_MOTOR_SIMULATION);
        configureBeamBreak();
    }

    private static void configureMotor(TalonFXMotor motor, SimpleMotorSimulation simulation) {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motor.applyConfiguration(config);
        motor.setPhysicsSimulation(simulation);

        motor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        motor.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }


    private static void configureBeamBreak() {
        BEAM_BREAK.setSimulationSupplier(BEAM_BREAK_SIMULATION_SUPPLIER);
    }

    public enum TransporterState {
        REST(0, 0),
        COLLECT(5, 5),
        ALIGN_CORAL(5, 6),
        HOLD_CORAL(1, 1),
        EJECT(-5, -5);

        public final double
                targetRightMotorVoltage,
                targetLeftMotorVoltage;

        TransporterState(double targetRightMotorVoltage, double targetLeftMotorVoltage) {
            this.targetRightMotorVoltage = targetRightMotorVoltage;
            this.targetLeftMotorVoltage = targetLeftMotorVoltage;
        }
    }
}