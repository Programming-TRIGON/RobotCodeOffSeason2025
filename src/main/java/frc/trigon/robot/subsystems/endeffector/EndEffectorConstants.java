package frc.trigon.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import lib.hardware.misc.simplesensor.SimpleSensor;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import lib.hardware.simulation.SimpleMotorSimulation;
import lib.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class EndEffectorConstants {
    private static final int
            END_EFFECTOR_MOTOR_ID = 15,
            DISTANCE_SENSOR_CHANNEL = 4;
    private static final String
            END_EFFECTOR_MOTOR_NAME = "EndEffectorMotor",
            DISTANCE_SENSOR_NAME = "EndEffectorDistanceSensor";
    static final TalonFXMotor END_EFFECTOR_MOTOR = new TalonFXMotor(END_EFFECTOR_MOTOR_ID, END_EFFECTOR_MOTOR_NAME);
    static final SimpleSensor DISTANCE_SENSOR = SimpleSensor.createDigitalSensor(DISTANCE_SENSOR_CHANNEL, DISTANCE_SENSOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double END_EFFECTOR_GEAR_RATIO = 12.82;
    private static final int END_EFFECTOR_MOTOR_AMOUNT = 1;
    private static final DCMotor END_EFFECTOR_GEARBOX = DCMotor.getKrakenX60Foc(END_EFFECTOR_MOTOR_AMOUNT);
    private static final double
            END_EFFECTOR_MOMENT_OF_INERTIA = 0.003,
            END_EFFECTOR_MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    private static final SimpleMotorSimulation END_EFFECTOR_SIMULATION = new SimpleMotorSimulation(
            END_EFFECTOR_GEARBOX,
            END_EFFECTOR_GEAR_RATIO,
            END_EFFECTOR_MOMENT_OF_INERTIA
    );

    private static final DoubleSupplier DISTANCE_SENSOR_SIMULATION_SUPPLIER = () -> (SimulationFieldHandler.isHoldingCoral() && SimulationFieldHandler.isCoralInEndEffector()) || SimulationFieldHandler.isHoldingAlgae() ? 1 : 0;

    static final SpeedMechanism2d END_EFFECTOR_MECHANISM = new SpeedMechanism2d(
            "EndEffectorMechanism",
            END_EFFECTOR_MAXIMUM_DISPLAYABLE_VELOCITY
    );

    private static final double COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS = 0.2;
    static final BooleanEvent COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            DISTANCE_SENSOR::getBinaryValue
    ).debounce(COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS);
    static final double WHEEL_RADIUS_METERS = edu.wpi.first.math.util.Units.inchesToMeters(1.5);

    static {
        configureEndEffectorMotor();
        configureDistanceSensor();
    }

    private static void configureEndEffectorMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.RotorToSensorRatio = END_EFFECTOR_GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;

        END_EFFECTOR_MOTOR.applyConfiguration(config);
        END_EFFECTOR_MOTOR.setPhysicsSimulation(END_EFFECTOR_SIMULATION);

        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureDistanceSensor() {
        DISTANCE_SENSOR.setSimulationSupplier(DISTANCE_SENSOR_SIMULATION_SUPPLIER);
    }

    public enum EndEffectorState {
        REST(0),
        EJECT(4),
        LOAD_CORAL(-4),
        SCORE_CORAL(4),
        COLLECT_ALGAE(-4),
        SCORE_ALGAE(4),
        HOLD_ALGAE(-4);

        public final double targetVoltage;

        EndEffectorState(double targetVoltage) {
            this.targetVoltage = targetVoltage;
        }
    }
}
