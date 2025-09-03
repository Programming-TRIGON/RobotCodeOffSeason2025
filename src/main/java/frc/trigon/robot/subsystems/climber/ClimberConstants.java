package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import trigon.hardware.RobotHardwareStats;
import trigon.hardware.misc.servo.Servo;
import trigon.hardware.misc.simplesensor.SimpleSensor;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import trigon.hardware.simulation.SimpleMotorSimulation;
import trigon.utilities.mechanisms.SingleJointedArmMechanism2d;

import java.util.function.DoubleSupplier;

public class ClimberConstants {
    private static final int
            MOTOR_ID = 18,
            REVERSE_LIMIT_SENSOR_CHANNEL = 0,
            RIGHT_SERVO_CHANNEL = 1,
            LEFT_SERVO_CHANNEL = 2,
            CAGE_SENSOR_CHANNEL = 3;
    private static final String
            MOTOR_NAME = "ClimberMotor",
            REVERSE_LIMIT_SWITCH_NAME = "ClimberReverseLimitSwitch",
            RIGHT_SERVO_NAME = "ClimberRightServo",
            LEFT_SERVO_NAME = "ClimberLeftServo",
            CAGE_SENSOR_NAME = "ClimberCageSensor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final Servo
            RIGHT_SERVO = new Servo(RIGHT_SERVO_CHANNEL, RIGHT_SERVO_NAME),
            LEFT_SERVO = new Servo(LEFT_SERVO_CHANNEL, LEFT_SERVO_NAME);
    private static final SimpleSensor
            REVERSE_LIMIT_SENSOR = SimpleSensor.createDigitalSensor(REVERSE_LIMIT_SENSOR_CHANNEL, REVERSE_LIMIT_SWITCH_NAME),
            CAGE_SENSOR = SimpleSensor.createDigitalSensor(CAGE_SENSOR_CHANNEL, CAGE_SENSOR_NAME);

    static final int
            GROUNDED_PID_SLOT = 0,
            ON_CAGE_PID_SLOT = 1;
    private static final double GEAR_RATIO = 37.5;
    private static final double REVERSE_LIMIT_SENSOR_RESET_POSITION = 0;
    private static final double FORWARD_SOFT_LIMIT_POSITION_ROTATIONS = 3;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor
            GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);//TODO: KrakenX44
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation MOTOR_SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Second),
            Units.Volts.of(5),
            null
    );

    static final Pose3d CLIMBER_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );
    static final double CLIMBER_VISUALIZATION_POSITION_SCALAR = 0.1;
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            "ClimberMechanism",
            Color.kGreen
    );

    public static final double MAXIMUM_MANUAL_CONTROL_VOLTAGE = 4;
    private static final double
            HAS_CAGE_DEBOUNCE_TIME_SECONDS = 0.5,
            REVERSE_LIMIT_DEBOUNCE_TIME_SECONDS = 0.1;
    static final BooleanEvent HAS_CAGE_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            CAGE_SENSOR::getBinaryValue
    ).debounce(HAS_CAGE_DEBOUNCE_TIME_SECONDS);


    final BooleanEvent REVERSE_LIMIT_SENSOR_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            REVERSE_LIMIT_SENSOR::getBinaryValue
    ).debounce(REVERSE_LIMIT_DEBOUNCE_TIME_SECONDS);
    private static final DoubleSupplier REVERSE_LIMIT_SWITCH_SIMULATION_SUPPLIER = () -> 0;
    static final double CLIMBER_TOLERANCE_ROTATIONS = 0.01;

    static {
        configureMotor();
        configureReverseLimitSwitch();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 22.373 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.33014 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.016057 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 4.3932 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.074561 : 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.Slot1.kP = RobotHardwareStats.isSimulation() ? 22.373 : 0;
        config.Slot1.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kD = RobotHardwareStats.isSimulation() ? 0.33014 : 0;
        config.Slot1.kS = RobotHardwareStats.isSimulation() ? 0.016057 : 0;
        config.Slot1.kV = RobotHardwareStats.isSimulation() ? 4.3932 : 0;
        config.Slot1.kA = RobotHardwareStats.isSimulation() ? 0.074561 : 0;
        config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 2 : 0;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 10 : 0;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT_POSITION_ROTATIONS;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_LIMIT_SENSOR_RESET_POSITION;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(MOTOR_SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.REVERSE_LIMIT, 100);
        MOTOR.registerSignal(TalonFXSignal.FORWARD_LIMIT, 100);
    }

    private static void configureReverseLimitSwitch() {
        REVERSE_LIMIT_SENSOR.setSimulationSupplier(REVERSE_LIMIT_SWITCH_SIMULATION_SUPPLIER);
        REVERSE_LIMIT_SENSOR_BOOLEAN_EVENT.ifHigh(() -> MOTOR.setPosition(REVERSE_LIMIT_SENSOR_RESET_POSITION));
    }

    public enum ClimberState {
        REST(0, 0, false),
        PREPARE_FOR_CLIMB(3, 1, false),
        CLIMB(0, 0, true);

        public final double targetPositionRotations;
        public final double targetServoPower;
        public final boolean isAffectedByRobotWeight;

        ClimberState(double targetPositionRotations, double targetServoPower, boolean isAffectedByRobotWeight) {
            this.targetPositionRotations = targetPositionRotations;
            this.targetServoPower = targetServoPower;
            this.isAffectedByRobotWeight = isAffectedByRobotWeight;
        }
    }
}