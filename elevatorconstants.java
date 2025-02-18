package frc.robot.subsystems.elevator;

public class elevatorconstants {
    package frc.robot.subsystems.elevator;

/**
 * Constants file for storing all robot-wide constant values
 * Organizing constants in a central location helps with maintenance and configuration
 */
public final class Constants {
    
    /**
     * Constants related to the Elevator subsystem
     */
    public static final class ElevatorConstants {
        // CAN IDs for motor controllers
        public static final int LEFT_MOTOR_ID = 11; //TODO update
        public static final int RIGHT_MOTOR_ID = 12; //TODO update
        
        // Digital IO port numbers for sensors
        public static final int UPPER_LIMIT_SWITCH_PORT = 0; //TODO
        public static final int LOWER_LIMIT_SWITCH_PORT = 1;
        
        // Motor configuration
        public static final boolean LEFT_MOTOR_INVERTED = false; //TODO
        public static final boolean RIGHT_MOTOR_INVERTED = true;
        
        // Encoder configuration
        public static final double GEAR_RATIO = 10.0;        // 10:1 reduction from MAX planetary
        public static final double SPOOL_DIAMETER = 2.0;     // 2 inch diameter spool
        public static final double INCHES_PER_ROTATION = Math.PI * SPOOL_DIAMETER;
        public static final double POSITION_CONVERSION_FACTOR = INCHES_PER_ROTATION / GEAR_RATIO;
        
        // PID controller constants TODO
        public static final double ELEVATOR_KP = 0.05;  // Proportional gain
        public static final double ELEVATOR_KI = 0.0;   // Integral gain (typically start at 0)
        public static final double ELEVATOR_KD = 0.0;   // Derivative gain (typically start at 0)
        public static final double POSITION_TOLERANCE = 0.5;  // Position tolerance in inches
        
        // Feedforward constants TODO
        public static final double ELEVATOR_KF = 0.15;  // Feedforward to overcome gravity/friction
        
        // Safety limits TODO
        public static final double MAX_OUTPUT = 0.8;  // Maximum allowed motor output (0.0 to 1.0)
        public static final double MAX_HEIGHT = 60.0; // Maximum height in inches
        public static final double MIN_HEIGHT = 0.0;  // Minimum height in inches
        
        // Preset positions (in inches) TODO
        public static final double HOME_POSITION = 0.0;
        public static final double PICKUP_POSITION = 4.0;
        public static final double MID_POSITION = 30.0;
        public static final double HIGH_POSITION = 55.0;
    }
  
}
}
