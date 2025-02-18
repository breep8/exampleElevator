package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.elevatorconstants.Constants;

/**
 * Elevator Subsystem - 
 * Controls the elevator mechanism using NEO Vortex motors with MAX Planetary gearboxes
 * 
 * Complete the implementation of this elevator subsystem by filling in
 * the sections marked with [TODO]. Refer to the comments for guidance.
 */

public class Elevator extends SubsystemBase {
    
// Define your motor controllers here
// [TODO] - Create the CANSparkMax object for the right motor
    private CANSparkMax leftMotor;

// Define your encoders here
// [TODO] - Create the right RelativeEncoder object
    private RelativeEncoder leftEncoder;
    
// Define your limit switches here
// [TODO] - Create DigitalInput objects for your lower limit switch
    private DigitalInput upperLimitSwitch;
    
// Define your PID controller here
    private PIDController elevatorController;

    /**
     * Constructor - initializes the elevator subsystem
     */

    public Elevator() {
        // [TODO] Initialize your other motor controller
        this.rightMotor = new CANSparkMax(Constants.ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        
        // [TODO] Configure the other motor (set inverted status if needed)
        // HINT: Think about which motor might need to spin in the opposite direction
        this.leftMotor.setInverted(Constants.ElevatorConstants.LEFT_MOTOR_INVERTED);

        // [TODO] Get and configure the other encoder
        // HINT: Use getEncoder() method on CANSparkMax, then set conversion factors
        this.leftEncoder = this.leftMotor.getEncoder();

        // [TODO] Initialize the lower limit switch
        // HINT: Use the DigitalInput constructor with the appropriate DIO port numbers
        this.upperLimitSwitch = new DigitalInput(Constants.ElevatorConstants.UPPER_LIMIT_SWITCH_PORT);

        // [TODO] Create and configure the PID controller
        this.elevatorController = new PIDController(
            Constants.ElevatorConstants.ELEVATOR_KP,  // Proportional gain
            Constants.ElevatorConstants.ELEVATOR_KI,  // Integral gain
            Constants.ElevatorConstants.ELEVATOR_KD   // Derivative gain
        );


    }
    
    @Override
    public void periodic() {
        // [TODO] Update SmartDashboard with useful information
        // HINT: Consider displaying position, limit switch status, and motor current
        SmartDashboard.putNumber("Elevator Position", this.getPosition());
    }
    
    /**
     * Checks if the elevator has reached the upper limit
     */
    public boolean isAtUpperLimit() {
        // [TODO] Return whether the upper limit is reached
        // HINT: Remember that limit switches typically return false when pressed
        return false; // Replace with your implementation
    }
    
    /**
     * Checks if the elevator has reached the lower limit
     */
    public boolean isAtLowerLimit() {
        // [TODO] Return whether the lower limit is reached
        return false; // Replace with your implementation
    }
    
    /**
     * Sets the speed of both elevator motors
     * @param speed The speed to set (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        // [TODO] Set the speed of both motors, with safety checks
        // HINT: Consider what happens when trying to move past the limits
    }
    
    /**
     * Gets the current position of the elevator
     */
    public double getPosition() {
        // [TODO] Return the elevator position
        // HINT: Consider which encoder you want to use for position feedback
        return 0.0; // Replace with your implementation
    }
    
    /**
     * Uses PID control to move the elevator to a specific position
     * @param targetPosition The desired position
     */
    public void setPosition(double targetPosition) {
        // [TODO] Implement PID position control
        // HINT: Calculate PID output, apply safety checks, and set motor outputs
    }
    
    /**
     * Checks if the elevator is at the desired position
     */
    public boolean atPosition() {
        // [TODO] Determine if the elevator is at the target position
        // HINT: Use the PID controller's atSetpoint method and check limit switches
        return false; // Replace with your implementation
    }
    
    /**
     * Sets the motor idle behavior
     * @param brake If true, use brake mode; if false, use coast mode
     */
    public void setBrakeMode(boolean brake) {
        // [TODO] Set the appropriate idle mode on both motors
        // HINT: Use setIdleMode with IdleMode.kBrake or IdleMode.kCoast
    }
    
    /**
     * Resets the elevator encoders to zero
     */
    public void resetEncoders() {
        // [TODO] Reset both encoders to zero position
        // HINT: Use the setPosition method on the encoders
    }
}
