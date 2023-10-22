// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ExampleSubsystem extends SubsystemBase {

  // Motors
  private final WPI_TalonSRX topMotor;
  private final WPI_TalonSRX bottomMotor;
  // Controller + buttons
  private final XboxController controller;
  private final JoystickButton buttonA;
  private final JoystickButton buttonB;
  private final JoystickButton buttonX;
  private final JoystickButton buttonY;
  // Sensors
  private final ColorSensorV3 color;
  private final AnalogPotentiometer topIR;
  private final AnalogPotentiometer bottomIR;
  // Timer
  private final Timer timer;

  // Button status variables
  private boolean aPress;
  private boolean bPress;
  private boolean xPress;
  private boolean yPress;
  // Distance variables
  private double topDist = 0;
  private double bottomDist = 0;
  private double colorDist = 0;
  // Status variables
  private boolean timerStarted = false;
  private boolean topRunning = false;
  private boolean bottomRunning = false;
  private int balls = 0;

  public ExampleSubsystem() {
    // Define motors
    topMotor = new WPI_TalonSRX(14);
    bottomMotor = new WPI_TalonSRX(13);
    // Define controller + buttons
    controller = new XboxController(0);
    buttonA = new JoystickButton(controller, XboxController.Button.kA.value);
    buttonB = new JoystickButton(controller, XboxController.Button.kB.value);
    buttonX = new JoystickButton(controller, XboxController.Button.kX.value);
    buttonY = new JoystickButton(controller, XboxController.Button.kY.value);
    // Define sensors
    color = new ColorSensorV3(I2C.Port.kMXP);
    topIR = new AnalogPotentiometer(1);
    bottomIR = new AnalogPotentiometer(0);
    // Define timer
    timer = new Timer();
  }

  // Ignore, everything breaks without this :)
  public CommandBase exampleMethodCommand() {return runOnce(() -> {});}
  public boolean exampleCondition() {return false;}

  public void updateButtons() {
    // Update button status variables
    aPress = buttonA.getAsBoolean();
    bPress = buttonB.getAsBoolean();
    xPress = buttonX.getAsBoolean();
    yPress = buttonY.getAsBoolean();
  }
  public void updateSensors() {
    // Update sensor variables
    topDist = topIR.get();
    bottomDist = bottomIR.get();
    colorDist = color.getProximity();
  }

  public boolean bottomSeesBall() {return bottomDist > 0.10;} // Check if bottom IR sees ball
  public boolean topSeesBall() {return topDist > 0.25;} // Check if top IR sees ball
  public boolean colorSeesBall() {return colorDist > 1500;} // Check if color sensor sees ball

  public void startTop() {
    // Start top motor
    topMotor.set(-0.3);
    topRunning = true;
  }
  public void startBottom() {
    // Start bottom motor
    bottomMotor.set(-0.1);
    bottomRunning = true;
  }
  public void stopTop() {
    // Stop top motor
    topMotor.set(0);
    topRunning = false;
  }
  public void stopBottom() {
    // Stop bottom motor
    bottomMotor.set(0);
    bottomRunning = false;
  }

  public void shoot() {
    // Shoot ball if available
    if (balls == 0) { // 0 balls
      System.out.println("No balls :("); // Can't shoot
    } else { // 1+ balls
      System.out.println("PEW"); // Shoot ball (?)
      balls--;
    }
  }

  @Override
  public void periodic() {
    // Update variables
    updateButtons();
    updateSensors();

    // Shooting
    if (aPress && !bottomRunning && !topRunning) { // A button pressed and not running motors
      shoot(); // Shoot ball
    }

    // Intake
    if (bottomRunning && topRunning) { // Running both motors (1 ball case)
      if (!topSeesBall()) { // Top IR doesn't see ball (risen above)
        stopTop(); // Stop top motor
      }
    } else if (bottomRunning) { // Running bottom motor only (2 balls case)
      if (!bottomSeesBall() && !colorSeesBall()) { // Bottom IR and color don't see ball (risen to stage 2)
        stopBottom(); // Stop bottom motor
      }
    } else if (bottomSeesBall()) { // Bottom IR sees new ball (not running motors)
      balls++; // Increment ball counter
      if (balls == 1) { // 1 ball
        startTop(); // Run both motors
        startBottom();
      } else if (balls == 2) { // 2 balls
        startBottom(); // Run bottom motor only
      } else { // 3+ balls
        System.out.println("Enough balls.");
      }
    }
  }
}