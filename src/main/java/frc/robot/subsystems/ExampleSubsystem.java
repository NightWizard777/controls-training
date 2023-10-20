// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
  private final CANSparkMax topMotor;
  private final CANSparkMax bottomMotor;
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

  // Motor variables
  private double topSpeed = 0;
  private double bottomSpeed = 0;
  // Button status variables
  private boolean aPress;
  private boolean bPress;
  private boolean xPress;
  private boolean yPress;
  // Distance variables
  private double topDist = 0;
  private double bottomDist = 0;
  // Color sensor variables
  private Color colorSeen = Color.kBlack;
  // Timer variables
  private boolean started = false;
  // Status variables
  private int balls = 0;

  public ExampleSubsystem() {
    // Define motors
    topMotor = new CANSparkMax(13, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
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
    colorSeen = color.getColor();
  }

  public void updateBalls() {
    // Update number of balls in intake
    // TODO
  }

  @Override
  public void periodic() {
    // Update variables
    updateButtons();
    updateSensors();
    updateBalls();

    // TODO
    topMotor.set(topSpeed);
    bottomMotor.set(bottomSpeed);
  }
}
