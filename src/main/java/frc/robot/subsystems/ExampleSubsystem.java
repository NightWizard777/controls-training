// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ExampleSubsystem extends SubsystemBase {

  private final CANSparkMax main;
  private final CANSparkMax follower;
  private final XboxController controller;
  private final JoystickButton buttonA;
  private final JoystickButton buttonB;
  private final JoystickButton buttonX;
  private final JoystickButton buttonY;
  private boolean aPress;
  private boolean bPress;
  private boolean xPress;
  private boolean yPress;
  private final Timer timer;
  private boolean started = false;
  private double speed = 0;
  private double multi = 1;

  public ExampleSubsystem() {
    main = new CANSparkMax(1, MotorType.kBrushless);
    follower = new CANSparkMax(2, MotorType.kBrushless);
    follower.follow(main);
    controller = new XboxController(0);
    buttonA = new JoystickButton(controller, XboxController.Button.kA.value);
    buttonB = new JoystickButton(controller, XboxController.Button.kB.value);
    buttonX = new JoystickButton(controller, XboxController.Button.kX.value);
    buttonY = new JoystickButton(controller, XboxController.Button.kY.value);
    timer = new Timer();
  }

  public CommandBase exampleMethodCommand() {return runOnce(() -> {});}
  public boolean exampleCondition() {return false;}

  @Override
  public void periodic() {
    aPress = buttonA.getAsBoolean();
    bPress = buttonB.getAsBoolean();
    xPress = buttonX.getAsBoolean();
    yPress = buttonY.getAsBoolean();

    if (aPress && !bPress) {
      speed = 0.25;
    } else if (!aPress && bPress) {
      speed = -0.25;
    } else {
      speed = 0;
    }

    if (!started && (xPress || yPress)) {
      started = true;
      timer.start();
      if (xPress) multi = 2;
      else if (yPress) multi = 0.5;
    } else if (started && timer.hasElapsed(2)) {
      timer.stop();
      timer.reset();
      started = false;
      multi = 1;
    }

    main.set(speed*multi);
  }
}
