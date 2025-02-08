package frc.lib;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Joystick wrapper because ~~it was pissing me off~~ we need a cleaner RobotContainer.
 * <p>
 * The methods in this class return triggers. To get boolean values, use {@link Trigger#getAsBoolean()}
 * </p>
 */
public class FlightControl {
    private final Joystick joystick;
    private final Trigger button1;
    private final Trigger button2;
    private final Trigger button3;
    private final Trigger button4;

    public  FlightControl(int port) {
        joystick = new Joystick(port);

        button1 = new JoystickButton(joystick, 1);
        button2 = new JoystickButton(joystick, 2);
        button3 = new JoystickButton(joystick, 3);
        button4 = new JoystickButton(joystick, 4);
    }
    

    public Joystick getJoystick() {
        return joystick;
    }
    
    public double getJoyX() {
        return joystick.getRawAxis(Joystick.AxisType.kX.value)*(0.75-joystick.getRawAxis(4)/4);
    }

    public double getJoyY() {
        return joystick.getRawAxis(Joystick.AxisType.kY.value)*(0.75-joystick.getRawAxis(4)/4);
    }

    public double getTwist() {
        return joystick.getRawAxis(2);
    }

    public Trigger getButton1() {
        return button1;
    }
    public Trigger getButton2() {
        return button2;
    }
    public Trigger getButton3() {
        return button3;
    }
    public Trigger getButton4() {
        return button4;
    }
    public Trigger getButton(int button) {
        Trigger buttont;
        buttont = new JoystickButton(joystick, button);
        return buttont;
    }



   
   
}