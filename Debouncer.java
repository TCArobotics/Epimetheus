package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

//wait for debouce_period amount of seconds before checking for a button press. This prevents the controller from sending thousands of button presses per second.
public class Debouncer
{
    XboxController myController;
    int buttonnum;
    double latest;
    double debounce_period;

    public Debouncer(XboxController _myController, int _buttonnum)
    {
        this.myController = _myController;
        this.buttonnum = _buttonnum;
        this.latest = 0;
        this.debounce_period = .5;
    }
    public Debouncer(XboxController _myController, int _buttonnum, float _period)
    {
        this.myController = _myController;
        this.buttonnum = _buttonnum;
        this.latest = 0;
        this.debounce_period = _period;
    }
    public void setDebouncePeriod(float _period)
    {
        this.debounce_period = _period;
    }
    public boolean get()
    {
        double now = Timer.getFPGATimestamp();
        if(myController.getRawButton(buttonnum))
        {
            if ((now-latest) > debounce_period)
            {
                latest = now;
                return true;
            }
        }
        return false;
    }
}
