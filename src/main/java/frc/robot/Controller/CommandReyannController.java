package frc.robot.Controller;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandReyannController extends CommandGenericHID {
  private final ReyannController m_hid;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandReyannController(int port) {
    super(port);
    m_hid = new ReyannController(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public ReyannController getHID() {
    return m_hid;
  }

  /**
   * Constructs a Trigger instance around the L1 button's digital signal.
   *
   * @return a Trigger instance representing the L1 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #L1(EventLoop)
   */
  public Trigger L1() {
    return L1(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the L1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the L1 button's digital signal attached to the given
   *     loop.
   */
  public Trigger L1(EventLoop loop) {
    return button(ReyannController.Button.kL1.value, loop);
  }

  /**
   * Constructs a Trigger instance around the L2 button's digital signal.
   *
   * @return a Trigger instance representing the L2 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #L2(EventLoop)
   */
  public Trigger L2() {
    return L2(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the L2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the L2 button's digital signal attached to the given
   *     loop.
   */
  public Trigger L2(EventLoop loop) {
    return button(ReyannController.Button.kL2.value, loop);
  }

  /**
   * Constructs a Trigger instance around the X button's digital signal.
   *
   * @return a Trigger instance representing the X button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #L3(EventLoop)
   */
  public Trigger L3() {
    return L3(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the X button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the X button's digital signal attached to the given
   *     loop.
   */
  public Trigger L3(EventLoop loop) {
    return button(ReyannController.Button.kL3.value, loop);
  }

  /**
   * Constructs a Trigger instance around the L4 button's digital signal.
   *
   * @return a Trigger instance representing the L4 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #L4(EventLoop)
   */
  public Trigger L4() {
    return L4(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the L4 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the L4 button's digital signal attached to the given
   *     loop.
   */
  public Trigger L4(EventLoop loop) {
    return button(ReyannController.Button.kL4.value, loop);
  }
}
