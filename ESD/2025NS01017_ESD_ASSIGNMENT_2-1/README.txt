## ARM7TDMI NXP LPC2378

1.  **A/D Converter 0:** For manipulating the sensor input (LM35 voltage).
2.  **Timer 0:** Although labeled "Timer," this is the window where we will look at **Match Register 1 (T0MR1)** to confirm the fan's PWM duty cycle output.
3.  **External Interrupts:** For simulating the emergency stop button press on EINT0.
4.  **Pulse Width Modulator 1 (PWM 1):** This window is not strictly needed since our code uses **Timer 0 (T0)** for PWM, but it's good to know where the PWM registers usually live.

The only window you still need to open manually is the **Serial Window** for logging the data:

* Go to **Peripherals $\to$ Serial Window $\to$ UART 0**

---

### Validation Step-by-Step

Now, let's proceed with the two critical validation tests using the windows you have open.

#### 1. Test A: High Temperature (Proportional Control)

This proves the **ADC $\to$ Control Logic $\to$ Timer/PWM** path works.

| Action | Window to Manipulate | Register/Value to Observe | Expected Result |
| :--- | :--- | :--- | :--- |
| **Simulate Input** | **A/D Converter 0** | Locate **AD0.0** (Channel 0). Manually set the **Analog Input Voltage** to **$\mathbf{400\text{ mV}}$** ($\approx 40^\circ C$). | **A/D Converter 0:** **AD0DR0** should show a value around **600** (digital value). |
| **Check Actuation** | **Timer 0** | Observe **Match Register 1 (T0MR1)**. | The value should stabilize around **$\mathbf{500}$** to **$\mathbf{600}$** (representing $50\% - 60\%$ duty cycle). |
| **Check Communication**| **UART 0** | View the serial output. | Message showing: `Temp: 40.0C, Fan Duty: 050%` (or similar). |

#### 2. Test B: Emergency Shutdown (Interrupt Priority)

This proves the **External Interrupt (EINT0)** and **VIC** functionality.

| Action | Window to Manipulate | Register/Value to Observe | Expected Result |
| :--- | :--- | :--- | :--- |
| **Set State** | (Use the $40^\circ C$ high temp state, with **T0MR1** high). | | |
| **Trigger EINT0** | **External Interrupts** | Locate **EINT0**. Manually **toggle the input state** to trigger the **falling edge** (High $\to$ Low). | **Timer 0 (T0MR1):** Value immediately drops to **$\mathbf{0}$**. |
| **Check Communication**| **UART 0** | View the serial output. | Message should immediately appear: `*** EMERGENCY SHUTDOWN TRIGGERED! (LPC2378) ***` |

***
Gathering screenshots from the successful **UART 0** log and the **Timer 0** register window during these two tests will complete the validation section of your report.
