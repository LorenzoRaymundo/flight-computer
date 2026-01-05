import matplotlib
matplotlib.use("TkAgg")  # Setting the backend to Tkinter for matplotlib integration
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import matplotlib.pyplot as plt
import serial
import threading
from time import sleep, time, perf_counter
from serial.tools.list_ports import comports
from collections import deque
import numpy as np
from sys import exit

BAUDRATE = 115200  # Set baudrate for serial communication
N_SAMPLES = 70  # Number of samples to store in the data buffer

# Serial port detection
ports = comports()  # Detect available serial ports
print('Available ports:')
for i, port in enumerate(ports):
    if 'CP210x' in port.description:  # Look for the ESP32 device in the list of available ports
        print(f"{i}. {port.device} - {port.description}\n")
        esp_port = port.device  # Save the ESP32 port

try:
    ser = serial.Serial(esp_port, BAUDRATE, timeout=1)  # Establish serial connection with the ESP32
except Exception as e:
    print(e)  # Print any error that occurs during the connection
    exit()  # Exit the program if connection fails

sleep(2)  # Wait for the serial connection to stabilize

# Measurement groups - Only the measurements we want to plot
group1 = ['ax', 'ay', 'az']  # Accelerometer data
group2 = ['roll', 'pitch']  # Orientation data
group3 = ['vertical_velocity']  # Vertical velocity data
group4 = ['altitude', 'altitude_filtered', 'apogee']  # Altitude and apogee data

# All measurements received from the serial, including non-plotted ones
all_measurements = ['ax', 'ay', 'az', 'roll', 'pitch', 'vertical_velocity', 'altitude', 'altitude_filtered', 'apogee']

# Measurements that will be plotted
plotted_measurements = group1 + group2 + group3 + group4

# Data buffers for storing the measurements
rocket_data = {measurement: deque(maxlen=N_SAMPLES) for measurement in all_measurements}
time_data = deque(maxlen=N_SAMPLES)

t0 = time()  # Record the start time
last_time = 0  # Initialize the last time for data logging
running = True  # Flag to indicate whether the program is running

# Creates a file to log the data for replay after
log_file = None  # Data log file (initialized as None)

# Thread function for reading the serial port
def serial_reader():
    global running, last_time, log_file

    # Open file inside the thread
    reading = ser.readline().decode("utf-8")
    if reading != '':
        with open("telemetry_log.csv", "w") as log_file:
            log_file.write("time," + ",".join(all_measurements) + "\n")  # Write the header to the log file
            log_file.flush()  # Force write to file

            while running:  # Keep reading data while the program is running
                if ser.in_waiting > 0:  # Check if there is data to read
                    line = ser.read_until(b'\n')  # Read a line from the serial port
                    print(line)  # Print the line for debugging

                    try:
                        parts = line.decode("utf-8", errors="ignore").strip().split("\t")  # Decode and split the data
                        if len(parts) < len(all_measurements):  # If the data is incomplete, skip it
                            continue
                        vals = list(map(float, parts))  # Convert the data to floats
                    except:
                        continue  # Skip any line that causes an error

                    current_time = time() - t0  # Calculate the current time based on the start time
                    time_data.append(current_time)  # Append the current time to the time buffer

                    if current_time - last_time > 0.1:  # Log data every 0.1 seconds
                        last_time = current_time
                        log_file.write(f"{current_time:.3f}," + ",".join(parts) + "\n")  # Write data to the log file
                        log_file.flush()  # Force write after each line

                    # Store the received values in the corresponding measurement buffers
                    for i, measurement in enumerate(all_measurements):
                        rocket_data[measurement].append(vals[i])

# Create and start a new thread for serial reading
thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

print(f'Listening to ESP32 at {esp_port}...')  # Indicate the port that the ESP32 is connected to

# Plot setup

# ===========================
# SpaceX / Terminal Neon Theme
# ===========================

# Set background color to almost-black, text color to light-gray
matplotlib.rcParams['figure.facecolor'] = '#000000'
matplotlib.rcParams['axes.facecolor'] = '#050505'
matplotlib.rcParams['savefig.facecolor'] = '#000000'

matplotlib.rcParams['text.color'] = '#E0E0E0'
matplotlib.rcParams['axes.labelcolor'] = '#E0E0E0'
matplotlib.rcParams['xtick.color'] = '#AAAAAA'
matplotlib.rcParams['ytick.color'] = '#AAAAAA'
matplotlib.rcParams['axes.edgecolor'] = '#3A3A3A'

# Neon color palette for plotting
NEON_COLORS = [
    '#00FF9F',  # neon green
    '#00E5FF',  # neon cyan
    '#FF00FF',  # neon magenta
    '#FFFF00',  # neon yellow
    '#FF0055'   # bright red futuristic
]

# Create subplots (2x2 grid) for the measurements
fig, axes = plt.subplots(2, 2, figsize=(12, 1), sharex=True)

ax_acc = axes[0][0]  # Accelerometer plot
ax_rp  = axes[0][1]  # Roll and Pitch plot
ax_alt = axes[1][0]  # Altitude plot
ax_vel = axes[1][1]  # Vertical velocity plot

plot_lines = {}  # Dictionary to hold the plot lines for each measurement
color_idx = 0  # Color index for plotting

# Plot accelerometer data (group1)
for measurement in group1:
    line, = ax_acc.plot([], [], label=measurement, color=NEON_COLORS[color_idx % len(NEON_COLORS)])  # Plot each measurement with a neon color
    plot_lines[measurement] = line
    color_idx += 1

ax_acc.set_title("ACCELEROMETER", fontsize=10)
ax_acc.set_ylabel("g")
ax_acc.grid(True, color='#00FF9F', alpha=0.15, linestyle='--')  # Grid with neon color
ax_acc.legend(loc='upper left', fontsize=8, framealpha=0, labelcolor='#FFFFFF')  # Legend for accelerometer


# Plot roll and pitch data (group2)
for measurement in group2:
    if measurement == 'roll':
        line, = ax_rp.plot([], [], label=measurement, color=NEON_COLORS[1])  # Cyan for roll
    elif measurement == 'pitch':
        line, = ax_rp.plot([], [], label=measurement, color=NEON_COLORS[4])  # Red for pitch
    plot_lines[measurement] = line

ax_rp.set_title("ORIENTATION", fontsize=10)
ax_rp.set_ylabel("Degrees")
ax_rp.grid(True, color='#00FF9F', alpha=0.15, linestyle='--')
ax_rp.legend(loc='upper left', fontsize=8, framealpha=0, labelcolor='#FFFFFF')  # Legend for orientation


# Plot altitude data (group4)
for measurement in group4:
    line, = ax_alt.plot([], [], label=measurement, color=NEON_COLORS[color_idx % len(NEON_COLORS)])
    plot_lines[measurement] = line
    color_idx += 1

ax_alt.set_title("ALTITUDE", fontsize=10)
ax_alt.set_ylabel("Meters")
ax_alt.grid(True, color='#00FF9F', alpha=0.15, linestyle='--')
ax_alt.legend(loc='upper left', fontsize=8, framealpha=0, labelcolor='#FFFFFF')  # Legend for altitude


# Plot vertical velocity data (group3)
for measurement in group3:
    line, = ax_vel.plot([], [], label=measurement, color=NEON_COLORS[color_idx % len(NEON_COLORS)])
    plot_lines[measurement] = line
    color_idx += 1

ax_vel.set_title("VERTICAL VELOCITY", fontsize=10)
ax_vel.set_ylabel("m/s")
ax_vel.set_xlabel("Time [s]")
ax_vel.grid(True, color='#00FF9F', alpha=0.15, linestyle='--')
ax_vel.legend(loc='upper left', fontsize=8, framealpha=0, labelcolor='#FFFFFF')  # Legend for vertical velocity


# Main title for the figure
fig.suptitle("ROCKET TELEMETRY", fontsize=20, fontweight='bold')

# Create the Tkinter window for the interface
window = tk.Tk()
window.title("Rocket Telemetry Interface")
window.geometry("1300x900")
window.configure(bg="#111111")

# Button to start replay from file
btn_replay = tk.Button(
    window,
    text="Replay last log",
    bg="#222222",
    fg="#00FF9F",
    command=lambda: start_replay_from_file("telemetry_log_escola.csv")
)
btn_replay.pack(pady=10)

# Embed the matplotlib figure in the Tkinter window
canvas = FigureCanvasTkAgg(fig, master=window)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(fill="both", expand=True)

replay_mode = False  # Flag to indicate if the system is in replay mode

# Function to start replay from a log file
def start_replay_from_file(filename):
    global replay_mode
    replay_mode = True

    # Clear existing data in the plots
    time_data.clear()
    for m in all_measurements:
        rocket_data[m].clear()

    # Create a generator for replaying data
    gen = replay_generator(filename)

    # Start the replay loop
    window.after(5, lambda: replay_step(gen))

# Generator for replaying data from a file
def replay_generator(filename):
    with open(filename, "r") as f:
        next(f)  # Skip header line
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < len(all_measurements) + 1:
                continue

            t = float(parts[0])
            vals = list(map(float, parts[1:]))
            yield t, vals

# Replay step function to update the plot with replay data
def replay_step(gen):
    if not replay_mode:
        return

    try:
        t, vals = next(gen)
    except StopIteration:
        return  # End of replay

    time_data.append(t)
    for i, m in enumerate(all_measurements):
        rocket_data[m].append(vals[i])

    # Only update plotted measurements
    for m in plotted_measurements:
        plot_lines[m].set_xdata(time_data)
        plot_lines[m].set_ydata(rocket_data[m])

    for row in axes:
        for ax in row:
            ax.relim()
            ax.autoscale_view()

    canvas.draw_idle()

    window.after(5, lambda: replay_step(gen))

# Performance optimization variables
frame_times = deque(maxlen=30)  # Keep the last 30 frame times for averaging
last_limits_update = time()
update_interval = 0.1  # Seconds between full updates

max_vv_display = 0
parachute_deployed = False  # Add this flag to prevent multiple lines for parachute deployment

# Function to update the plot with live data
def update_plot():
    global max_vv_display, parachute_deployed, deployment_time

    if time_data:
        x = np.array(time_data)

        # Updates vertical velocity and apogee display
        try:
            apogee_display = float(rocket_data['apogee'][-1]) / 100
            vv = float(rocket_data['vertical_velocity'][-1]) / 100
            ax_alt.set_title(f"ALTITUDE | APOGEE: {apogee_display:.2f} m", fontsize=10)

            if vv > max_vv_display:
                max_vv_display = vv
                ax_vel.set_title(f"VERTICAL VELOCITY | MAX: {max_vv_display:.2f} m/s", fontsize=10)
        except:
            apogee_display = 0

        # Check parachute deployment condition
        if (not parachute_deployed and
            len(rocket_data['apogee']) > 0 and
            len(rocket_data['altitude']) > 0 and
            rocket_data['apogee'][-1] - rocket_data['altitude'][-1] >= 200 and  # Changed from == to >=
            rocket_data['apogee'][-1] > 100):
           
            parachute_deployed = True
            deployment_time = time_data[-1]  # Get the current time

        # Update data - only update plotted measurements
        for measurement in plotted_measurements:
            y = np.array(rocket_data[measurement])
            plot_lines[measurement].set_xdata(x)
            plot_lines[measurement].set_ydata(y)

        # Rescale the plots
        for row in axes:
            for ax in row:
                ax.relim()
                ax.autoscale_view()

    # Update the canvas with the new data
    canvas.draw_idle()
    window.after(33, update_plot)

# Function to handle window closing
def on_closing():
    global running
    running = False
    ser.close()  # Close the serial connection
    if log_file:
        log_file.close()  # Close the log file if it's open
    window.destroy()  # Close the Tkinter window
    program_end_time = time()

    if parachute_deployed:
        print(f'deployment time at t={deployment_time:.2f}s')
    execution_time = program_end_time - t0
    print("Program ended!")
    print(f'Execution time: {execution_time:.2f}s')
    exit()  # Exit the program

# Set up the close event handler for the window
window.protocol("WM_DELETE_WINDOW", on_closing)

# Begin the update loop
update_plot()
window.mainloop()
