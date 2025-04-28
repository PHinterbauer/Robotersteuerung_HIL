import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# --- CartesianRobot3D Class (Modified for External Control) ---
# [NO CHANGES IN THIS CLASS - Keep as is]
class CartesianRobot3D:
    """
    Simulates a 3-axis Cartesian robot with a gripper in 3D space.
    State is updated externally.

    Attributes:
        ax (matplotlib.axes._axes.Axes3D): The 3D axes for plotting.
        x, y, z (float): Current robot coordinates.
        gripper_open (bool): Current gripper state.
        # ... plot objects (lines, markers) ...
    """
    def __init__(self, ax, x_lim=(-10, 110), y_lim=(-10, 110), z_lim=(-10, 110)):
        """
        Initializes the robot visualization elements on the provided axes.

        Args:
            ax (matplotlib.axes._axes.Axes3D): The matplotlib axes to draw on.
            x_lim, y_lim, z_lim (tuple): Axis limits.
        """
        self.ax = ax
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.gripper_open = True # Default state

        # Store limits for potential use in drawing rails relative to bounds
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.z_lim = z_lim

        # --- Constants for visualization ---
        self.RAIL_COLOR = 'gray'
        self.CARRIAGE_COLOR = 'black'
        self.TCP_COLOR = 'red'
        self.GRIPPER_COLOR = 'blue'
        self.GRIPPER_WIDTH_OPEN = 8.0
        self.GRIPPER_WIDTH_CLOSED = 1.0
        self.GRIPPER_LENGTH = 10.0
        self.CARRIAGE_SIZE = 8
        self.TCP_SIZE = 10

        # --- Clear and Setup Axes ---
        self.ax.clear() # Ensure axes are clean before drawing
        self.ax.set_xlim(x_lim)
        self.ax.set_ylim(y_lim)
        self.ax.set_zlim(z_lim)
        self.ax.set_xlabel('X-Axis')
        self.ax.set_ylabel('Y-Axis')
        self.ax.set_zlabel('Z-Axis')
        self.ax.set_title('Cartesian Robot Simulation')

        # --- Initialize Plot Objects ---
        # Rails drawn relative to axis limits
        self.x_axis_line, = self.ax.plot([x_lim[0], x_lim[1]], [y_lim[0], y_lim[0]], [z_lim[0], z_lim[0]],
                                         color=self.RAIL_COLOR, linewidth=2, label='X-Rail')
        self.y_axis_line, = self.ax.plot([self.x, self.x], [y_lim[0], y_lim[1]], [z_lim[0], z_lim[0]],
                                         color=self.RAIL_COLOR, linewidth=2, label='Y-Rail')
        self.z_axis_line, = self.ax.plot([self.x, self.x], [self.y, self.y], [z_lim[0], z_lim[1]],
                                         color=self.RAIL_COLOR, linewidth=2, label='Z-Rail')
        # Carriages
        self.x_carriage_marker, = self.ax.plot([self.x], [y_lim[0]], [z_lim[0]], marker='s',
                                                markersize=self.CARRIAGE_SIZE, color=self.CARRIAGE_COLOR, label='X-Carriage')
        self.y_carriage_marker, = self.ax.plot([self.x], [self.y], [z_lim[0]], marker='s',
                                                markersize=self.CARRIAGE_SIZE, color=self.CARRIAGE_COLOR, label='Y-Carriage')
        # TCP
        self.tcp_marker, = self.ax.plot([self.x], [self.y], [self.z], marker='o',
                                         markersize=self.TCP_SIZE, color=self.TCP_COLOR, label='TCP')
        # Gripper
        gx1, gy1, gz1 = self._get_gripper_coords(finger=1)
        gx2, gy2, gz2 = self._get_gripper_coords(finger=2)
        self.gripper_finger1, = self.ax.plot(gx1, gy1, gz1, linewidth=4, color=self.GRIPPER_COLOR, label='Gripper')
        self.gripper_finger2, = self.ax.plot(gx2, gy2, gz2, linewidth=4, color=self.GRIPPER_COLOR)

        self.ax.legend(fontsize='small') # Add legend

    def _get_gripper_coords(self, finger):
        """
        Helper function to calculate gripper finger coordinates.
        Fingers are separated along Z and extend along +Y.
        """
        separation = self.GRIPPER_WIDTH_OPEN if self.gripper_open else self.GRIPPER_WIDTH_CLOSED
        half_separation = separation / 2.0
        z_offset = half_separation if finger == 1 else -half_separation
        start_point = [self.x, self.y, self.z + z_offset]
        end_point = [self.x, self.y + self.GRIPPER_LENGTH, self.z + z_offset]
        return [start_point[0], end_point[0]], [start_point[1], end_point[1]], [start_point[2], end_point[2]]

    def set_state(self, x, y, z, gripper_open):
        """Directly sets the robot's state variables."""
        self.x = x
        self.y = y
        self.z = z
        # Ensure gripper_open is boolean
        self.gripper_open = bool(gripper_open)

    def update_plot(self):
        """Updates the plot objects with the current robot state."""
        # Update Y-axis rail position (depends on X)
        self.y_axis_line.set_data([self.x, self.x], [self.y_lim[0], self.y_lim[1]])
        self.y_axis_line.set_3d_properties([self.z_lim[0], self.z_lim[0]])

        # Update Z-axis rail position (depends on X and Y)
        self.z_axis_line.set_data([self.x, self.x], [self.y, self.y])
        self.z_axis_line.set_3d_properties([self.z_lim[0], self.z_lim[1]])

        # Update carriage markers
        self.x_carriage_marker.set_data([self.x], [self.y_lim[0]])
        self.x_carriage_marker.set_3d_properties([self.z_lim[0]])

        self.y_carriage_marker.set_data([self.x], [self.y])
        self.y_carriage_marker.set_3d_properties([self.z_lim[0]])

        # Update TCP marker position
        self.tcp_marker.set_data([self.x], [self.y])
        self.tcp_marker.set_3d_properties([self.z])

        # Update Gripper fingers
        gx1, gy1, gz1 = self._get_gripper_coords(finger=1)
        gx2, gy2, gz2 = self._get_gripper_coords(finger=2)
        self.gripper_finger1.set_data(gx1, gy1)
        self.gripper_finger1.set_3d_properties(gz1)
        self.gripper_finger2.set_data(gx2, gy2)
        self.gripper_finger2.set_3d_properties(gz2)

        # Note: The actual drawing/refreshing is handled by the canvas in the GUI


# --- Main Application Class ---
class RobotSimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Simulator Control")
        self.geometry("800x700") # Adjust size as needed

        # --- State Variables ---
        self.serial_port = None
        self.serial_thread = None
        self.stop_thread_flag = threading.Event()
        self.data_queue = queue.Queue()
        self.is_running = False
        self.port_map = {} # <<< ADDED: Dictionary to map display names to device names

        # --- GUI Setup ---
        # Control Frame
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(side=tk.TOP, fill=tk.X)

        # Serial Port Selection
        ttk.Label(control_frame, text="Serial Port:").pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar()
        # Increase width to accommodate longer descriptions
        self.port_combobox = ttk.Combobox(control_frame, textvariable=self.port_var, state="readonly", width=45) # <<< Increased width
        self.port_combobox.pack(side=tk.LEFT, padx=5)
        self.refresh_button = ttk.Button(control_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_button.pack(side=tk.LEFT, padx=5)

        # Control Buttons
        self.start_button = ttk.Button(control_frame, text="Start", command=self.start_simulation)
        self.start_button.pack(side=tk.LEFT, padx=5)
        self.stop_button = ttk.Button(control_frame, text="Stop", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)

        # Status Label
        self.status_var = tk.StringVar(value="Status: Idle")
        status_label = ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_label.pack(side=tk.BOTTOM, fill=tk.X)

        # Plot Frame
        plot_frame = ttk.Frame(self)
        plot_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- Matplotlib Setup ---
        self.fig = plt.figure(figsize=(7, 6)) # Adjust figure size for embedding
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

        # --- Robot Instance ---
        # Initialize robot visualization on the axes
        self.robot = CartesianRobot3D(self.ax)
        self.canvas.draw() # Initial draw

        # --- Initial Setup ---
        self.refresh_ports() # Populate ports on startup
        self.check_queue() # Start the loop checking for data from serial thread

        # --- Handle Window Closing ---
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def refresh_ports(self):
        """
        Scans for available serial ports, creates descriptive names,
        updates the combobox, and maintains a mapping.
        """
        self.port_map.clear() # Clear the old mapping
        ports_info = serial.tools.list_ports.comports()
        display_ports = []
        actual_devices = [] # Keep track of actual device names found

        for port in ports_info:
            # Create a descriptive string: "Description (Device)" or just "Device" if description is missing
            description = port.description if port.description and port.description != "n/a" else "Unknown Port"
            display_name = f"{description} ({port.device})"
            display_ports.append(display_name)
            actual_devices.append(port.device)
            self.port_map[display_name] = port.device # Map display name to actual device

        self.port_combobox['values'] = display_ports
        print(f"Available ports: {self.port_map}") # Log the mapping for debugging

        current_selection_display = self.port_var.get()
        current_selection_device = self.port_map.get(current_selection_display) # Get device for current display string

        if display_ports:
            # Check if the currently selected port (based on its device name) is still available
            if current_selection_device and current_selection_device in actual_devices:
                # The previously selected port is still valid, keep it selected
                # (StringVar already holds the display name, so no change needed here)
                pass
            else:
                # Previously selected port is gone or nothing was selected, default to the first available port
                self.port_var.set(display_ports[0])
        else:
            # No ports found
            self.port_var.set("") # Clear selection
            messagebox.showwarning("No Ports Found", "No serial ports detected. Connect hardware and refresh.")


    def start_simulation(self):
        """Starts the serial connection and data reading thread."""
        if self.is_running:
            messagebox.showwarning("Already Running", "Simulation is already running.")
            return

        selected_display_name = self.port_var.get()
        if not selected_display_name:
            messagebox.showerror("Port Error", "Please select a serial port.")
            return

        # <<< MODIFIED: Look up the actual device name from the map
        selected_port_device = self.port_map.get(selected_display_name)

        if not selected_port_device:
             # This might happen if the list was refreshed between selection and clicking Start
             messagebox.showerror("Port Error", f"Could not find device for '{selected_display_name}'. Please refresh ports and try again.")
             self.refresh_ports() # Refresh to potentially fix the issue
             return

        try:
            # --- Connect to Serial Port ---
            # Common baud rate for Pico, adjust if needed
            # <<< MODIFIED: Use the looked-up device name
            self.serial_port = serial.Serial(selected_port_device, 115200, timeout=1)
            print(f"Connected to {selected_port_device} (selected as '{selected_display_name}') at 115200 baud.")
            # Display the selected *display name* in the status for clarity
            self.status_var.set(f"Status: Running ({selected_display_name})")
            self.is_running = True

            # --- Start Serial Reading Thread ---
            self.stop_thread_flag.clear() # Reset flag before starting
            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_thread.start()

            # --- Update GUI State ---
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.port_combobox.config(state=tk.DISABLED)
            self.refresh_button.config(state=tk.DISABLED)

        except serial.SerialException as e:
            messagebox.showerror("Serial Connection Error", f"Failed to connect to {selected_port_device}:\n{e}")
            self.status_var.set("Status: Connection Error")
            if self.serial_port:
                self.serial_port.close()
            self.serial_port = None
            self.is_running = False # Ensure state is correct
        except Exception as e: # Catch potential other errors during setup
             messagebox.showerror("Error", f"An unexpected error occurred during startup:\n{e}")
             self.status_var.set("Status: Startup Error")
             if self.serial_port and self.serial_port.is_open:
                 self.serial_port.close()
             self.serial_port = None
             self.is_running = False


    def stop_simulation(self):
        """Stops the serial reading thread and closes the connection."""
        if not self.is_running:
            print("Simulation not running.")
            return

        print("Stopping simulation...")
        self.status_var.set("Status: Stopping...")
        self.stop_thread_flag.set() # Signal the thread to stop

        # Wait briefly for the thread to exit gracefully
        if self.serial_thread is not None:
            try:
                self.serial_thread.join(timeout=1.0) # Wait max 1 second
            except Exception as e:
                 print(f"Error joining serial thread: {e}") # Log error but continue

        # Close the serial port if open
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                print("Serial port closed.")
            except Exception as e:
                print(f"Error closing serial port: {e}") # Log error but continue

        self.serial_port = None
        self.serial_thread = None
        self.is_running = False

        # --- Update GUI State ---
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.port_combobox.config(state="readonly") # Allow changing port when stopped
        self.refresh_button.config(state=tk.NORMAL)
        self.status_var.set("Status: Idle")
        print("Simulation stopped.")


    def read_serial_data(self):
        """
        Runs in a separate thread. Reads lines from serial, parses them,
        and puts the data into the queue.
        Expected format: "X,Y,Z,GripperState\n" (e.g., "50.5,25.0,10.0,1")
        where GripperState is 1 for open, 0 for closed.
        """
        print("Serial reading thread started.")
        while not self.stop_thread_flag.is_set():
            # Check serial port validity within the loop for robustness
            if not self.serial_port or not self.serial_port.is_open:
                if not self.stop_thread_flag.is_set(): # Avoid error message if stopping normally
                    print("Serial port closed or invalid in read thread.")
                    self.after(0, self.handle_serial_error, "Serial port became unavailable.")
                break # Exit thread if port is bad

            try:
                if self.serial_port.in_waiting > 0:
                    # Read line, decode, strip whitespace
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        # print(f"Raw serial: {line}") # Uncomment for debugging
                        parts = line.split(',')
                        if len(parts) == 4:
                            try:
                                x = float(parts[0])
                                y = float(parts[1])
                                z = float(parts[2])
                                gripper_state = int(parts[3]) # 1 or 0
                                gripper_open = (gripper_state == 1) # Convert to boolean

                                # Put parsed data into the thread-safe queue
                                self.data_queue.put({'x': x, 'y': y, 'z': z, 'gripper': gripper_open})

                            except ValueError as e:
                                print(f"Warning: Could not parse data '{line}': {e}")
                        else:
                             print(f"Warning: Unexpected data format received: {line}")
                    # else: No complete line received yet

            except serial.SerialException as e:
                # Check if the error is due to the port being closed during shutdown
                if not self.stop_thread_flag.is_set():
                    print(f"Serial error in read thread: {e}")
                    # Signal main thread to handle the error (updates GUI, stops sim)
                    self.after(0, self.handle_serial_error, f"Serial error: {e}")
                break # Exit thread on serial error
            except Exception as e:
                 # Catch other potential errors like read errors
                 if not self.stop_thread_flag.is_set():
                    print(f"Unexpected error in read thread: {e}")
                    # Also signal main thread
                    self.after(0, self.handle_serial_error, f"Unexpected error: {e}")
                 break # Exit thread

            # Small sleep to prevent busy-waiting and yield CPU time
            time.sleep(0.01)

        print("Serial reading thread finished.")

    def handle_serial_error(self, error_message):
        """Handles serial errors detected in the read thread. Called via self.after."""
        if self.is_running: # Only show message and stop if we thought we were running
            messagebox.showerror("Serial Error", f"Connection lost or error occurred:\n{error_message}")
            self.stop_simulation() # Attempt graceful stop
            self.status_var.set(f"Status: Error ({error_message[:30]}...)")
        else:
            # If not running, maybe just log it, as stop_simulation might have already run
            print(f"Serial error handled while not in running state: {error_message}")


    def check_queue(self):
        """
        Periodically checks the data queue (called by tk.after).
        If data is found, updates the robot state and redraws the plot.
        """
        try:
            # Process all available data in the queue
            while not self.data_queue.empty():
                data = self.data_queue.get_nowait()

                # Update the robot's internal state
                self.robot.set_state(data['x'], data['y'], data['z'], data['gripper'])

                # Update the plot elements with the new state
                self.robot.update_plot()

                # Redraw the canvas efficiently
                self.canvas.draw_idle()

        except queue.Empty:
            pass # No new data in the queue
        except Exception as e:
            print(f"Error processing queue data: {e}") # Log other potential errors

        finally:
            # Schedule the next check, always run this unless app is closing
            # Check if the window still exists before scheduling next check
            if self.winfo_exists():
                 self.after(50, self.check_queue) # Check every 50ms

    def on_closing(self):
        """Handles the event when the user closes the window."""
        print("Window closing...")
        if self.is_running:
            # Ask user if they are sure if simulation is running
            if messagebox.askokcancel("Quit", "Simulation is running. Stop and quit?"):
                self.stop_simulation() # Gracefully stop simulation first
                # Wait a very short moment for stop_simulation tasks if needed
                self.after(100, self.destroy) # Close the Tkinter window after a short delay
            else:
                return # Don't close if user cancels
        else:
            # Ensure stop flag is set even if not "running" (e.g., thread error occurred)
            self.stop_thread_flag.set()
            # If serial thread exists, try to join it briefly
            if self.serial_thread and self.serial_thread.is_alive():
                self.serial_thread.join(timeout=0.2)
            self.destroy() # Close if not running

# --- Main Execution ---
if __name__ == "__main__":
    # Set backend explicitly for compatibility if needed, though TkAgg is default with Tkinter
    # import matplotlib
    # matplotlib.use('TkAgg')

    app = RobotSimApp()
    app.mainloop()
