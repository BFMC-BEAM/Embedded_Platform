import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import time
import math
import numpy as np

# Initialize lists to store data
time_data = []
accelx_data = []
accely_data = []
accelz_data = []
velx_data = []
vely_data = []
velz_data = []
x_data = []
y_data = []
z_data = []
yaw_data = []

FFT = True
paused = False

def read_imu_data(port='/dev/ttyACM0', baudrate=115200):
    ser = serial.Serial(port, baudrate)
    ser.write(b'#kl:30;;\r\n')
    time.sleep(0.1)
    ser.write(b'#ultra:0;;\r\n')
    time.sleep(0.1)
    ser.write(b'#ultra:0;;\r\n')
    time.sleep(0.1)
    ser.write(b'#instant:0;;\r\n')
    time.sleep(0.1)
    ser.write(b'#resourceMonitor:0;;\r\n')
    time.sleep(0.1)
    ser.write(b'#vcd:0;;\r\n')
    time.sleep(0.1)
    ser.write(b'#battery:0;;\r\n')
    time.sleep(0.1)
    while True:
        line = ser.readline().decode('utf-8').strip()
        print(line)   
        if line.startswith('@imu:'):
            data = line[5:].split(';')
            if len(data) >= 12:
                try:
                    yaw , pitch, rol, accelx, accely, accelz, velx, vely, velz, x,y,z = map(float, data[:12])
                    current_time = time.time()
                    time_data.append(current_time)

                    accelx_data.append(accelx / 1)
                    accely_data.append(accely / 1)
                    accelz_data.append(accelz / 1)

                    velx_data.append(velx / 1)
                    vely_data.append(vely / 1)
                    velz_data.append(velz / 1)

                    x_data.append(velx / 1)
                    y_data.append(vely / 1)
                    z_data.append(z / 1)
                    
                    yaw_data.append(yaw)

                    print(accelz,velz,z)
                except ValueError as e:
                    print(f"Error converting data to float: {e}")

def update_plot(frame):
    if paused:
        return

    current_time = time.time()
    window_time = 50  # 10 second window

    # Filter data to keep only the last 10 seconds
    while time_data and current_time - time_data[0] > window_time:
        time_data.pop(0)
        accelx_data.pop(0)
        accely_data.pop(0)
        accelz_data.pop(0)
        velx_data.pop(0)
        vely_data.pop(0)
        velz_data.pop(0)
        x_data.pop(0)
        y_data.pop(0)
        z_data.pop(0)
        yaw_data.pop(0)

    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()

    ax1.plot(time_data, accelx_data, label='Accel X')
    ax1.plot(time_data, accely_data, label='Accel Y')
    ax1.set_title('Acceleration')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(time_data, velx_data, label='Vel X')
    ax2.plot(time_data, vely_data, label='Vel Y')
    ax2.set_title('Velocity')
    ax2.legend()
    ax2.grid(True)

    ax3.plot(y_data, x_data, label='Trajectory')
    if x_data and y_data and yaw_data:
        ax3.plot(y_data[-1], x_data[-1], 'ro')  # Plot the current position as a red dot
        yaw_rad = math.radians(yaw_data[-1])
        vector_length = 0.5  # Length of the vector
        dx = vector_length * math.cos(yaw_rad)
        dy = vector_length * math.sin(yaw_rad)
        ax3.arrow(y_data[-1], x_data[-1], dy, dx, head_width=0.1, head_length=0.1, fc='blue', ec='blue')
    ax3.set_title('2D Position (X vs Y)')
    ax3.set_xlabel('X Position')
    ax3.set_ylabel('Y Position')
    ax3.set_xlim(-6, 6)
    ax3.set_ylim(-6, 6)
    ax3.legend()
    ax3.grid(True)

    if FFT:
        # FFT of Acceleration X
        if len(accely_data) > 1:
            N = len(accely_data)
            T = 1.0 / 100.0  # Sample spacing with 100 Hz sampling frequency
            yf = np.fft.fft(accely_data)
            xf = np.fft.fftfreq(N, T)[:N//2]
            amplitude = 2.0/N * np.abs(yf[:N//2])
            amplitude_db = 20 * np.log10(amplitude)
            ax4.plot(xf, amplitude_db)
            ax4.set_title('FFT of Acceleration X')
            ax4.set_xlabel('Frequency (Hz)')
            ax4.set_ylabel('Amplitude (dB)')
            ax4.grid(True)

def toggle_pause(event):
    global paused
    paused = not paused

if __name__ == "__main__":
    import threading
    data_thread = threading.Thread(target=read_imu_data)
    data_thread.daemon = True
    data_thread.start()

    # Create subplots
    fig = plt.figure(figsize=(15, 10))
    ax1 = plt.subplot2grid((3, 2), (0, 0))
    ax2 = plt.subplot2grid((3, 2), (1, 0))
    ax3 = plt.subplot2grid((3, 2), (0, 1), rowspan=2)
    ax4 = plt.subplot2grid((3, 2), (2, 0), colspan=2)
    ani = FuncAnimation(fig, update_plot, interval=10)  # Update interval set to 10 ms

    # Add a button to pause/resume the animation
    pause_ax = plt.axes([0.81, 0.01, 0.1, 0.075])
    pause_button = Button(pause_ax, 'Pause')
    pause_button.on_clicked(toggle_pause)

    plt.tight_layout()
    plt.show()