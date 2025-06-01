import serial
import matplotlib.pyplot as plt

def real_time_3d_plot(serial_port='COM9', baud_rate=9600):
    """
    Opens the specified serial port, reads lines of the form 'X,Y,Z',
    and plots the 3D points in real-time.
    """
    # Initialize serial connection
    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    # Initialize data lists
    xs, ys, zs = [], [], []

    # Set up the 3D plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Real-time 3D Plot from {serial_port}')
    # Fix axes to –6000…+6000
    ax.set_xlim(-6000, 6000)
    ax.set_ylim(-6000, 6000)
    ax.set_zlim(-6000, 6000)

    try:
        while True:
            # Read and decode a line from the serial port
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue

            # Parse the comma-separated values
            try:
                x_str, y_str, z_str = line.split(',')
                x, y, z = float(x_str), float(y_str), float(z_str)
            except ValueError:
                # Skip malformed lines
                continue
            # Append new point to data lists
            xs.append(x)
            ys.append(y)
            zs.append(z)

            # Clear and redraw scatter plot
            ax.clear()
            ax.scatter(xs, ys, zs)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(f'Real-time 3D Plot from {serial_port}')
            # Fix axes to –6000…+6000
            ax.set_xlim(-6000, 6000)
            ax.set_ylim(-6000, 6000)
            ax.set_zlim(-6000, 6000)

            # Brief pause to render the plot
            plt.draw()
            plt.pause(0.01)

    except KeyboardInterrupt:
        print("Stopping real-time plot.")
    finally:
        ser.close()

if __name__ == '__main__':
    real_time_3d_plot(serial_port='COM9', baud_rate=115200)
