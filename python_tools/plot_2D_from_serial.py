import serial
import matplotlib.pyplot as plt

def real_time_xy_plot(serial_port='COM9', baud_rate=115200):
    """
    Opens the specified serial port, reads lines of the form 'X,Y,Z',
    and plots the X–Y points in real-time (ignoring Z).
    """
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    xs, ys = [], []

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(f'Real-time XY Plot from {serial_port}')
    # fix axes if you like:
    ax.set_xlim(-6000, 6000)
    ax.set_ylim(-6000, 6000)

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) < 2:
                continue
           # Parse the comma-separated values

            try:
                x_str, y_str, z_str = line.split(',')
                x, y, z = float(x_str), float(y_str), float(z_str)
            except ValueError:
                # Skip malformed lines
                continue

            xs.append(x)
            ys.append(y)

            ax.clear()
            ax.scatter(xs, ys, s=10)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title(f'Real-time XY Plot from {serial_port}')
            ax.set_xlim(-4000, 4000)
            ax.set_ylim(-4000, 4000)

            plt.draw()
            plt.pause(0.01)

    except KeyboardInterrupt:
        print("Stopping real-time XY plot.")
    finally:
        ser.close()

if __name__ == '__main__':
    real_time_xy_plot(serial_port='COM9', baud_rate=115200)
