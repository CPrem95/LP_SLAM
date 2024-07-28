import matplotlib.pyplot as plt
import numpy as np
import threading
import time
'''
# Function to update the plot
def update_plot(line, stop_event):
    x = np.linspace(0, 2 * np.pi, 100)
    while not stop_event.is_set():
        y = np.sin(x + time.time())
        line.set_ydata(y)
        plt.draw()
        plt.pause(0.1)
        if stop_event.is_set():
            break

# Setting up the plot
fig, ax = plt.subplots()
x = np.linspace(0, 2 * np.pi, 100)
y = np.sin(x)
line, = ax.plot(x, y)
plt.ylim(-1.5, 1.5)

# Event to signal the thread to stop
stop_event = threading.Event()

# Start the thread
thread = threading.Thread(target=update_plot, args=(line, stop_event))
thread.start()

# To stop the thread after some time (e.g., 10 seconds)
time.sleep(10)
stop_event.set()
thread.join()

# Close the plot after stopping the thread
plt.close(fig)
'''

global_var = 0

# Function to run in the thread
def additional_thread_function():
    print("Additional thread started.")
    while True:
        print("Additional thread is running. ", global_var)
        time.sleep(0.5)

# Create a new thread
additional_thread = threading.Thread(target=additional_thread_function)

# Start the additional thread
additional_thread.start()

# Main loop
while True:
    print("Main loop is running.")
    time.sleep(2)
    global_var += 2