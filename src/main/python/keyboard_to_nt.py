from pynput import keyboard
import ntcore
import time

# Tracks currently pressed keys to avoid repeat events
pressed_keys = set()

def main():
    ntcoreinst = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client")
    ntcoreinst.startClient4("KeyboardToNT")
    ntcoreinst.setServer("127.0.0.1")  # Change if your robot is remote
    ntcoreinst.startDSClient()

    print("Waiting for connection to NetworkTables server...")
    while not ntcoreinst.isConnected():
        time.sleep(0.1)

    print("Connected!")
    table = ntcoreinst.getTable("SmartDashboard/keyboard")

    def on_press(key):
        try:
            name = key.char.lower()
        except AttributeError:
            name = str(key).replace("Key.", "").lower()

        if name not in pressed_keys:
            pressed_keys.add(name)
            table.putBoolean(name, True)

    def on_release(key):
        try:
            name = key.char.lower()
        except AttributeError:
            name = str(key).replace("Key.", "").lower()

        if name in pressed_keys:
            pressed_keys.remove(name)
            table.putBoolean(name, False)

        # Optional: exit on ESC
        if key == keyboard.Key.esc:
            return False

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

if __name__ == '__main__':
    main()
