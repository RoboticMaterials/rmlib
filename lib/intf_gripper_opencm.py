import serial
import time


class OpenCM:
    def __init__(self):

        # set constants
        self.MAX_WIDTH = 108
        self.MIN_WIDTH = -60
        self.MAX_TORQUE = 200

        # init serial connection
        port = "/dev/ttyACM0"
        timeout = 3
        self.ser = serial.Serial(port=port, timeout=timeout)
        self.ser.close()
        self.ser.open()

# ======================
# Gripper Setup Commands
# ======================

    def activate_gripper(self):
        self.ser.flushInput()
        self.ser.write('a\r'.encode())
        return 1

    def deactivate_gripper(self):
        self.ser.flushInput()
        self.ser.write('d\r'.encode())
        return 1

    def zero_gripper(self):
        self.ser.flushInput()
        self.ser.write('z\r'.encode())
        time.sleep(15)
        return 1

# ====================
# Gripper Set Commands
# ====================

    def open_gripper(self, blocking=True):
        """
        Opens gripper.

        Return
        ------
        success: bool
            If the gripper opens completely, success is True.
        """

        self.ser.flushInput()
        self.ser.write('o\r'.encode())
        if blocking:
            return bool(int(self.ser.readline().decode()[0]))
        else:
            return True

    def close_gripper(self, blocking=True):
        """
        Closes gripper.

        Return 
        ------
        success: bool
            If the gripper closes completely, success is False, else success is True.
        """

        self.ser.flushInput()
        self.ser.write('c\r'.encode())
        if blocking:
            return bool(int(self.ser.readline().decode()[0]))
        else:
            return True

    def set_gripper_torque(self, torque):
        """
        Set torque of both gripper fingers.

        Parameters
        ----------
        torque: int
            Torque value between 0 and 1.

        Return 
        ------
        1
        """
        torque = torque*self.MAX_TORQUE
        if torque > self.MAX_TORQUE:
            torque = self.MAX_TORQUE
        elif torque < 0:
            torque = 0
        self.ser.write('t{}\r'.format(torque).encode())
        return True

    def set_gripper_torque_right(self, torque):
        """
        Set torque of right gripper finger.

        Parameters
        ----------
        torque: int
            Torque value between 0 and 1.

        Return 
        ------
        1
        """
        torque = torque*self.MAX_TORQUE
        if torque > self.MAX_TORQUE:
            torque = self.MAX_TORQUE
        elif torque < 0:
            torque = 0
        self.ser.flushInput()
        self.ser.write(('){}\r'.format(torque)).encode())
        return True

    def set_gripper_torque_left(self, torque):
        """
        Set torque of left gripper finger.

        Parameters
        ----------
        torque: int
            Torque value between 0 and 1.

        Return 
        ------
        1
        """
        torque = torque*self.MAX_TORQUE
        if torque > self.MAX_TORQUE:
            torque = self.MAX_TORQUE
        elif torque < 0:
            torque = 0
        self.ser.flushInput()
        self.ser.write(('({}\r'.format(torque)).encode())
        return True

    def set_gripper_width(self, width, blocking=True, width2=None):
        """
        Set width between fingers.

        Parameters 
        ----------
        width: float
            If only 1 argument is passed, this width is the width between fingers (m). \
            If 2 arguments are passed, this is the distance from the right finger to 0.
        width2: float (optional)
            If this argument is passed, this will be the distance from the right finger to 0. (m)

        Return
        ------
        Success: bool
        """

        if width2 is not None:
            width2 = int(float(width2*2) * 1000.0)
            if width2 > self.MAX_WIDTH:
                width2 = self.MAX_WIDTH
            elif width2 < self.MIN_WIDTH:
                width2 = self.MIN_WIDTH

            width = int(float(width*2) * 1000.0)
            if width > self.MAX_WIDTH:
                width = self.MAX_WIDTH
            elif width < self.MIN_WIDTH:
                width = self.MIN_WIDTH

        else:
            width = int(float(width) * 1000.0)
            if width > self.MAX_WIDTH:
                width = self.MAX_WIDTH
            elif width < 0:
                width = 0

        self.ser.flushInput()
        if width2 is not None:
            self.ser.write(('w{:03d},{:03d}\r'.format(width, width2)).encode())
        else:
            self.ser.write(('w{:03d}\r'.format(width)).encode())

        if blocking:
            return bool(self.ser.readline().decode()[0])
        else:
            return True

    def set_gripper_width_right(self, width, blocking=True):
        """
        Set width between right finger and 0 position.

        Parameters 
        ----------
        width: float
            Width to be set (m).

        Return
        ------
        Success: bool
        """
        width = int(float(width) * 2000.0)
        if width > self.MAX_WIDTH:
            width = self.MAX_WIDTH
        elif width < self.MIN_WIDTH:
            width = self.MIN_WIDTH
        self.ser.write(('>{}\r'.format(width)).encode())
        if blocking:
            return bool(self.ser.readline().decode()[0])
        else:
            return True

    def set_gripper_width_left(self, width, blocking=True):
        """
        Set width between left finger and 0 position.

        Parameters 
        ----------
        width: float
            Width to be set (m).

        Return
        ------
        Success: bool
        """
        width = int(float(width) * 2000.0)
        if width > self.MAX_WIDTH:
            width = self.MAX_WIDTH
        elif width < self.MIN_WIDTH:
            width = self.MIN_WIDTH
        self.ser.flushInput()
        self.ser.write(('<{}\r'.format(width)).encode())
        if blocking:
            return bool(self.ser.readline().decode()[0])
        else:
            return True

    def set_min_object_size(self, object_size):
        """
        Set the threshold of what is considered a successful close.

        Parameters
        ----------
        object_size: float
            Width of smallest acceptable object (m).

        Return
        ------
        Success: bool
        """
        self.ser.flushInput()
        self.ser.write(('m{}\r'.format(object_size)).encode())
        return int(self.ser.readline().decode()[0])

    def set_red_led(self, intensity):
        """
        Set red LED color.

        Parameters
        ----------
        intensity: int
            Intensity value between 0 and 255.
        """
        intensity = 255-intensity
        self.ser.flushInput()
        self.ser.write(('r{}\r'.format(intensity)).encode())
        return

    def set_green_led(self, intensity):
        """
        Set green LED color.

        Parameters
        ----------
        intensity: int
            Intensity value between 0 and 255.
        """
        intensity = 255-intensity
        self.ser.flushInput()
        self.ser.write(('g{}\r'.format(intensity)).encode())
        return

    def set_blue_led(self, intensity):
        """
        Set blue LED color.

        Parameters
        ----------
        intensity: int
            Intensity value between 0 and 255.
        """
        intensity = 255-intensity
        self.ser.flushInput()
        self.ser.write(('b{}\r'.format(intensity)).encode())
        return

    def set_gripper_color(self, r, g, b):
        self.set_red_led(r)
        self.set_green_led(g)
        self.set_blue_led(b)
        return

    def set_dance_mode(self):
        self.ser.flushInput()
        self.ser.write('%\r'.encode())
        return int(self.ser.readline().decode()[0])


# ====================
# Get Gripper Commands
# ====================

    def get_firmware_version(self):
        """
        Gets current software version date.

        Return 
        ------
        version_data: str
            The date that the current software was published. 
        """
        self.ser.flushInput()
        self.ser.write('S\r'.encode())
        return int(self.ser.readline().decode())

    def get_gripper_width(self):
        """
        Gets the width between gripper fingers.

        Return
        ------
        width: float
            Current width of fingers (m).
        """
        self.ser.flushInput()
        self.ser.write('W\r'.encode())
        width = int(self.ser.readline().decode()[:-2])
        width /= 1000
        return width

    def get_gripper_width_right(self):
        """
        Gets the width between gripper finger and 0 position.

        Return
        ------
        width: float
            Current width of finger (m).
        """
        self.ser.flushInput()
        self.ser.write('R\r'.encode())
        width = int(self.ser.readline().decode()[:-2])
        width /= 1000
        return width

    def get_gripper_width_left(self):
        """
        Gets the width between gripper finger and 0 position.

        Return
        ------
        width: float
            Current width of finger (m).
        """
        self.ser.flushInput()
        self.ser.write('L\r'.encode())
        width = int(self.ser.readline().decode()[:-2])
        width /= 1000
        return width

    def get_motor_temperature(self):
        self.ser.flushInput()
        self.ser.write('T\r'.encode())
        string = self.ser.readline().decode()[:-2]
        temps = [int(digit) for digit in string.split(' ')]
        return temps
