import os
import time

import serial

from serial.tools import list_ports


def available_ports():

    allowed_prefixes = (
        "/dev/ttyUSB",
        "/dev/ttyACM",
    )

    return [
        port.device
        for port in list_ports.comports()
        if port.device.startswith(
            allowed_prefixes
        )
    ]

def open_serial(
    port_name,
    baud_rate,
    timeout=0.1,
):

    arguments = {
        "port": port_name,
        "baudrate": baud_rate,
        "timeout": timeout,
    }

    # Prevent two ROS bridges from probing the same
    # Linux serial device at exactly the same time.
    if os.name == "posix":

        arguments["exclusive"] = True

    return serial.Serial(
        **arguments
    )


def line_matches_role(
    line,
    role,
):

    line = line.strip().lower()

    if role == "bench":

        return (
            "laser=" in line
            and
            "force=" in line
        )

    if role == "actuator":

        return (
            "pressure=" in line
        )

    return False


def probe_port(
    port_name,
    baud_rate,
    role,
    startup_delay=1.0,
    probe_duration=2.0,
):

    serial_port = None

    try:

        serial_port = open_serial(
            port_name,
            baud_rate,
            timeout=0.1,
        )

        # Some ESP32 USB interfaces reset the board
        # when the serial connection is opened.
        time.sleep(
            startup_delay
        )

        try:

            serial_port.reset_input_buffer()

        except (
            serial.SerialException,
            OSError,
        ):

            pass

        deadline = (
            time.monotonic()
            +
            probe_duration
        )

        while (
            time.monotonic()
            <
            deadline
        ):

            line = (
                serial_port
                .readline()
                .decode(
                    "utf-8",
                    errors="ignore",
                )
                .strip()
            )

            if not line:

                continue

            if line_matches_role(
                line,
                role,
            ):

                return serial_port

        serial_port.close()

        return None

    except (
        serial.SerialException,
        OSError,
    ):

        if serial_port is not None:

            try:

                serial_port.close()

            except Exception:

                pass

        return None


def discover_serial_device(
    baud_rate,
    role,
):

    for port_name in available_ports():

        serial_port = probe_port(
            port_name,
            baud_rate,
            role,
        )

        if serial_port is not None:

            return (
                port_name,
                serial_port,
            )

    return (
        None,
        None,
    )
    