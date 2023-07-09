import time

from evdev import InputDevice, categorize, ecodes

key_map: dict = {
    "xBtn": 307,
    "yBtn": 308,
    "aBtn": 304,
    "bBtn": 305,
    "upBtn": 544,
    "downBtn": 545,
    "leftBtn": 546,
    "rightBtn": 547,
    "lBtn": 310,
    "rBtn": 311,
    "lTrig": 312,
    "rTrig": 313,
    "pressL": 317,
    "pressR": 318,
    "startBtn": 315,
    "selectBtn": 314,
    "lStickX": 0,
    "lStickY": 1,
    "rStickX": 2,
    "rStickY": 5,
    "dpadX": 16,
    "dpadY": 17,

}


def communication_with_evdev() -> None:
    gamepad = InputDevice("/dev/input/event3")
    print(gamepad)
    for event in gamepad.read_loop():
        # print(categorize(event))
        if event.code in key_map.values():
            print(f"event: {event.code} | value:{event.value} | type:{event.type})")
        else:
            # print(f"event of type {event.type} with code {event.code} and value {event.value} Not in key map")
            pass


if __name__ == "__main__":
    communication_with_evdev()
