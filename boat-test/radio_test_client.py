from boat_simulation.hardware_tests.radio_test import Robot


def main():
    r = Robot()

    test = ""
    for i in range(252):
        test += 'e'

    r.transmit_message(test)


if __name__ == '__main__':
    main()
