import sys
from global_params import *
import digit_interface as Digit


def connectDigit(intensity=8):
    digits = Digit.DigitHandler.list_digits()
    if len(digits) == 0:
        sys.exit("No Digit Found uwu")

    digit = Digit.Digit(digits[0]["serial"])
    digit.connect()
    digit.set_intensity(intensity)
    if ifVGA:
        digit.set_resolution(digit.STREAMS["VGA"])
    return digit


def main():

    pass


if __name__ == "__main__":
    main()
