Import("env")
from distutils.dir_util import copy_tree
import os
# access to global construction environment
print(env)

#only copy these files , delete ones in .platformio subfolder first to ensure build updates correctly.
PIN_NAMES_VAR = "PinNamesVar.h"
LD_SCRIPT = "ldscript.ld"
PERIPHERALS_PINS = "PeripheralPins.C"
VARIANT_C = "variant.cpp"
VARIANT_H = "variant.h"

# Get Arduino Variant Path for STM32 Platform
FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduinoststm32")
#ENV_TEXT = env['stm_platform']
#print(ENV_TEXT)
VARIANT_DIR = FRAMEWORK_DIR + "/variants"
#BOARDS_MANANAGER_FILE = env.PioPlatform().get_platforms_dir("ststm32")
#BMG_FILE =BOARDS_MANANAGER_FILE + "/boards/"
BOARD_DEST_DIR = "/MKSTFT_F107VC/"
VARIANT_SRC_DIR = env['PROJECT_DIR'] + "/buildroot/arduino_variant"

# Copy variant in this repo to platform-variant folder
if os.path.isfile(VARIANT_DIR + BOARD_DEST_DIR + PIN_NAMES_VAR):
    os.remove(VARIANT_DIR + BOARD_DEST_DIR + PIN_NAMES_VAR)
    os.remove(VARIANT_DIR + BOARD_DEST_DIR + LD_SCRIPT)
    os.remove(VARIANT_DIR + BOARD_DEST_DIR + PERIPHERALS_PINS)
    os.remove(VARIANT_DIR + BOARD_DEST_DIR + VARIANT_C)
    os.remove(VARIANT_DIR + BOARD_DEST_DIR + VARIANT_H)
#os.rmdir(VARIANT_DIR)
#shutil.rmtree(VARIANT_DIR)
else:
    print(env)
copy_tree(VARIANT_SRC_DIR, VARIANT_DIR)

