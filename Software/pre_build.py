Import("env")
import os
import os.path
import shutil

def get_version(file_name):
    with open(file_name, 'r') as read_obj:
        for line in read_obj:
            # For each line, check if line contains the string
            if "#define VERSION" in line:
                token = line.split(" ")
                return token[2].replace('\n', '')

    return '0'

print("PRE SCRIPT: build")

# access to global build environment
#print(env)

# access to project build environment (is used source files in "src" folder)
#print(projenv)

#
# Dump build environment (for debug purpose)
#print(env.Dump())
#

#firmware_name
# Change build flags in runtime
#
#env.ProcessUnFlags("-DVECT_TAB_ADDR")
#env.Append(CPPDEFINES=("VECT_TAB_ADDR", 0x123456789))

#print("Current build targets", map(str, BUILD_TARGETS))


#rename 
#build_flags = env.ParseFlags(env['BUILD_FLAGS'])
#defines = {k: v for (k, v) in build_flags.get('CPPDEFINES')}

#add version
#firmware_name = 'firmware_v%s' % (get_version('src/version.h'))
#firmware_name = firmware_name.strip()

#add revision
#firmware_name = '%s-%s' % (firmware_name, defines.get("REVISION"))

#add cpu type
#if 'stm32l412cbt6' in env['BOARD_MCU']:
#    firmware_name = '%s%s' % (firmware_name, '_L412')
#else:
#    firmware_name = '%s%s' % (firmware_name, '_XXXX')

#add '_'
#if int(defines.get('WATCHDOG')) == 1 or int(defines.get('USE_LED')) == 1:
#    firmware_name = '%s%s' % (firmware_name, '_')

#add watchdog
#firmware_name = '%s%s' % (firmware_name, 'w' if int(defines.get('WATCHDOG')) == 1 else '')

#add led
#firmware_name = '%s%s' % (firmware_name, 'l' if int(defines.get('USE_LED')) == 1 else '')

#add fec
#firmware_name = '%s%s' % (firmware_name, 'f' if int(defines.get('USE_FEC')) == 1 else '')

#add boost
#firmware_name = '%s%s' % (firmware_name, 'b' if int(defines.get('USE_BOOST')) == 1 else '')

#add filter
#firmware_name = '%s%s' % (firmware_name, 'F' if int(defines.get('TELEMETRY_FILTER')) == 1 else '')

#add filter
#firmware_name = '%s%s' % (firmware_name, 'B' if int(defines.get('RESCUE_BATT')) == 1 else '')

#add profile static
#firmware_name = '%s%s' % (firmware_name, 'P' if int(defines.get('PROFIL_STATIC')) == 1 else '')

#add pn9
#firmware_name = '%s%s' % (firmware_name, '_PN9' if int(defines.get('MODE_PN9')) == 1 else '')

#add spy
#firmware_name = '%s%s' % (firmware_name, '_SPY' if int(defines.get('MODEM_SPY')) == 1  else '')

#add HSE
#firmware_name = '%s%s' % (firmware_name, '_HSE' if int(defines.get('USE_HSI')) == 0  else '')

#add certif
#firmware_name = '%s%s' % (firmware_name, '_certif' if int(defines.get('CERTIF')) == 1  else '')

#add pn9_2fsk
#firmware_name = '%s%s' % (firmware_name, '_pn9_2fsk' if int(defines.get('CERTIF_PN9_2FK')) == 1  else '')

#add pn9_cw
#firmware_name = '%s%s' % (firmware_name, '_pn9_cw' if int(defines.get('CERTIF_PN9_CW')) == 1  else '')

#add p<power>
#firmware_name = '%s%s' % (firmware_name, '_P' + defines.get('OUTPUT_RF_POWER') if int(defines.get('OUTPUT_RF_POWER')) != 0  else '_P14Default')

#add baseband
#if int(defines.get('MODEM_TPMS')) == 0:
#    firmware_name = '%s%s' % (firmware_name, '_868' if int(defines.get('BASE_BAND')) == 0  else '_433')

#add tpms
#firmware_name = '%s%s' % (firmware_name, '_tpms' if int(defines.get('MODEM_TPMS')) == 1  else '')

#add USE_DYNAMIC_LOW_SPEED_OSC
#firmware_name = '%s%s' % (firmware_name, '_d' if int(defines.get('USE_DYNAMIC_LOW_SPEED_OSC')) == 1 else '_s')

#add LS
#firmware_name = '%s%s' % (firmware_name, 'LSI' if int(defines.get('USE_LSI')) == 1 else 'LSE')

#env.Replace(PROGNAME='%s' % (firmware_name))

