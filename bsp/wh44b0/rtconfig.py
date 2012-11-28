import os

# toolchains options
ARCH     = 'arm'
CPU      = 's3c44b0'
#TextBase = '0x0c008000'
TextBase = '0x0'
POSITION = 'home'
CROSS_TOOL 	= 'gcc'

if os.getenv('RTT_CC'):
	CROSS_TOOL = os.getenv('RTT_CC')

if  CROSS_TOOL == 'gcc':
	PLATFORM 	= 'gcc'
	if POSITION == 'home':
		EXEC_PATH	= '/opt/CrossCompile/CodeSourcery/Sourcery_CodeBench_for_ARM_EABI/bin'
	else:
		EXEC_PATH 	= 'c:/Program Files(x86)/yagarto/bin'
	
elif CROSS_TOOL == 'keil':
	PLATFORM 	= 'armcc'
	EXEC_PATH 	= 'C:/Keil'
elif CROSS_TOOL == 'iar':
    print '================ERROR============================'
    print 'Not support iar yet!'
    print '================================================='
    exit(0)

if os.getenv('RTT_EXEC_PATH'):
	EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debugr'

if PLATFORM == 'gcc':
    # toolchains
	if POSITION == 'home':
		PREFIX = 'arm-none-eabi-'
	else:
		PREFIX = 'arm-elf-'
	CC = PREFIX + 'gcc'
	CXX = PREFIX + 'g++'
	AS = PREFIX + 'gcc'
	AR = PREFIX + 'ar'
	LINK = PREFIX + 'gcc'
	TARGET_EXT = 'axf'
	SIZE = PREFIX + 'size'
	OBJDUMP = PREFIX + 'objdump'
	OBJCPY = PREFIX + 'objcopy'

	DEVICE = ' -mcpu=arm720t'
	CFLAGS = DEVICE
	if TextBase == '0x0':
		TextBase = '0x0c000000'
		AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp' + ' -DTEXT_BASE=' + TextBase + ' -D__FLASH_BUILD__'
		CFLAGS += ' -D__FLASH_BUILD__'
	else:
		AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp' + ' -DTEXT_BASE=' + TextBase
	LFLAGS = DEVICE + ' -Wl,--gc-sections,-Map=rtthread_wh44b0.map,-cref,-u,_rstart -T wh44b0_ram.lds' + ' -Ttext ' + TextBase

	CPATH = ''
	LPATH = ''

	if BUILD == 'debug':
		CFLAGS += ' -O0 -gstabs+'
		AFLAGS += ' -gstabs+'
	else:
		CFLAGS += ' -O2'

	POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + SIZE + ' $TARGET \n'

elif PLATFORM == 'armcc':
    # toolchains
    CC = 'armcc'
    CXX = 'armcc'    
    AS = 'armasm'
    AR = 'armar'
    LINK = 'armlink'
    TARGET_EXT = 'axf'

    DEVICE = ' --device DARMSS9'
    CFLAGS = DEVICE + ' --apcs=interwork --diag_suppress=870'
    AFLAGS = DEVICE
    LFLAGS = DEVICE + ' --strict --info sizes --info totals --info unused --info veneers --list rtthread-mini2440.map --ro-base 0x30000000 --entry Entry_Point --first Entry_Point'

    CFLAGS += ' -I"' + EXEC_PATH + '/ARM/RV31/INC"'
    LFLAGS += ' --libpath "' + EXEC_PATH + '/ARM/RV31/LIB"'

    EXEC_PATH += '/arm/bin40/'

    if BUILD == 'debug':
        CFLAGS += ' -g -O0'
        AFLAGS += ' -g'
    else:
        CFLAGS += ' -O2'

    POST_ACTION = 'fromelf --bin $TARGET --output rtthread.bin \nfromelf -z $TARGET'

elif PLATFORM == 'iar':
    # toolchains
    CC = 'armcc'
    AS = 'armasm'
    AR = 'armar'
    LINK = 'armlink'

    CFLAGS = ''
    AFLAGS = ''
    LFLAGS = ''
