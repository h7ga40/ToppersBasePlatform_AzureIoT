# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-GCC-ARM-Embedded

###############################################################################
# Boiler-plate

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = '$(SHELL)' -c "mkdir -p \"$(1)\""
    RM = '$(SHELL)' -c "rm -rf \"$(1)\""
endif

OBJDIR := Debug
# Move to the build directory
ifeq (,$(filter $(OBJDIR),$(notdir $(CURDIR))))
.SUFFIXES:
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = '$(MAKE)' --no-print-directory -C $(OBJDIR) -f '$(mkfile_path)' \
		'SRCDIR=$(CURDIR)' $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

# trick rules into thinking we are in the root, when we are in the bulid dir
VPATH = ../../azure_iot_sdk

# Boiler-plate
###############################################################################
# Project settings

PROJECT := azure_iot_sdk


# Project settings
###############################################################################
# Objects and Paths

OBJECTS += c-utility/adapters/agenttime_mbed.o
OBJECTS += c-utility/adapters/httpapi_compact.o
OBJECTS += c-utility/adapters/lock_toppers.o
OBJECTS += c-utility/adapters/platform_toppers.o
OBJECTS += c-utility/adapters/socketio_lwip.o
OBJECTS += c-utility/adapters/threadapi_toppers.o
OBJECTS += c-utility/adapters/tickcounter_toppers.o
OBJECTS += c-utility/adapters/tlsio_wolfssl.o
OBJECTS += c-utility/adapters/uniqueid_stub.o
OBJECTS += c-utility/src/azure_base32.o
OBJECTS += c-utility/src/azure_base64.o
OBJECTS += c-utility/src/buffer.o
OBJECTS += c-utility/src/connection_string_parser.o
OBJECTS += c-utility/src/consolelogger.o
OBJECTS += c-utility/src/constbuffer.o
OBJECTS += c-utility/src/constmap.o
OBJECTS += c-utility/src/crt_abstractions.o
OBJECTS += c-utility/src/dns_resolver_sync.o
OBJECTS += c-utility/src/doublylinkedlist.o
OBJECTS += c-utility/src/gb_rand.o
OBJECTS += c-utility/src/gb_stdio.o
OBJECTS += c-utility/src/gb_time.o
OBJECTS += c-utility/src/gballoc.o
OBJECTS += c-utility/src/hmac.o
OBJECTS += c-utility/src/hmacsha256.o
OBJECTS += c-utility/src/http_proxy_io.o
OBJECTS += c-utility/src/httpapiex.o
OBJECTS += c-utility/src/httpapiexsas.o
OBJECTS += c-utility/src/httpheaders.o
OBJECTS += c-utility/src/map.o
OBJECTS += c-utility/src/optionhandler.o
OBJECTS += c-utility/src/sastoken.o
OBJECTS += c-utility/src/sha1.o
OBJECTS += c-utility/src/sha224.o
OBJECTS += c-utility/src/sha384-512.o
OBJECTS += c-utility/src/singlylinkedlist.o
OBJECTS += c-utility/src/string_token.o
OBJECTS += c-utility/src/string_tokenizer.o
OBJECTS += c-utility/src/strings.o
OBJECTS += c-utility/src/urlencode.o
OBJECTS += c-utility/src/usha.o
OBJECTS += c-utility/src/utf8_checker.o
OBJECTS += c-utility/src/uuid.o
OBJECTS += c-utility/src/uws_client.o
OBJECTS += c-utility/src/uws_frame_encoder.o
OBJECTS += c-utility/src/vector.o
OBJECTS += c-utility/src/wsio.o
OBJECTS += c-utility/src/xio.o
OBJECTS += c-utility/src/xlogging.o
OBJECTS += certs/certs.o
OBJECTS += deps/parson/parson.o
OBJECTS += deps/uhttp/src/uhttp.o
OBJECTS += iothub_client/src/blob.o
OBJECTS += iothub_client/src/iothub.o
OBJECTS += iothub_client/src/iothub_client.o
OBJECTS += iothub_client/src/iothub_client_authorization.o
OBJECTS += iothub_client/src/iothub_client_core.o
OBJECTS += iothub_client/src/iothub_client_core_ll.o
OBJECTS += iothub_client/src/iothub_client_diagnostic.o
OBJECTS += iothub_client/src/iothub_client_ll.o
OBJECTS += iothub_client/src/iothub_client_ll_uploadtoblob.o
OBJECTS += iothub_client/src/iothub_client_retry_control.o
OBJECTS += iothub_client/src/iothub_device_client_ll.o
OBJECTS += iothub_client/src/iothub_message.o
OBJECTS += iothub_client/src/iothub_transport_ll_private.o
OBJECTS += iothub_client/src/iothubtransport.o
OBJECTS += iothub_client/src/iothubtransport_mqtt_common.o
OBJECTS += iothub_client/src/iothubtransporthttp.o
OBJECTS += iothub_client/src/iothubtransportmqtt.o
OBJECTS += iothub_client/src/iothubtransportmqtt_websockets.o
OBJECTS += iothub_client/src/version.o
OBJECTS += provisioning_client/adapters/hsm_client_data.o
OBJECTS += provisioning_client/adapters/hsm_client_key.o
OBJECTS += provisioning_client/src/iothub_auth_client.o
OBJECTS += provisioning_client/src/iothub_security_factory.o
OBJECTS += provisioning_client/src/prov_auth_client.o
OBJECTS += provisioning_client/src/prov_device_client.o
OBJECTS += provisioning_client/src/prov_device_ll_client.o
OBJECTS += provisioning_client/src/prov_security_factory.o
OBJECTS += provisioning_client/src/prov_transport_http_client.o
OBJECTS += provisioning_client/src/prov_transport_mqtt_client.o
OBJECTS += provisioning_client/src/prov_transport_mqtt_common.o
OBJECTS += provisioning_client/src/prov_transport_mqtt_ws_client.o
OBJECTS += serializer/src/methodreturn.o
OBJECTS += serializer/src/multitree.o
OBJECTS += serializer/src/schema.o
OBJECTS += serializer/src/schemalib.o
OBJECTS += serializer/src/schemaserializer.o
OBJECTS += serializer/src/datamarshaller.o
OBJECTS += serializer/src/datapublisher.o
OBJECTS += serializer/src/dataserializer.o
OBJECTS += serializer/src/iotdevice.o
OBJECTS += serializer/src/jsondecoder.o
OBJECTS += serializer/src/jsonencoder.o
OBJECTS += serializer/src/agenttypesystem.o
OBJECTS += serializer/src/codefirst.o
OBJECTS += serializer/src/commanddecoder.o
OBJECTS += umqtt/src/mqtt_client.o
OBJECTS += umqtt/src/mqtt_codec.o
OBJECTS += umqtt/src/mqtt_message.o

INCLUDE_PATHS += -I../../asp_baseplatform
INCLUDE_PATHS += -I../../asp_baseplatform/include
INCLUDE_PATHS += -I../../asp_baseplatform/arch
INCLUDE_PATHS += -I../../asp_baseplatform/arch/arm_m_gcc/common
INCLUDE_PATHS += -I../../asp_baseplatform/arch/arm_m_gcc/stm32f7xx
INCLUDE_PATHS += -I../../asp_baseplatform/target/stm32f767nucleo144_gcc
INCLUDE_PATHS += -I../../asp_baseplatform/lwip/lwip-2.1.2/src/include
INCLUDE_PATHS += -I../../asp_baseplatform/lwip/lwip-2.1.2/src/include/compat/posix
INCLUDE_PATHS += -I../../asp_baseplatform/lwip/contrib-2.1.0/ports/toppers/include
INCLUDE_PATHS += -I../../asp_baseplatform/lwip/contrib-2.1.0
INCLUDE_PATHS += -I../../asp_baseplatform/OBJ/STM32F767NUCLEO144_GCC/MAC
INCLUDE_PATHS += -I../../asp_baseplatform/monitor
INCLUDE_PATHS += -I../../asp_baseplatform/files
INCLUDE_PATHS += -I../../azure_iot_sdk/c-utility/inc
INCLUDE_PATHS += -I../../azure_iot_sdk/c-utility/pal/toppers
INCLUDE_PATHS += -I../../azure_iot_sdk/iothub_client/inc
INCLUDE_PATHS += -I../../azure_iot_sdk/iothub_client/inc/internal
INCLUDE_PATHS += -I../../azure_iot_sdk/provisioning_client/adapters
INCLUDE_PATHS += -I../../azure_iot_sdk/provisioning_client/inc
INCLUDE_PATHS += -I../../azure_iot_sdk/provisioning_client/inc/azure_prov_client/internal
INCLUDE_PATHS += -I../../azure_iot_sdk/deps/parson
INCLUDE_PATHS += -I../../azure_iot_sdk/deps/azure-macro-utils-c/inc
INCLUDE_PATHS += -I../../azure_iot_sdk/deps/uhttp/inc
INCLUDE_PATHS += -I../../azure_iot_sdk/deps/umock-c/inc
INCLUDE_PATHS += -I../../azure_iot_sdk/serializer/inc
INCLUDE_PATHS += -I../../azure_iot_sdk/umqtt/inc
INCLUDE_PATHS += -I../../wolfssl-4.7.0/wolfssl
INCLUDE_PATHS += -I../../wolfssl-4.7.0/wolfssl/wolfcrypt
INCLUDE_PATHS += -I../../wolfssl-4.7.0
INCLUDE_PATHS += -I../../zlib-1.2.11/src

# Objects and Paths
###############################################################################
# Tools and Flags

AS      = arm-none-eabi-gcc
CC      = arm-none-eabi-gcc
CPP     = arm-none-eabi-g++
AR      = arm-none-eabi-ar
RANLIB  = arm-none-eabi-ranlib


C_FLAGS += -std=gnu11
C_FLAGS += -mcpu=cortex-m4
C_FLAGS += -march=armv7e-m
C_FLAGS += -mthumb
C_FLAGS += -mlittle-endian
C_FLAGS += -mfloat-abi=hard
C_FLAGS += -mfpu=fpv4-sp-d16
C_FLAGS += -Og
C_FLAGS += -fmessage-length=0
C_FLAGS += -fsigned-char
C_FLAGS += -ffunction-sections
C_FLAGS += -fdata-sections
C_FLAGS += -Wall
C_FLAGS += -Wextra
C_FLAGS += -Wno-unused-parameter
C_FLAGS += -Wno-missing-field-initializers
C_FLAGS += -Wnull-dereference

C_FLAGS += -Wno-unused-variable
C_FLAGS += -Wno-unused-function

C_FLAGS += -g3
C_FLAGS += -Wstack-usage=256

C_FLAGS += -DWOLFSSL_USER_SETTINGS
C_FLAGS += -DCYASSL_USER_SETTINGS
C_FLAGS += -DUSE_WOLFSSL=1
C_FLAGS += -DHSM_TYPE_SYMM_KEY
C_FLAGS += -DUSE_PROV_MODULE
C_FLAGS += -Dget_time=azure_get_time

CXX_FLAGS += -std=gnu++11
CXX_FLAGS += -mcpu=cortex-m4
CXX_FLAGS += -march=armv7e-m
CXX_FLAGS += -mthumb
CXX_FLAGS += -mlittle-endian
CXX_FLAGS += -mfloat-abi=hard
CXX_FLAGS += -mfpu=fpv4-sp-d16
CXX_FLAGS += -Og
CXX_FLAGS += -fmessage-length=0
CXX_FLAGS += -fsigned-char
CXX_FLAGS += -ffunction-sections
CXX_FLAGS += -fdata-sections
CXX_FLAGS += -Wall
CXX_FLAGS += -Wextra
CXX_FLAGS += -Wno-unused-parameter
CXX_FLAGS += -Wno-missing-field-initializers
CXX_FLAGS += -Wnull-dereference

CXX_FLAGS += -g3
CXX_FLAGS += -Wstack-usage=256

CXX_FLAGS += -DWOLFSSL_USER_SETTINGS
CXX_FLAGS += -DCYASSL_USER_SETTINGS
CXX_FLAGS += -DUSE_WOLFSSL=1
CXX_FLAGS += -DHSM_TYPE_SYMM_KEY
CXX_FLAGS += -DUSE_PROV_MODULE
CXX_FLAGS += -Dget_time=azure_get_time

ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -mcpu=cortex-m4
ASM_FLAGS += -march=armv7e-m
ASM_FLAGS += -mthumb
ASM_FLAGS += -mlittle-endian
ASM_FLAGS += -mfloat-abi=hard
ASM_FLAGS += -mfpu=fpv4-sp-d16
ASM_FLAGS += -Og
ASM_FLAGS += -fmessage-length=0
ASM_FLAGS += -fsigned-char
ASM_FLAGS += -ffunction-sections
ASM_FLAGS += -fdata-sections
ASM_FLAGS += -Wall
ASM_FLAGS += -Wextra
ASM_FLAGS += -Wno-unused-parameter
ASM_FLAGS += -Wno-missing-field-initializers
ASM_FLAGS += -Wnull-dereference
ASM_FLAGS += -g3
ASM_FLAGS += -Wstack-usage=256


# Tools and Flags
###############################################################################
# Rules

.PHONY: all lst size


all: lib$(PROJECT).a


.s.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
	@$(AS) -c $(ASM_FLAGS) -o $@ $<


.S.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
	@$(AS) -c $(ASM_FLAGS) -o $@ $<

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CC) $(C_FLAGS) $(INCLUDE_PATHS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CPP) $(CXX_FLAGS) $(INCLUDE_PATHS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

lib$(PROJECT).a: $(OBJECTS)
	rm -f lib$(PROJECT).a
	$(AR) -rcs lib$(PROJECT).a $(OBJECTS)
	$(RANLIB) lib$(PROJECT).a



# Rules
###############################################################################
# Dependencies

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)
endif

# Dependencies
###############################################################################
