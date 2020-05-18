
MAKE = make

LIBKERNEL = asp_baseplatform/OBJ/STM32F767NUCLEO144_GCC/libkernel/libkernel.a
LIBWOLFSSL = wolfssl-3.15.7/Debug/libwolfssl.a
LIBZLIB = zlib-1.2.11/Debug/libzlib.a
LIBAZURE_IOTHUB = azure_iothub/Debug/libazure_iothub.a
ASP_ELF = app_iothub_client/Debug/asp.elf

$(LIBKERNEL):
	$(MAKE) -j 1 -C asp_baseplatform/OBJ/STM32F767NUCLEO144_GCC/MAC all

$(LIBWOLFSSL):
	$(MAKE) -j -C wolfssl-3.15.7 all

$(LIBZLIB):
	$(MAKE) -j -C zlib-1.2.11 all

$(LIBAZURE_IOTHUB):
	$(MAKE) -j -C azure_iothub all

$(ASP_ELF): $(LIBKERNEL) $(LIBWOLFSSL) $(LIBZLIB) $(LIBAZURE_IOTHUB)
	$(MAKE) -j 1 -C app_iothub_client/Debug all

.PHONY: refresh
refresh: 
	rm -f $(LIBKERNEL)
	rm -f $(LIBWOLFSSL)
	rm -f $(LIBZLIB)
	rm -f $(LIBAZURE_IOTHUB)
	rm -f $(ASP_ELF)

.PHONY: all
all: refresh $(ASP_ELF)

.PHONY: clean
clean:
	$(MAKE) -j -C asp_baseplatform/OBJ/STM32F767NUCLEO144_GCC/MAC clean
	$(MAKE) -j -C wolfssl-3.15.7 clean
	$(MAKE) -j -C zlib-1.2.11 clean
	$(MAKE) -j -C azure_iothub clean
	$(MAKE) -j -C app_iothub_client/Debug clean

.PHONY: realclean
realclean:
	$(MAKE) -j -C asp_baseplatform/OBJ/STM32F767NUCLEO144_GCC/MAC realclean
	$(MAKE) -j -C wolfssl-3.15.7 clean
	$(MAKE) -j -C zlib-1.2.11 clean
	$(MAKE) -j -C azure_iothub clean
	$(MAKE) -j -C app_iothub_client/Debug realclean
