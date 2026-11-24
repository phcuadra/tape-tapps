TAPE_SDK ?= $(if $(wildcard ../../tape_sdk/tapp-build),../../tape_sdk,../tape_sdk)
OUT      ?= gonna_rain.tapp

.PHONY: all clean verify

all: $(OUT)

$(OUT): gonna_rain.c reich_dsp.c inc/reich_dsp.h assets/app_icon.png assets/mini_app_icon.png
	$(TAPE_SDK)/tapp-build . $(OUT)

verify: $(OUT)
	$(TAPE_SDK)/tools/verify-tapp.sh $(OUT)

clean:
	rm -f $(OUT)
