TAPE_SDK ?= $(if $(wildcard ../../tape_sdk/tapp-build),../../tape_sdk,../tape_sdk)
OUT      ?= disintegrate.tapp

.PHONY: all clean verify

all: $(OUT)

$(OUT): disintegrate.c disintegrate_dsp.c inc/disintegrate_dsp.h assets/app_icon.png assets/mini_app_icon.png
	$(TAPE_SDK)/tapp-build . $(OUT)

verify: $(OUT)
	$(TAPE_SDK)/tools/verify-tapp.sh $(OUT)

clean:
	rm -f $(OUT)
