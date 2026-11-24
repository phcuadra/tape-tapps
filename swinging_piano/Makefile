TAPE_SDK ?= $(if $(wildcard ../../tape_sdk/tapp-build),../../tape_sdk,../tape_sdk)
OUT      ?= swinging_piano.tapp

.PHONY: all clean verify

all: $(OUT)

$(OUT): swinging_piano.c pendulum_dsp.c inc/pendulum_dsp.h assets/app_icon.png assets/mini_app_icon.png
	$(TAPE_SDK)/tapp-build . $(OUT)

verify: $(OUT)
	$(TAPE_SDK)/tools/verify-tapp.sh $(OUT)

clean:
	rm -f $(OUT)
