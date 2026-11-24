TAPE_SDK ?= $(abspath ../tape_sdk)
APPS     := disintegrate sitting_in_a_room gonna_rain swinging_piano pendulum_music

.PHONY: all verify clean $(APPS)

all: $(APPS)

$(APPS):
	$(MAKE) -C $@ TAPE_SDK="$(TAPE_SDK)" verify
	@mkdir -p apps
	cp $@/$@.tapp apps/$@.tapp

verify:
	$(TAPE_SDK)/tools/verify-tapp.sh $(patsubst %,apps/%.tapp,$(APPS))

clean:
	@for d in $(APPS); do $(MAKE) -C $$d clean; done
