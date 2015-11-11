.PHONY: all configure install clean

all: configure
	@$(MAKE) -C src all
	@$(MAKE) -C comp all

clean:
	@$(MAKE) -C src clean
	@$(MAKE) -C comp clean
	rm -f config.mk config.mk.tmp

install: configure
	@$(MAKE) -C src install
	@$(MAKE) -C comp install
	@$(MAKE) -C examples install-examples

configure: config.mk

config.mk: configure.mk
	@$(MAKE) -s -f configure.mk > config.mk.tmp
	@mv config.mk.tmp config.mk

