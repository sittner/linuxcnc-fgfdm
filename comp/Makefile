.PHONY: all install clean

-include ../config.mk

ifeq ($(BUILDSYS),kbuild)
  BINSFX = .ko
else
  BINSFX = .so
endif

COMPS = fgaxis fgipol fgplc

SRCS = $(addsuffix .comp, $(COMPS))
BINS = $(addsuffix $(BINSFX), $(COMPS))

all: $(BINS)

install: $(BINS)
	cp $(BINS) $(DESTDIR)$(RTLIBDIR)/

%$(BINSFX): %.comp
	$(COMP) --compile $<

clean:
	rm -f *.ko *.so

