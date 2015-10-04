include ../config.mk

EXTRA_CFLAGS := $(filter-out -Wframe-larger-than=%,$(EXTRA_CFLAGS))

.PHONY: all clean install

all: fgfdm_lsnr

install: fgfdm_lsnr
	mkdir -p $(DESTDIR)$(EMC2_HOME)/bin
	cp fgfdm_lsnr $(DESTDIR)$(EMC2_HOME)/bin/

fgfdm_lsnr: fgfdm_lsnr.o net_fdm.o
	$(CC) -o $@ fgfdm_lsnr.o net_fdm.o -Wl,-rpath,$(LIBDIR) -L$(LIBDIR) -llinuxcnchal -lrt

%.o: %.c
	$(CC) -o $@ $(EXTRA_CFLAGS) -URTAPI -U__MODULE__ -DULAPI -Os -c $<

