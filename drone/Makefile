# drone/Makefile — builds dl-applier and dl-inject.
#
# Native build by default. Cross-compile via:
#     make CROSS_COMPILE=arm-linux-gnueabihf-
# or by setting CC directly:
#     make CC=arm-linux-gnueabihf-gcc

CROSS_COMPILE ?=
CC      ?= $(CROSS_COMPILE)gcc
CFLAGS  ?= -O2 -Wall -Wextra -Wformat=2 -std=c11 -D_GNU_SOURCE
LDFLAGS ?=

SRCDIR  := src
TESTDIR := ../tests/drone
OUTDIR  := build

# Modules shared by dl-applier.
APPLIER_SRCS := \
    $(SRCDIR)/dl_applier.c \
    $(SRCDIR)/dl_backend_enc.c \
    $(SRCDIR)/dl_backend_radio.c \
    $(SRCDIR)/dl_backend_tx.c \
    $(SRCDIR)/dl_ceiling.c \
    $(SRCDIR)/dl_config.c \
    $(SRCDIR)/dl_log.c \
    $(SRCDIR)/dl_osd.c \
    $(SRCDIR)/dl_watchdog.c \
    $(SRCDIR)/dl_wire.c

# dl-inject only needs the wire encoder.
INJECT_SRCS := \
    $(SRCDIR)/dl_inject.c \
    $(SRCDIR)/dl_wire.c

# Unit-test binary links whole library minus dl_applier.c's main().
TEST_SRCS := \
    $(TESTDIR)/test_main.c \
    $(TESTDIR)/test_wire.c \
    $(TESTDIR)/test_ceiling.c \
    $(TESTDIR)/test_config.c \
    $(TESTDIR)/test_watchdog.c \
    $(SRCDIR)/dl_wire.c \
    $(SRCDIR)/dl_ceiling.c \
    $(SRCDIR)/dl_config.c \
    $(SRCDIR)/dl_log.c \
    $(SRCDIR)/dl_watchdog.c

APPLIER := $(OUTDIR)/dl-applier
INJECT  := $(OUTDIR)/dl-inject
TESTBIN := $(OUTDIR)/dl-test

.PHONY: all clean test install

all: $(APPLIER) $(INJECT)

$(OUTDIR):
	mkdir -p $(OUTDIR)

$(APPLIER): $(APPLIER_SRCS) | $(OUTDIR)
	$(CC) $(CFLAGS) -I$(SRCDIR) -o $@ $(APPLIER_SRCS) $(LDFLAGS)

$(INJECT): $(INJECT_SRCS) | $(OUTDIR)
	$(CC) $(CFLAGS) -I$(SRCDIR) -o $@ $(INJECT_SRCS) $(LDFLAGS)

$(TESTBIN): $(TEST_SRCS) | $(OUTDIR)
	$(CC) $(CFLAGS) -I$(SRCDIR) -I$(TESTDIR) -o $@ $(TEST_SRCS) $(LDFLAGS)

test: $(TESTBIN)
	./$(TESTBIN)

clean:
	rm -rf $(OUTDIR)

PREFIX ?= /usr/local
install: all
	install -D -m 0755 $(APPLIER) $(DESTDIR)$(PREFIX)/bin/dl-applier
	install -D -m 0755 $(INJECT)  $(DESTDIR)$(PREFIX)/bin/dl-inject
