#-*- makefile -*-
#
# Copyright 2010 Michael Ossmann
#
# This file is part of Project Ubertooth.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

#DIRS = blinky bluetooth_rxtx cc2400_test clock_test usb_test bootloader assembly_test ioatest
DIRS = blinky bluetooth_rxtx cc2400_test clock_test usb_test bootloader assembly_test

all: $(DIRS)

clean: $(DIRS:%=%_clean)

$(DIRS):
	$(MAKE) -C $@

$(DIRS:%=%_clean):
	$(MAKE) -C $(@:%_clean=%) clean

.PHONY : clean all $(DIRS)
