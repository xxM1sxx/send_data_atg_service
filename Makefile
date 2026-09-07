# libmodbus di-build langsung dari sumber vendored (libs/libmodbus/src) --
# tidak butuh libmodbus-dev/autotools di BeagleBone. Cuma modbus.c +
# modbus-data.c + modbus-rtu.c yang dipakai (backend RTU saja, modbus-tcp.c
# tidak dibutuhkan untuk slave RS485).
MODBUS_DIR = libs/libmodbus/src
MODBUS_SRC = $(MODBUS_DIR)/modbus.c $(MODBUS_DIR)/modbus-data.c $(MODBUS_DIR)/modbus-rtu.c

CC = gcc
CFLAGS = -Wall -O2 -I$(MODBUS_DIR)
LDLIBS = -lsqlite3

all: atg_modbus_slave

atg_modbus_slave: src/atg_modbus_slave.c $(MODBUS_SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

selftest: src/atg_modbus_slave.c $(MODBUS_SRC)
	$(CC) $(CFLAGS) -DATG_MODBUS_SELFTEST $^ -o atg_modbus_selftest $(LDLIBS)
	./atg_modbus_selftest

install: atg_modbus_slave
	install -m 755 atg_modbus_slave /usr/local/bin/atg_modbus_slave
	install -m 644 atg_modbus_slave.service /etc/systemd/system/atg_modbus_slave.service
	systemctl daemon-reload
	systemctl enable --now atg_modbus_slave

clean:
	rm -f atg_modbus_slave atg_modbus_selftest

.PHONY: all selftest install clean
