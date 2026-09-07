# ATG Modbus RTU Slave

Device ini berperan sebagai **slave Modbus RTU** di jalur RS-485 (via adaptor
USB-to-RS485). Master eksternal (PLC/SCADA/HMI) bisa:

- **Baca** level & temperatur 6 tangki (input register, read-only)
- **Baca/ubah** identitas serial device ini sendiri: slave ID, baudrate,
  parity, stop bit (holding register, read/write) — perubahan langsung
  diterapkan (reconnect port otomatis), tanpa restart service

Data tangki diambil dari tabel `atg_data` di `master_data.db` — tabel yang
sama yang diisi `atg_mqtt_sub.c` / `glt500` di project `flowmeter_spt_edge`.
Program ini murni pembaca (`SELECT`), **tidak pernah menulis** ke `atg_data`.

---

## Spesifikasi Transport

| Parameter | Nilai |
|---|---|
| Protokol | Modbus RTU |
| Physical layer | RS-485 half-duplex, via USB-to-RS485 |
| Default baudrate | 9600 |
| Default framing | 8 data bit, No parity, 1 stop bit (8N1) |
| Default slave address | 1 |
| Function code didukung | 0x03, 0x04, 0x06, 0x10 |

---

## Register Map

### Holding Registers — konfigurasi (Read/Write, FC 0x03 / 0x06 / 0x10)

4 register berurutan mulai address `0x0000`.

| Addr (hex) | Addr (dec) | Nama | Rentang valid | Keterangan |
|---|---|---|---|---|
| `0x0000` | 0 | `slave_id` | 1–247 | Alamat Modbus device sendiri |
| `0x0001` | 1 | `baudrate_code` | 0–7 | Indeks tabel baud (lihat bawah), **bukan** nilai baud langsung |
| `0x0002` | 2 | `parity` | 0–2 | 0=None, 1=Even, 2=Odd |
| `0x0003` | 3 | `stop_bits` | 0–1 | 0=1 stop bit, 1=2 stop bit |

Tabel `baudrate_code`:

| Code | 0 | 1 | 2 | 3 (default) | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Baud | 1200 | 2400 | 4800 | **9600** | 19200 | 38400 | 57600 | 115200 |

Data bit selalu **8**, tidak ada register untuk itu (tetap, tidak diminta).

**Perilaku write:**
- **Nilai valid** → disimpan ke DB (`TBL_MODBUS_SLAVE_SETTING`), port serial
  ditutup & dibuka ulang dengan parameter baru (tanpa restart service).
  Kalau `slave_id`/parameter serial diubah, master harus melanjutkan query
  ke alamat/parameter yang **baru** — reaksi ini memang disengaja.
- **Nilai di luar rentang** → write tetap sukses di level protokol (echo
  standar dikembalikan), tapi **tidak disimpan** — register ditulis-balik
  ke nilai lama pada siklus berikutnya. Bukan Modbus exception; deteksi
  ada di lapisan aplikasi setelah reply terkirim.

### Input Registers — data sensor (Read-only, FC 0x04)

12 register (`0x0000`–`0x000B`), 2 register per tangki × 6 tangki.

Formula alamat: `addr(level) = (tank_number - 1) * 2`, `addr(temp) = addr(level) + 1`

| Tank | Addr level | Addr temp |
|---|---|---|
| 1 | `0x0000` | `0x0001` |
| 2 | `0x0002` | `0x0003` |
| 3 | `0x0004` | `0x0005` |
| 4 | `0x0006` | `0x0007` |
| 5 | `0x0008` | `0x0009` |
| 6 | `0x000A` | `0x000B` |

| Register | Tipe | Skala | Rentang aman |
|---|---|---|---|
| `level` | `uint16` | register = `level_mm × 10` | 0 – 6553.5 mm |
| `temp` | `int16` (signed) | register = `temp_c × 100` | −327.68 – 327.67 °C |

Skala level sengaja `×10` (bukan `×100`) karena nilai sumbernya bisa
mencapai ribuan milimeter — `×100` akan overflow batas 16-bit.

Tangki yang belum punya baris di `atg_data` dibiarkan pada nilai **terakhir**
(tidak ditimpa ke 0) — supaya `0 mm`/`0 °C` tidak disalahartikan sebagai
pembacaan valid.

Write (FC 0x06/0x10) ke area ini otomatis ditolak protokol Modbus sendiri
(`ILLEGAL FUNCTION`) — bukan validasi manual aplikasi.

### Contoh frame (hex, CRC16-Modbus)

```
Baca level+temp Tank 1 (FC 0x04, addr 0x0000, qty 2):
  REQ : 01 04 00 00 00 02 71 CB
  RESP: 01 04 04 0C A7 08 CA CE A0
  -> 0x0CA7=3239 -> 323.9 mm ; 0x08CA=2250 -> 22.50 °C

Baca semua config (FC 0x03, addr 0x0000, qty 4):
  REQ : 01 03 00 00 00 04 44 09
  RESP: 01 03 08 00 01 00 03 00 00 00 00 C1 17
  -> slave_id=1, baudrate_code=3 (9600), parity=0 (None), stop_bits=0 (1 bit)

Ubah baudrate ke 38400 (FC 0x06, addr 0x0001, value 5):
  REQ : 01 06 00 01 00 05 18 09
  RESP: 01 06 00 01 00 05 18 09   (echo standar)
```

---

## Struktur Project

```
service_send_data_atg/
├── src/atg_modbus_slave.c    # program utama
├── Makefile
├── atg_modbus_slave.service  # systemd unit
└── libs/libmodbus/           # vendored, dicompile langsung dari source
```

`libmodbus` di-vendor sebagai source (bukan lib sistem) — Makefile compile
langsung `modbus.c`, `modbus-data.c`, `modbus-rtu.c` (backend RTU saja,
`modbus-tcp.c` tidak dipakai). Jadi **tidak perlu install `libmodbus-dev`**
di target.

---

## Build

Dependency yang dibutuhkan di target (Debian/BeagleBone):

```bash
sudo apt update
sudo apt install -y build-essential libsqlite3-dev
```

Build:

```bash
make selftest    # jalankan dulu -- cek logika (scaling, validasi config), tanpa hardware/DB nyata
make             # build binary: ./atg_modbus_slave
```

`make selftest` harus berakhir dengan `SELFTEST OK`. Kalau ada test yang
`assert`-nya gagal, **jangan lanjut install** — ada regresi di logika.

---

## Test manual (sebelum jadi service)

```bash
sudo ./atg_modbus_slave -c /dev/ttyUSB0 --debug
```

`-c <device>` menentukan port serial (default `/dev/ttyUSB0` kalau tidak
diisi). `--debug` menampilkan log ke terminal (tanpa flag ini, stdout
dibuang ke `/dev/null` — pola yang sama dengan service lain di project ini).

Cek device serial-nya terdeteksi dulu:

```bash
ls /dev/ttyUSB*
dmesg | grep -i usb | tail -20
```

Test baca register dari sisi master (perlu adaptor RS485 kedua yang
tersambung ke jalur A/B yang sama — device ini tidak bisa dites loopback
dengan dirinya sendiri):

```bash
sudo apt install -y mbpoll

# baca 12 input register (level+temp 6 tangki)
mbpoll -m rtu -a 1 -b 9600 -P none -s 1 -r 1 -c 12 -t 4 /dev/ttyUSB1

# baca 4 holding register (config)
mbpoll -m rtu -a 1 -b 9600 -P none -s 1 -r 1 -c 4 -t 3 /dev/ttyUSB1
```

---

## Install sebagai service

```bash
sudo make install
```

Target `install` di Makefile otomatis:
1. Copy binary ke `/usr/local/bin/atg_modbus_slave`
2. Copy `atg_modbus_slave.service` ke `/etc/systemd/system/`
3. `systemctl daemon-reload`
4. `systemctl enable --now atg_modbus_slave`

> Sebelum install, cek dulu `atg_modbus_slave.service` — `ExecStart` di
> situ hardcode `-c /dev/ttyUSB0`. Kalau device serial di unit Anda beda
> (atau lebih baik pakai path stabil `/dev/serial/by-id/...` supaya tidak
> berubah kalau ada USB lain dicolok), edit file service-nya **sebelum**
> `sudo make install`.

Cek status setelah jalan:

```bash
sudo systemctl status atg_modbus_slave
sudo journalctl -u atg_modbus_slave -f
```

### Update setelah ubah kode

```bash
make clean && make
sudo make install     # timpa binary + service, restart otomatis lewat enable --now
```

Atau kalau cuma ganti binary tanpa ubah service:

```bash
make
sudo systemctl stop atg_modbus_slave
sudo install -m 755 atg_modbus_slave /usr/local/bin/atg_modbus_slave
sudo systemctl start atg_modbus_slave
```

---

## Database

Path DB: `/home/user/Apps/master_data.db` (hardcode, sama dengan project
`flowmeter_spt_edge`).

Program ini otomatis membuat tabel `TBL_MODBUS_SLAVE_SETTING` (setting
slave ID/baudrate/parity/stop bit) kalau belum ada — **tidak perlu query
manual** untuk tabel ini.

Tabel `atg_data` (sumber level/temp) **tidak dibuat oleh program ini** —
tabel ini normalnya sudah ada karena dibuat `atg_mqtt_sub.c`/`glt500` di
project `flowmeter_spt_edge`. Kalau testing di device yang benar-benar
kosong (belum pernah ada program itu jalan), buat manual dulu:

```sql
CREATE TABLE IF NOT EXISTS atg_data (
    tank_number   INTEGER,
    datetime      TEXT,
    slocn         TEXT,
    level         DOUBLE,
    temp          DOUBLE,
    volume        DOUBLE,
    capacity_tank INTEGER
);

INSERT INTO atg_data (tank_number, datetime, slocn, level, temp, volume, capacity_tank)
VALUES (1, datetime('now'), 'FS-01', 323.9, 22.5, 16500, 40000);
-- ulangi untuk tank_number 2..6
```

---

## Exception Codes (standar Modbus)

| Kode | Nama | Kapan terjadi |
|---|---|---|
| `0x01` | ILLEGAL FUNCTION | Function code selain 0x03/0x04/0x06/0x10 |
| `0x02` | ILLEGAL DATA ADDRESS | Address di luar `0x0000`–`0x0003` (holding) atau `0x0000`–`0x000B` (input) |
| `0x03` | ILLEGAL DATA VALUE | Jumlah register pada request melebihi batas map |

Penolakan nilai konfigurasi di luar rentang (lihat bagian holding register
di atas) **bukan** exception Modbus — itu terjadi di lapisan aplikasi.
