/*
 * atg_modbus_slave.c
 *
 * Device ini jadi SLAVE Modbus-RTU. Master eksternal (PLC/SCADA/HMI) bisa:
 *   - baca level & temperatur 6 tangki (INPUT REGISTERS, read-only, function 04)
 *   - baca/ubah identitas serial device ini sendiri: slave id, baudrate,
 *     parity, stop bit (HOLDING REGISTERS, read/write, function 03/06/16)
 *
 * Data tangki diambil dari tabel atg_data (master_data.db) -- tabel yang sama
 * yang diisi atg_mqtt_sub.c / glt500 di flowmeter_spt_edge. Program ini murni
 * pembaca (SELECT saja), tidak pernah menulis ke atg_data.
 *
 * Build: `make` (lihat Makefile di root project -- compile langsung dari
 * sumber vendored libs/libmodbus/src, tidak butuh libmodbus-dev di BeagleBone)
 * Self-test (murni logika, tanpa port serial/DB nyata): `make selftest`
 *
 * ==================== REGISTER MAP ====================
 *
 * HOLDING REGISTERS (function 03 read / 06 write single / 16 write multiple)
 * -- konfigurasi identitas serial device ini sendiri:
 *   addr 0  slave_id       1..247
 *   addr 1  baudrate_code  0..7  -> {1200,2400,4800,9600,19200,38400,57600,115200}
 *   addr 2  parity         0=None 1=Even 2=Odd
 *   addr 3  stop_bits      0=One  1=Two
 * Data bit selalu 8 (tidak ada register untuk itu -- tidak diminta, samakan
 * dengan default AtgConfig di project sebelah yang juga selalu D8).
 *
 * Nilai di luar rentang DITOLAK: register langsung ditulis-balik ke nilai
 * lama (bukan diterima diam-diam) supaya master tahu dari hasil baca
 * berikutnya bahwa perubahannya gagal, dan device tidak berakhir dengan
 * setting serial yang tidak valid / tidak bisa dihubungi lagi.
 *
 * Menulis salah satu dari 4 register ini men-trigger reconnect port serial
 * dengan setting baru (tutup lalu buka ulang) + simpan ke DB, tanpa perlu
 * restart service. PERHATIAN: kalau slave_id atau parameter serial diubah,
 * master harus tahu alamat/parameter barunya untuk query berikutnya --
 * ini reaksi yang memang diharapkan dari fitur "ganti ID sendiri".
 *
 * INPUT REGISTERS (function 04, read-only -- ditolak otomatis oleh protokol
 * kalau master coba menulis fungsi 06/16 ke sini, tidak perlu validasi manual)
 * -- level, temperatur, volume 6 tangki, 3 register per tangki (urut per
 *    tangki: level, temp, volume):
 *   addr (n-1)*3 + 0   level_x10   uint16, level_mm * 10  (0 .. 6553.5 mm)
 *   addr (n-1)*3 + 1   temp_x100   int16,  temp_c  * 100  (-327.68 .. 327.67 C)
 *   addr (n-1)*3 + 2   volume_l    uint16, volume liter bulat (0 .. 65535 L)
 * n = tank_number 1..6 -> addr 0..17
 *
 * ponytail: level di atg_data satuannya bisa mm (jalur glt500/atg_height)
 * atau persen (jalur atg_mqtt_sub, lihat komentar di file itu) tergantung
 * yang terakhir menulis baris tangki itu -- x10 dipilih (bukan x100) supaya
 * nilai mm yang bisa mencapai ribuan tidak overflow int16. Kalau butuh
 * presisi desimal lebih (mis. 2 angka di belakang koma persis), pindah ke
 * IEEE754 float 2-register/nilai -- perlu register map baru, jangan diam-
 * diam ubah skala di addr yang sama (breaking change untuk master).
 *
 * volume tidak di-scale (liter sudah bulat di kolom `volume`) dan cukup di
 * uint16 (0..65535 L) untuk tangki BBM biasa; kalau ada tangki >65535 L
 * pindah ke 2 register (uint32) -- jangan diam-diam ubah addr yang sudah ada.
 */

#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <time.h>
#include <stdint.h>
#include <sys/select.h>

#include <sqlite3.h>
#include <modbus.h>

#define DB_PATH         "/home/user/Apps/master_data.db"
/* Stable symlink: ttyUSB0/1 boleh loncat, path fisik USB tetap sama */
#define DEFAULT_DEVICE  "/dev/serial/by-path/platform-musb-hdrc.1-usb-0:1.2:1.0-port0"
#define NUM_SENSORS     6
#define NUM_HOLDING_REGS 4
#define NUM_INPUT_REGS  (NUM_SENSORS * 3)

#define REG_SLAVE_ID    0
#define REG_BAUD_CODE   1
#define REG_PARITY      2
#define REG_STOPBITS    3

#define LEVEL_SCALE     10.0
#define TEMP_SCALE      100.0

static const int BAUD_TABLE[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

typedef struct {
    int slave_id;
    int baud_code;
    int parity;     /* 0=None 1=Even 2=Odd */
    int stop_bits;  /* 0=One 1=Two */
} SlaveConfig;

static const SlaveConfig DEFAULT_CONFIG = { .slave_id = 1, .baud_code = 3 /*9600*/, .parity = 0, .stop_bits = 0 };

/* satu instance saja -- pola sama seperti atg_mqtt_sub.c */
static int check_prg(const char *program_name)
{
    DIR *dir;
    struct dirent *ent;
    char path[256], cmdline[256];
    FILE *fp;
    int running = 0;

    dir = opendir("/proc");
    if (!dir) return 0;

    while ((ent = readdir(dir)) != NULL) {
        if (!isdigit((unsigned char)ent->d_name[0]))
            continue;

        snprintf(path, sizeof(path), "/proc/%s/cmdline", ent->d_name);
        fp = fopen(path, "r");
        if (!fp) continue;

        if (fgets(cmdline, sizeof(cmdline), fp)) {
            if (strstr(cmdline, program_name))
                running++;
        }
        fclose(fp);
    }
    closedir(dir);
    return running;
}

static int config_valid(const SlaveConfig *c)
{
    return c->slave_id >= 1 && c->slave_id <= 247 &&
           c->baud_code >= 0 && c->baud_code <= 7 &&
           c->parity >= 0 && c->parity <= 2 &&
           c->stop_bits >= 0 && c->stop_bits <= 1;
}

static void ensure_tables(sqlite3 *db)
{
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS TBL_MODBUS_SLAVE_SETTING ("
        "ID INTEGER PRIMARY KEY, slave_id INTEGER, baudrate_code INTEGER, "
        "parity INTEGER, stop_bits INTEGER)",
        NULL, NULL, NULL);
    sqlite3_exec(db,
        "INSERT OR IGNORE INTO TBL_MODBUS_SLAVE_SETTING "
        "(ID, slave_id, baudrate_code, parity, stop_bits) VALUES (1, 1, 3, 0, 0)",
        NULL, NULL, NULL);
}

static void load_slave_config(sqlite3 *db, SlaveConfig *cfg)
{
    *cfg = DEFAULT_CONFIG;
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db,
            "SELECT slave_id, baudrate_code, parity, stop_bits "
            "FROM TBL_MODBUS_SLAVE_SETTING WHERE ID=1", -1, &stmt, NULL) == SQLITE_OK) {
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            cfg->slave_id  = sqlite3_column_int(stmt, 0);
            cfg->baud_code = sqlite3_column_int(stmt, 1);
            cfg->parity    = sqlite3_column_int(stmt, 2);
            cfg->stop_bits = sqlite3_column_int(stmt, 3);
        }
        sqlite3_finalize(stmt);
    }
    if (!config_valid(cfg)) {
        printf("[MB] setting tersimpan tidak valid, pakai default\n");
        *cfg = DEFAULT_CONFIG;
    }
}

static void save_slave_config(sqlite3 *db, const SlaveConfig *cfg)
{
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db,
            "UPDATE TBL_MODBUS_SLAVE_SETTING SET slave_id=?, baudrate_code=?, "
            "parity=?, stop_bits=? WHERE ID=1", -1, &stmt, NULL) == SQLITE_OK) {
        sqlite3_bind_int(stmt, 1, cfg->slave_id);
        sqlite3_bind_int(stmt, 2, cfg->baud_code);
        sqlite3_bind_int(stmt, 3, cfg->parity);
        sqlite3_bind_int(stmt, 4, cfg->stop_bits);
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
    }
}

/* Baca 6 tangki (tank_number 1..6) dari atg_data, isi input register.
   Tangki yang tidak ada barisnya di DB dibiarkan (register tetap nilai
   terakhir) -- lebih aman daripada menimpa jadi 0 (0 mm/0 C bisa disalah-
   artikan sebagai pembacaan valid oleh master). */
static void refresh_sensor_registers(sqlite3 *db, modbus_mapping_t *mb)
{
    sqlite3_stmt *stmt;
    const char *sql = "SELECT tank_number, level, temp, volume FROM atg_data "
                       "WHERE tank_number BETWEEN 1 AND ?";
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK)
        return;
    sqlite3_bind_int(stmt, 1, NUM_SENSORS);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        int tank = sqlite3_column_int(stmt, 0);
        double level  = sqlite3_column_double(stmt, 1);
        double temp   = sqlite3_column_double(stmt, 2);
        double volume = sqlite3_column_double(stmt, 3);
        int idx = (tank - 1) * 3;

        double level_scaled = level * LEVEL_SCALE;
        if (level_scaled < 0) level_scaled = 0;
        if (level_scaled > 65535.0) level_scaled = 65535.0;

        double temp_scaled = temp * TEMP_SCALE;
        if (temp_scaled < -32768.0) temp_scaled = -32768.0;
        if (temp_scaled > 32767.0) temp_scaled = 32767.0;

        if (volume < 0) volume = 0;
        if (volume > 65535.0) volume = 65535.0;

        mb->tab_input_registers[idx]     = (uint16_t)level_scaled;
        mb->tab_input_registers[idx + 1] = (uint16_t)(int16_t)temp_scaled;
        mb->tab_input_registers[idx + 2] = (uint16_t)volume;
    }
    sqlite3_finalize(stmt);
}

/* Tutup ctx lama (kalau ada) & buka ulang dengan setting baru. Return ctx
   baru, atau NULL kalau connect gagal (caller retry di loop berikutnya). */
static modbus_t *apply_serial_config(modbus_t *old_ctx, const char *device, const SlaveConfig *cfg)
{
    if (old_ctx) {
        modbus_close(old_ctx);
        modbus_free(old_ctx);
    }

    char parity_char = (cfg->parity == 1) ? 'E' : (cfg->parity == 2) ? 'O' : 'N';
    int stop_bits = (cfg->stop_bits == 1) ? 2 : 1;
    int baud = BAUD_TABLE[cfg->baud_code];

    modbus_t *ctx = modbus_new_rtu(device, baud, parity_char, 8, stop_bits);
    if (!ctx) {
        fprintf(stderr, "[MB] modbus_new_rtu gagal (device=%s)\n", device);
        return NULL;
    }
    modbus_set_slave(ctx, cfg->slave_id);

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "[MB] connect gagal %s: %s\n", device, modbus_strerror(errno));
        modbus_free(ctx);
        return NULL;
    }

    printf("[MB] serial diterapkan: device=%s baud=%d parity=%c stopbits=%d slave_id=%d\n",
           device, baud, parity_char, stop_bits, cfg->slave_id);
    return ctx;
}

/* Dipanggil tiap habis modbus_reply(): cek apakah 4 holding register config
   berubah dari nilai yang kita tahu terakhir. Kalau berubah & valid -> simpan
   ke DB + minta reconnect (lewat *need_reconnect). Kalau berubah tapi tidak
   valid -> tolak, tulis-balik register ke nilai lama (guard trust boundary:
   master eksternal bisa kirim apa saja). */
static void check_holding_writes(modbus_mapping_t *mb, SlaveConfig *current,
                                  sqlite3 *db, int *need_reconnect)
{
    SlaveConfig candidate = {
        .slave_id  = mb->tab_registers[REG_SLAVE_ID],
        .baud_code = mb->tab_registers[REG_BAUD_CODE],
        .parity    = mb->tab_registers[REG_PARITY],
        .stop_bits = mb->tab_registers[REG_STOPBITS],
    };

    if (candidate.slave_id == current->slave_id &&
        candidate.baud_code == current->baud_code &&
        candidate.parity == current->parity &&
        candidate.stop_bits == current->stop_bits) {
        return;  /* tidak ada perubahan */
    }

    if (!config_valid(&candidate)) {
        printf("[MB] tolak config invalid dari master (slave_id=%d baud_code=%d parity=%d stop_bits=%d), "
               "kembalikan ke nilai lama\n",
               candidate.slave_id, candidate.baud_code, candidate.parity, candidate.stop_bits);
        mb->tab_registers[REG_SLAVE_ID]  = current->slave_id;
        mb->tab_registers[REG_BAUD_CODE] = current->baud_code;
        mb->tab_registers[REG_PARITY]    = current->parity;
        mb->tab_registers[REG_STOPBITS]  = current->stop_bits;
        return;
    }

    printf("[MB] config baru diterima dari master: slave_id=%d baud_code=%d(%d) parity=%d stop_bits=%d\n",
           candidate.slave_id, candidate.baud_code, BAUD_TABLE[candidate.baud_code],
           candidate.parity, candidate.stop_bits);
    save_slave_config(db, &candidate);
    *current = candidate;
    *need_reconnect = 1;
}

#ifndef ATG_MODBUS_SELFTEST

int main(int argc, char **argv)
{
    int g_debug = 0;
    const char *device = DEFAULT_DEVICE;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--debug")) g_debug = 1;
        else if (!strcmp(argv[i], "-c") && i + 1 < argc) device = argv[++i];
    }
    if (!g_debug) freopen("/dev/null", "w", stdout);

    const char *program_name = "atg_modbus_slave";
    if (check_prg(program_name) > 1) {
        printf("Program %s sudah berjalan\n", program_name);
        return EXIT_FAILURE;
    }

    sqlite3 *db;
    if (sqlite3_open(DB_PATH, &db) != SQLITE_OK) {
        fprintf(stderr, "[MB] gagal buka DB: %s\n", sqlite3_errmsg(db));
        return EXIT_FAILURE;
    }
    sqlite3_busy_timeout(db, 5000);
    ensure_tables(db);

    SlaveConfig cfg;
    load_slave_config(db, &cfg);

    modbus_mapping_t *mb = modbus_mapping_new(0, 0, NUM_HOLDING_REGS, NUM_INPUT_REGS);
    if (!mb) {
        fprintf(stderr, "[MB] modbus_mapping_new gagal: %s\n", modbus_strerror(errno));
        sqlite3_close(db);
        return EXIT_FAILURE;
    }
    mb->tab_registers[REG_SLAVE_ID]  = cfg.slave_id;
    mb->tab_registers[REG_BAUD_CODE] = cfg.baud_code;
    mb->tab_registers[REG_PARITY]    = cfg.parity;
    mb->tab_registers[REG_STOPBITS]  = cfg.stop_bits;

    modbus_t *ctx = NULL;
    while (!ctx) {
        ctx = apply_serial_config(NULL, device, &cfg);
        if (!ctx) sleep(5);
    }

    printf("[MB] listening device=%s, %d holding reg (config) + %d input reg (%d sensor x2)\n",
           device, NUM_HOLDING_REGS, NUM_INPUT_REGS, NUM_SENSORS);

    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
    time_t last_refresh = 0;

    for (;;) {
        int fd = modbus_get_socket(ctx);
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };

        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);

        if (sel > 0 && FD_ISSET(fd, &rfds)) {
            int rc = modbus_receive(ctx, query);
            if (rc > 0) {
                modbus_reply(ctx, query, rc, mb);
                int need_reconnect = 0;
                check_holding_writes(mb, &cfg, db, &need_reconnect);
                if (need_reconnect) {
                    ctx = apply_serial_config(ctx, device, &cfg);
                    while (!ctx) {
                        sleep(5);
                        ctx = apply_serial_config(NULL, device, &cfg);
                    }
                }
            }
            /* rc == -1: frame rusak/bukan untuk slave_id kita -- libmodbus
               sudah menangani, tidak perlu aksi tambahan di sini */
        } else if (sel < 0) {
            fprintf(stderr, "[MB] select error, reconnect...\n");
            ctx = apply_serial_config(ctx, device, &cfg);
            while (!ctx) {
                sleep(5);
                ctx = apply_serial_config(NULL, device, &cfg);
            }
        }

        time_t now = time(NULL);
        if (now != last_refresh) {  /* ~1x/detik, selaras dgn timeout select di atas */
            refresh_sensor_registers(db, mb);
            last_refresh = now;
        }
    }

    modbus_mapping_free(mb);
    modbus_close(ctx);
    modbus_free(ctx);
    sqlite3_close(db);
    return 0;
}

#else  // ATG_MODBUS_SELFTEST

#include <assert.h>

int main(void)
{
    /* config_valid: batas rentang tiap field */
    SlaveConfig ok = {1, 3, 0, 0};
    assert(config_valid(&ok));
    SlaveConfig bad_slave = {0, 3, 0, 0};
    assert(!config_valid(&bad_slave));
    SlaveConfig bad_slave2 = {248, 3, 0, 0};
    assert(!config_valid(&bad_slave2));
    SlaveConfig bad_baud = {1, 8, 0, 0};
    assert(!config_valid(&bad_baud));
    SlaveConfig bad_parity = {1, 3, 3, 0};
    assert(!config_valid(&bad_parity));
    SlaveConfig bad_stop = {1, 3, 0, 2};
    assert(!config_valid(&bad_stop));
    printf("test1 ok: config_valid batas rentang\n");

    /* check_holding_writes: tidak ada perubahan -> current tetap, DB tak disentuh */
    sqlite3 *db;
    assert(sqlite3_open(":memory:", &db) == SQLITE_OK);
    ensure_tables(db);

    modbus_mapping_t *mb = modbus_mapping_new(0, 0, NUM_HOLDING_REGS, NUM_INPUT_REGS);
    assert(mb != NULL);

    SlaveConfig current = {1, 3, 0, 0};
    mb->tab_registers[REG_SLAVE_ID]  = 1;
    mb->tab_registers[REG_BAUD_CODE] = 3;
    mb->tab_registers[REG_PARITY]    = 0;
    mb->tab_registers[REG_STOPBITS]  = 0;
    int need_reconnect = 0;
    check_holding_writes(mb, &current, db, &need_reconnect);
    assert(need_reconnect == 0);
    printf("test2 ok: tidak ada perubahan -> tidak reconnect\n");

    /* master menulis slave_id valid baru -> diterima, need_reconnect=1, current update */
    mb->tab_registers[REG_SLAVE_ID] = 5;
    need_reconnect = 0;
    check_holding_writes(mb, &current, db, &need_reconnect);
    assert(need_reconnect == 1);
    assert(current.slave_id == 5);
    printf("test3 ok: slave_id valid baru diterima\n");

    /* master menulis baud_code invalid (9) -> ditolak, register ditulis-balik ke lama */
    mb->tab_registers[REG_BAUD_CODE] = 9;
    need_reconnect = 0;
    check_holding_writes(mb, &current, db, &need_reconnect);
    assert(need_reconnect == 0);
    assert(mb->tab_registers[REG_BAUD_CODE] == current.baud_code);
    printf("test4 ok: baud_code invalid ditolak & register ditulis-balik\n");

    /* refresh_sensor_registers: skala level(mm)*10 dan temp(C)*100, clamp batas int16 */
    sqlite3_exec(db,
        "CREATE TABLE atg_data (tank_number INTEGER, datetime TEXT, slocn TEXT, "
        "level DOUBLE, temp DOUBLE, volume DOUBLE, capacity_tank INTEGER)",
        NULL, NULL, NULL);
    sqlite3_exec(db, "INSERT INTO atg_data VALUES (1,'x','FS-01',323.9,22.5,29862,40000)", NULL, NULL, NULL);
    sqlite3_exec(db, "INSERT INTO atg_data VALUES (2,'x','FS-02',9999.0,-10.0,70000,40000)", NULL, NULL, NULL);
    /* tank 3 sengaja tidak ada baris -> register harus tetap nilai lama (999), bukan 0 */
    mb->tab_input_registers[6] = 999;

    refresh_sensor_registers(db, mb);
    assert(mb->tab_input_registers[0] == (uint16_t)(323.9 * 10));   /* tank1 level */
    assert((int16_t)mb->tab_input_registers[1] == (int16_t)(22.5 * 100)); /* tank1 temp */
    assert(mb->tab_input_registers[2] == 29862);                    /* tank1 volume liter */
    assert(mb->tab_input_registers[3] == (uint16_t)(9999.0 * 10));  /* tank2 level, masih muat di uint16 */
    assert((int16_t)mb->tab_input_registers[4] == (int16_t)(-10.0 * 100)); /* tank2 temp negatif */
    assert(mb->tab_input_registers[5] == 65535);                    /* tank2 volume clamp (70000 > 65535) */
    assert(mb->tab_input_registers[6] == 999);  /* tank3 kosong -> tak disentuh */
    printf("test5 ok: refresh_sensor_registers skala, volume (urut level/temp/volume per tangki), & tangki kosong dibiarkan\n");

    modbus_mapping_free(mb);
    sqlite3_close(db);
    printf("SELFTEST OK\n");
    return 0;
}

#endif
