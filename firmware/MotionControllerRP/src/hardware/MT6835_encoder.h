// --------------------------------------------------------------------------------------
// This code is a modified version of the MT6835 Driver from the SimpleFOC project
// being distributed under the MIT liscence as well. Thank you SimpleFOC !
// --------------------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"

#define MT6835_CPR (1<<21)

#define MT6835_OP_READ  0b0011
#define MT6835_OP_WRITE 0b0110
#define MT6835_OP_PROG  0b1100
#define MT6835_OP_ZERO  0b0101
#define MT6835_OP_ANGLE 0b1010

#define MT6835_CMD_MASK  0b111100000000000000000000
#define MT6835_ADDR_MASK 0b000011111111111100000000
#define MT6835_DATA_MASK 0b000000000000000011111111

#define MT6835_STATUS_OVERSPEED 0x01
#define MT6835_STATUS_WEAKFIELD 0x02
#define MT6835_STATUS_UNDERVOLT 0x04
#define MT6835_CRC_ERROR 0x08
#define MT6835_CRC_ERROR_RETURN -1

#define MT6835_WRITE_ACK 0x55

#define MT6835_REG_USERID 0x001

#define MT6835_REG_ANGLE1 0x003
#define MT6835_REG_ANGLE2 0x004
#define MT6835_REG_ANGLE3 0x005
#define MT6835_REG_ANGLE4 0x006

#define MT6835_REG_ABZ_RES1 0x007
#define MT6835_REG_ABZ_RES2 0x008

#define MT6835_REG_ZERO1 0x009
#define MT6835_REG_ZERO2 0x00A

#define MT6835_REG_OPTS0 0x00A
#define MT6835_REG_OPTS1 0x00B
#define MT6835_REG_OPTS2 0x00C
#define MT6835_REG_OPTS3 0x00D
#define MT6835_REG_OPTS4 0x00E
#define MT6835_REG_OPTS5 0x011

// NLC table, 192 bytes
#define MT6835_REG_NLC_BASE 0x013
#define MT6835_REG_CAL_STATUS 0x113

//*** DATATYPES **********************************************************************************/

union MT6835ABZRes {
	struct {
		uint8_t ab_swap:1;
		uint8_t abz_off:1;
		uint8_t abz_res_low:6;
	};
	uint8_t reg;
};

union MT6835Options0 {
	struct {
		uint8_t z_pul_wid:3;
		uint8_t z_edge:1;
		uint8_t zero_pos_low:4;
	};
	uint8_t reg;
};

union MT6835Options1 {
	struct {
		uint8_t uvw_res:4;
		uint8_t uvw_off:1;
		uint8_t uvw_mux:1;
		uint8_t z_phase:2;
	};
	uint8_t reg;
};

union MT6835Options2 {
	struct {
		uint8_t pwm_sel:3;
		uint8_t pwm_pol:1;
		uint8_t pwm_fq:1;
		uint8_t nlc_en:1;
		uint8_t reserved:2;
	};
	uint8_t reg;
};

union MT6835Options3 {
	struct {
		uint8_t hyst:3;
		uint8_t rot_dir:1;
		uint8_t reserved:4;
	};
	uint8_t reg;
};

union MT6835Options4 {
	struct {
		uint8_t reserved:4;
		uint8_t autocal_freq:3;
		uint8_t gpio_ds:1;
	};
	uint8_t reg;
};

union MT6835Options5 {
	struct {
		uint8_t bw:3;
		uint8_t reserved:5;
	};
	uint8_t reg;
};

union MT6835Command {
	struct {
		uint32_t unused:8;
		uint32_t data:8;
		uint32_t addr:12;
		uint32_t cmd:4;
	};
	uint32_t val;
};

//*** CLASS ************************************************************************************/

class MT6835Encoder {
  public:
    typedef int32_t AbsRawAngleType;

  public:
      static constexpr float RAW_TO_ANGLE = (2.0f*3.14159265358979323846f)/float(MT6835_CPR);

      // use this to setup a HW spi. It can then be used by multiple instances of MT6835
      static void setup_spi(spi_inst_t* spi, uint pin_sck, uint pin_mosi, uint pin_miso, int32_t baudrate_hz);

      // Constructor: pass SPI instance (spi0 or spi1), CS pin
      MT6835Encoder(spi_inst_t *spi, int32_t cs_pin);
      virtual ~MT6835Encoder();

      bool init(uint8_t bandwidth=0x5, uint8_t hysteresis=0x4);
      bool is_connected();
      bool is_initialized();

      void reset_abs_angle(int32_t abs_raw_angle=0);  // resets the total revolutions of abs angle
      void reset_abs_angle_period();                  // Brings abs angle into [0..2pi)
      float read_abs_angle();                         // returns the absolute angle in radians
      float get_last_abs_angle() const;               // returns the last read abs angle
      AbsRawAngleType read_abs_angle_raw();           // returns the absolute angle in raw counts
      AbsRawAngleType get_last_abs_raw_angle() const; // returns the last read abs raw angle

      int32_t get_rawcounts_per_rev();                // returns the number of raw counts per revolution

      uint8_t get_bandwidth();
      void set_bandwidth(uint8_t bw);

      uint8_t get_hysteresis();
      void set_hysteresis(uint8_t hyst);

      uint8_t get_rotation_direction();
      void set_rotation_direction(uint8_t dir);

      uint16_t get_abz_resolution();
      void set_abz_resolution(uint16_t res);

      bool is_abz_enabled();
      void set_abz_enabled(bool enabled);

      bool is_ab_swapped();
      void set_ab_swapped(bool swapped);

      uint16_t get_zero_position();
      void set_zero_position(uint16_t pos);

      MT6835Options1 get_options1();
      void set_options1(MT6835Options1 opts);

      MT6835Options2 get_options2();
      void set_options2(MT6835Options2 opts);

      MT6835Options3 get_options3();
      void set_options3(MT6835Options3 opts);

      MT6835Options4 get_options4();
      void set_options4(MT6835Options4 opts);

      uint8_t get_status();
      void set_crc_enabled(bool enable);
      bool is_crc_enabled();
      uint32_t get_crc_error_count(bool reset=false);

      uint8_t get_calibration_status();

      bool set_zero_from_current_position();
      bool write_eeprom();  // takes ~6s to complete after calling

      // Measures how often the chip crc actually validates and enables crc checking only
      // if it does. Some chips always return a crc of 0 which would reject every read.
      bool autodetect_crc(int sample_count=64, float max_error_rate=0.02f);

      // Configures the read rejection filter.
      //  retries                 : extra spi reads attempted when a sample is rejected
      //  max_counts_per_us       : fastest plausible motion [raw counts per us], 0 disables the gate
      //  max_consecutive_rejects : after this many rejects in a row the next sample is force
      //                            accepted (resync) and the read fault flag is raised
      void set_read_filter(int retries, float max_counts_per_us, uint32_t max_consecutive_rejects);

      // True if the most recent read_abs_angle_raw() produced a trusted sample. If false the
      // returned position is the last good one and the caller should skip this cycle.
      bool last_read_valid() const { return last_read_ok; }

      // Sticky flag: a resync happened, so the absolute position may have jumped.
      bool has_read_fault() const { return read_fault; }
      void clear_read_fault() { read_fault = false; consecutive_bad_reads = 0; }

      struct Diagnostics {
        uint32_t slip_count      = 0;  // implausible jumps - period probably lost
        int32_t  max_abs_delta   = 0;  // largest |d| seen [counts]
        uint32_t max_read_gap_us = 0;  // longest interval between two reads
        uint64_t first_slip_us   = 0;  // timestamp of the first slip
        int32_t  first_slip_delta= 0;  // the delta that triggered it
        uint32_t crc_rejects     = 0;  // samples dropped because the crc did not match
        uint32_t range_rejects   = 0;  // samples dropped because the delta was not plausible
        uint32_t retries         = 0;  // extra spi reads caused by rejected samples
        uint32_t recovered_reads = 0;  // reads that a retry managed to rescue
        uint32_t skipped_reads   = 0;  // reads given up on - position was held
        uint32_t resyncs         = 0;  // force accepted samples after repeated rejects
        uint32_t max_consecutive_bad = 0; // longest run of rejected reads
        int32_t  last_reject_delta = 0;   // delta of the most recent rejected sample
      };

      const Diagnostics& get_diagnostics() const { return diag; }
      void reset_diagnostics() { 
        diag = Diagnostics(); 
        last_read_time_us = 0; 
        consecutive_bad_reads = 0;
      }
      bool has_slipped() const { return diag.slip_count > 0; }
      

      bool check_crc = false;
      int32_t cs_pin;
  private:
      bool initialized=false;
      spi_inst_t *spi;
      
      uint8_t last_status = 0;
      uint8_t last_crc = 0;
      uint32_t crc_error_count = 0;

      int32_t last_raw_angle = 0;
      AbsRawAngleType abs_raw_angle = 0;

      AbsRawAngleType update_abs_raw_angle(AbsRawAngleType raw_angle);

      // reads one angle frame from the chip, returns false on a crc mismatch
      bool read_raw_frame(int32_t& raw_angle);

      // checks if the step from 'last_raw_angle' to 'raw_angle' could physically happen
      // within 'gap_us'. Always rejects steps beyond half a period, those are ambiguous.
      bool is_delta_plausible(int32_t raw_angle, uint32_t gap_us) const;

      void spi_begin_transaction();
      void spi_transfer(uint8_t* data, size_t length);
      void spi_end_transaction();

      void transfer_24(MT6835Command *out_value);
      uint8_t read_register(uint16_t reg);
      bool write_register(uint16_t reg, uint8_t value);
      uint8_t calc_crc(uint32_t angle, uint8_t status);

      Diagnostics diag;
      int32_t slip_threshold = MT6835_CPR/4;
      uint64_t last_read_time_us = 0;

      // read rejection filter
      int      read_retries = 0;              // 0 = no retry, sample is skipped right away
      float    max_counts_per_us = 0.0f;      // 0 = plausibility gate disabled
      uint32_t max_consecutive_rejects = 0;   // 0 = never force a resync
      uint32_t consecutive_bad_reads = 0;
      uint64_t last_good_read_time_us = 0;    // timestamp of the last accepted sample
      bool     last_read_ok = true;
      bool     read_fault = false;
};